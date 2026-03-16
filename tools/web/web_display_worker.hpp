#ifndef TOOLS__WEB__WEB_DISPLAY_WORKER_HPP
#define TOOLS__WEB__WEB_DISPLAY_WORKER_HPP

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>
#include <set>

#include "httplib.h"
#include "tools/logger.hpp"

namespace tools
{

struct FrameData
{
  cv::Mat image;
  std::string metadata_json;
};

class WebDisplayWorker
{
 public:
  explicit WebDisplayWorker(std::string title, int port = 8080, int jpeg_quality = 80, int max_width = 960)
    : title_(std::move(title)), port_(port), jpeg_quality_(jpeg_quality), max_width_(max_width)
  {
  }

  ~WebDisplayWorker()
  {
    stop();
  }

  void start()
  {
    server_thread_ = std::thread([this]() { run_server(); });
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested_ = true;
    }
    cond_.notify_all();
    
    if (server_) {
      server_->stop();
    }
    
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

  void submit(const cv::Mat & frame, const std::string & metadata_json = "")
  {
    if (failed_.load()) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Just copy the frame, resize will be done in streaming thread
      frame.copyTo(latest_frame_.image);
      latest_frame_.metadata_json = metadata_json;
      has_frame_ = true;
      frame_counter_++;
    }
    cond_.notify_all();
  }

  bool failed() const
  {
    return failed_.load();
  }

  int port() const
  {
    return port_;
  }

 private:
  void run_server()
  {
    try {
      server_ = std::make_unique<httplib::Server>();

      // Serve the HTML page
      server_->Get("/", [this](const httplib::Request & req, httplib::Response & res) {
        res.set_content(generate_html_page(), "text/html");
      });

      // WebSocket-like endpoint using Server-Sent Events (simpler alternative)
      // For metadata updates
      server_->Get("/metadata", [this](const httplib::Request & req, httplib::Response & res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!latest_frame_.metadata_json.empty()) {
          res.set_content(latest_frame_.metadata_json, "application/json");
        } else {
          res.set_content("{}", "application/json");
        }
      });

      // MJPEG stream endpoint
      server_->Get("/stream.mjpeg", [this](const httplib::Request & req, httplib::Response & res) {
        res.set_header("Cache-Control", "no-cache, no-store, must-revalidate");
        res.set_header("Pragma", "no-cache");
        res.set_header("Expires", "0");
        res.set_chunked_content_provider(
          "multipart/x-mixed-replace; boundary=frame",
          [this](size_t offset, httplib::DataSink & sink) {
            uint64_t last_frame_id = 0;
            while (!stop_requested_) {
              cv::Mat frame;
              uint64_t current_frame_id = 0;
              {
                std::unique_lock<std::mutex> lock(mutex_);
                // Wait for a new frame
                cond_.wait_for(lock, std::chrono::milliseconds(100), [this, last_frame_id]() {
                  return stop_requested_ || (has_frame_ && frame_counter_ > last_frame_id);
                });
                if (stop_requested_) break;
                if (has_frame_) {
                  latest_frame_.image.copyTo(frame);
                  current_frame_id = frame_counter_.load();
                }
              }

              if (!frame.empty() && current_frame_id > last_frame_id) {
                last_frame_id = current_frame_id;
                
                // Resize frame if needed (done in streaming thread, not blocking main loop)
                cv::Mat resized_frame;
                if (frame.cols > max_width_) {
                  double scale = static_cast<double>(max_width_) / static_cast<double>(frame.cols);
                  int target_w = max_width_;
                  int target_h = static_cast<int>(frame.rows * scale);
                  cv::resize(frame, resized_frame, cv::Size(target_w, target_h), 0, 0, cv::INTER_AREA);
                } else {
                  resized_frame = frame;
                }
                
                std::vector<uchar> buf;
                std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
                if (cv::imencode(".jpg", resized_frame, buf, params)) {
                  std::string header = "--frame\r\n"
                                       "Content-Type: image/jpeg\r\n"
                                       "Content-Length: " +
                                       std::to_string(buf.size()) + "\r\n\r\n";
                  if (!sink.write(header.c_str(), header.size())) break;
                  if (!sink.write(reinterpret_cast<const char *>(buf.data()), buf.size())) break;
                  if (!sink.write("\r\n", 2)) break;
                }
              } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
              }
            }
            return false;  // Stop chunked transfer
          });
      });

      tools::logger()->info("Web display server starting on http://localhost:{}", port_);
      tools::logger()->info("Open http://localhost:{} in your browser to view the stream", port_);

      if (!server_->listen("127.0.0.1", port_)) {
        tools::logger()->error("Failed to start web server on port {}", port_);
        failed_.store(true);
      }
    } catch (const std::exception & e) {
      tools::logger()->error("Web display server error: {}", e.what());
      failed_.store(true);
    }
  }

  std::string generate_html_page()
  {
    return R"(<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>)" + title_ +
           R"(</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
            background: #f5f5f5;
            padding: 10px;
        }
        .container {
            max-width: 1600px;
            margin: 0 auto;
        }
        .status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 6px 12px;
            border-radius: 4px;
            font-size: 12px;
            font-weight: 500;
            z-index: 1000;
            box-shadow: 0 2px 8px rgba(0,0,0,0.15);
        }
        .status.connected { background: #4caf50; color: white; }
        .status.disconnected { background: #f44336; color: white; }
        .main {
            display: grid;
            grid-template-columns: 1fr 420px;
            gap: 10px;
        }
        .video-panel {
            background: white;
            border-radius: 8px;
            padding: 15px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        #stream {
            width: 100%;
            height: auto;
            display: block;
            border-radius: 4px;
            background: #000;
        }
        .data-panel {
            background: white;
            border-radius: 8px;
            padding: 12px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            max-height: calc(100vh - 20px);
            overflow-y: auto;
        }
        .angle-viz {
            background: white;
            border-radius: 8px;
            padding: 15px;
            margin-bottom: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .angle-viz h3 {
            font-size: 13px;
            font-weight: 600;
            color: #666;
            margin-bottom: 12px;
        }
        .angle-display {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            margin-bottom: 12px;
        }
        .angle-box {
            background: #f8f9fa;
            border-radius: 6px;
            padding: 10px;
            text-align: center;
        }
        .angle-box .label {
            font-size: 11px;
            color: #666;
            margin-bottom: 4px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .angle-box .value {
            font-size: 20px;
            font-weight: 700;
            font-family: monospace;
        }
        .angle-box.tx { border-left: 4px solid #2196F3; }
        .angle-box.rx { border-left: 4px solid #4caf50; }
        .angle-box.tx .value { color: #2196F3; }
        .angle-box.rx .value { color: #4caf50; }
        .angle-diff {
            font-size: 11px;
            color: #999;
            text-align: center;
            margin-top: 8px;
        }
        .angle-diff.warning {
            color: #f57c00;
            font-weight: 600;
        }
        .data-section {
            margin-bottom: 15px;
        }
        .data-section h3 {
            font-size: 12px;
            font-weight: 600;
            color: #666;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .data-grid {
            display: grid;
            gap: 6px;
        }
        .data-item {
            display: flex;
            justify-content: space-between;
            padding: 6px 10px;
            background: #f8f9fa;
            border-radius: 4px;
            font-size: 12px;
        }
        .data-label {
            color: #666;
            font-weight: 500;
        }
        .data-value {
            color: #333;
            font-weight: 600;
            font-family: monospace;
        }
        @media (max-width: 1200px) {
            .main {
                grid-template-columns: 1fr;
            }
            .data-panel {
                max-height: none;
            }
        }
    </style>
</head>
<body>
    <div class="status disconnected" id="status">●</div>
    <div class="container">
        <div class="main">
            <div class="video-panel">
                <img id="stream" src="/stream.mjpeg?t=)" +
           std::to_string(std::time(nullptr)) + R"(" alt="Video Stream">
            </div>
            <div>
                <div class="angle-viz">
                    <h3>Gimbal Angles</h3>
                    <div id="yaw-display"></div>
                    <div id="pitch-display"></div>
                </div>
                <div class="data-panel">
                    <div class="data-section">
                        <h3>System</h3>
                        <div class="data-grid" id="system-data"></div>
                    </div>
                    <div class="data-section">
                        <h3>Control</h3>
                        <div class="data-grid" id="control-data"></div>
                    </div>
                    <div class="data-section">
                        <h3>Weapon</h3>
                        <div class="data-grid" id="weapon-data"></div>
                    </div>
                </div>
            </div>
        </div>
    </div>
<script>
        const statusEl = document.getElementById('status');
        const streamImg = document.getElementById('stream');
        const yawDisplay = document.getElementById('yaw-display');
        const pitchDisplay = document.getElementById('pitch-display');
        const systemData = document.getElementById('system-data');
        const controlData = document.getElementById('control-data');
        const weaponData = document.getElementById('weapon-data');

        streamImg.onload = () => statusEl.className = 'status connected';
        streamImg.onerror = () => {
            statusEl.className = 'status disconnected';
            setTimeout(() => streamImg.src = '/stream.mjpeg?t=' + Date.now(), 1000);
        };

        function renderAngleBox(txVal, rxVal, label) {
            const diff = Math.abs(txVal - rxVal);
            const diffClass = diff > 1 ? 'warning' : '';
            return `
                <div class="angle-display">
                    <div class="angle-box tx">
                        <div class="label">TX ${label}</div>
                        <div class="value">${txVal.toFixed(1)}°</div>
                    </div>
                    <div class="angle-box rx">
                        <div class="label">RX ${label}</div>
                        <div class="value">${rxVal.toFixed(1)}°</div>
                    </div>
                </div>
                <div class="angle-diff ${diffClass}">Δ ${diff.toFixed(1)}°</div>
            `;
        }

        function renderData(data) {
            const txYaw = parseFloat(data['TX Yaw (deg)']) || 0;
            const txPitch = parseFloat(data['TX Pitch (deg)']) || 0;
            const rxYaw = parseFloat(data['RX Yaw (deg)']) || 0;
            const rxPitch = parseFloat(data['RX Pitch (deg)']) || 0;

            yawDisplay.innerHTML = renderAngleBox(txYaw, rxYaw, 'Yaw');
            pitchDisplay.innerHTML = renderAngleBox(txPitch, rxPitch, 'Pitch');

            systemData.innerHTML = `
                <div class="data-item"><span class="data-label">FPS</span><span class="data-value">${data['FPS'] || '-'}</span></div>
                <div class="data-item"><span class="data-label">Mode</span><span class="data-value">${data['Mode'] || '-'}</span></div>
                <div class="data-item"><span class="data-label">Armors</span><span class="data-value">${data['Armors Detected'] || '0'}</span></div>
                <div class="data-item"><span class="data-label">Capture Age</span><span class="data-value">${data['Capture Age (ms)'] || '-'} ms</span></div>
            `;

            controlData.innerHTML = `
                <div class="data-item"><span class="data-label">TX Enable</span><span class="data-value">${data['TX Enabled']}</span></div>
                <div class="data-item"><span class="data-label">TX Control</span><span class="data-value">${data['TX Control']}</span></div>
                <div class="data-item"><span class="data-label">Hold</span><span class="data-value">${data['TX Hold Applied']}</span></div>
                <div class="data-item"><span class="data-label">Slew</span><span class="data-value">${data['TX Slew Applied']}</span></div>
                <div class="data-item"><span class="data-label">Yaw Vel</span><span class="data-value">${data['RX Yaw Vel (deg/s)']}°/s</span></div>
                <div class="data-item"><span class="data-label">Pitch Vel</span><span class="data-value">${data['RX Pitch Vel (deg/s)']}°/s</span></div>
            `;

            weaponData.innerHTML = `
                <div class="data-item"><span class="data-label">Shoot</span><span class="data-value">${data['TX Shoot']}</span></div>
                <div class="data-item"><span class="data-label">Bullet Speed</span><span class="data-value">${data['RX Bullet Speed (m/s)']} m/s</span></div>
                <div class="data-item"><span class="data-label">Bullet Count</span><span class="data-value">${data['RX Bullet Count']}</span></div>
            `;
        }

        function updateMetadata() {
            fetch('/metadata')
                .then(r => r.json())
                .then(renderData)
                .catch(err => console.error(err));
        }

        setInterval(updateMetadata, 100);
        updateMetadata();
    </script>
</body>
</html>
)";
  }

  std::string title_;
  int port_;
  int jpeg_quality_;
  int max_width_;
  std::unique_ptr<httplib::Server> server_;
  std::thread server_thread_;
  std::mutex mutex_;
  std::condition_variable cond_;
  FrameData latest_frame_;
  bool has_frame_ = false;
  bool stop_requested_ = false;
  std::atomic<bool> failed_{false};
  std::atomic<uint64_t> frame_counter_{0};
};

}  // namespace tools

#endif  // TOOLS__WEB__WEB_DISPLAY_WORKER_HPP
