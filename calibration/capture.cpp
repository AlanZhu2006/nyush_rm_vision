#include <fmt/core.h>

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }"
  "{cli-mode        |                          | 使用CLI模式（无GUI）}";

// Terminal input handler for CLI mode
class TerminalInput {
public:
  TerminalInput() {
    tcgetattr(STDIN_FILENO, &old_tio_);
    struct termios new_tio = old_tio_;
    new_tio.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    // Set stdin to non-blocking
    old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
  }

  ~TerminalInput() {
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_);
  }

  // Check if a key was pressed, returns the key or -1 if no key
  int get_key() {
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) {
      return c;
    }
    return -1;
  }

private:
  struct termios old_tio_;
  int old_flags_;
};

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & can, const std::string & output_folder, bool cli_mode)
{
  io::Gimbal cboard(config_path);
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;

  // Setup terminal input for CLI control (works with or without GUI)
  std::unique_ptr<TerminalInput> term_input;
  if (cli_mode) {
    term_input = std::make_unique<TerminalInput>();
    tools::logger()->info("CLI control: Press 's' in terminal to save, 'q' to quit");
  }

  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    // 在图像上显示欧拉角，用来判断imuabs系的xyz正方向，同时判断imu是否存在零漂
    auto img_with_ypr = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_ypr, fmt::format("Z {:.2f}", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Y {:.2f}", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("X {:.2f}", zyx[2]), {40, 120}, {0, 0, 255});

    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findCirclesGrid(img, cv::Size(10, 7), centers_2d);  // 默认是对称圆点图案
    cv::drawChessboardCorners(img_with_ypr, cv::Size(10, 7), centers_2d, success);  // 显示识别结果
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("Camera View", img_with_ypr);

    int key = -1;
    if (cli_mode) {
      // CLI control mode: check terminal input, but still show GUI window
      cv::waitKey(1);  // Process window events
      key = term_input->get_key();
      usleep(1000);  // Small delay to avoid busy waiting
    } else {
      // Window control mode: use OpenCV window for key input
      key = cv::waitKey(1);
    }

    // 按"s"保存图片和对应四元数，按"q"退出程序
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, q);
    tools::logger()->info("[{}] Saved in {}", count, output_folder);
  }

  // 离开该作用域时，camera和cboard会自动关闭
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");
  bool cli_mode = cli.has("cli-mode");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  tools::logger()->info("默认标定板尺寸为10列7行");
  if (cli_mode) {
    tools::logger()->info("GUI display with CLI control: Watch the window, press 's'/'q' in terminal");
  } else {
    tools::logger()->info("GUI control mode: Press 's'/'q' in the OpenCV window");
  }

  // 主循环，保存图片和对应四元数
  capture_loop(config_path, "can0", output_folder, cli_mode);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}
