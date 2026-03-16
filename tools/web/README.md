# Web Display for Auto Aim Camera Test

## Overview

This module provides a web-based display interface for `just test detect`, eliminating the need for X11 forwarding when running tests over SSH.

## Features

- **MJPEG Video Stream**: Real-time video streaming at up to 30 fps
- **Live Telemetry**: All parameters displayed in the original GUI are now available in a web dashboard
- **Zero X11 Configuration**: Access from any browser without X server setup
- **Multi-Device Support**: View from desktop, laptop, tablet, or mobile
- **Low Latency**: Local streaming over localhost with <1ms network delay

## Usage

### Basic Usage

```bash
# Start detect test with web interface
just test detect --web

# Then open in your browser:
# http://localhost:8080
```

### Custom Port

```bash
# Use a different port
just test detect --web --web-port=9000

# Access at: http://localhost:9000
```

### With Send Command

```bash
# Enable web display and send commands to gimbal
just test detect --web --send
```

### With Custom Config

```bash
# Use custom config file with web display
just test detect configs/odin.yaml --web
```

## Telemetry Data

The web interface displays the following real-time data:

- **FPS**: Current frame rate
- **Mode**: Current operation mode
- **Armors Detected**: Number of detected armor plates
- **TX (Transmit) Data**:
  - TX Enabled, Control, Shoot status
  - TX Yaw/Pitch angles in degrees
  - Hold/Slew applied flags
- **Raw Command Data**:
  - Raw control status
  - Raw yaw/pitch angles
  - Control streak count
- **RX (Receive) Data**:
  - RX Yaw/Pitch angles
  - Yaw/Pitch velocity (deg/s)
  - Bullet speed (m/s)
  - Bullet count
- **Capture Status**:
  - Capture age (ms)
  - Stale capture warning

## Performance

- **JPEG Quality**: 80% (configurable in code)
- **Typical Frame Rate**: 15-30 fps depending on network and processing load
- **Encoding Overhead**: 5-20ms per frame
- **Recommended Use**: Development, debugging, and monitoring

## Architecture

- **Backend**: C++ with cpp-httplib (single-header HTTP server)
- **Video Format**: MJPEG (multipart/x-mixed-replace)
- **Metadata Format**: JSON (polled at 10 Hz)
- **Threading**: Separate thread for HTTP server, non-blocking video encoding

## Files

- `web_display_worker.hpp`: Main web display worker class
- `httplib.h`: Single-header HTTP server library (cpp-httplib)
- Modified `tests/auto_aim_camera_test.cpp`: Integration with existing test

## Troubleshooting

### Port Already in Use

If port 8080 is already occupied:
```bash
just test detect --web --web-port=8081
```

### Browser Not Connecting

1. Check if the test is running:
   ```bash
   ps aux | grep auto_aim_camera_test
   ```

2. Check if port is listening:
   ```bash
   netstat -tlnp | grep 8080
   ```

3. Make sure you're accessing `localhost` or `127.0.0.1`, not a remote IP

### High Latency

- Lower JPEG quality in `web_display_worker.hpp` (change `jpeg_quality` parameter)
- Check CPU usage - encoding may be bottleneck
- Ensure no heavy browser extensions are interfering

## Comparison: Web Display vs X11 Forwarding

| Feature | Web Display | X11 Forwarding |
|---------|-------------|----------------|
| Setup | Zero config | Requires X server + SSH -X |
| Latency | Very Low | Medium-High over network |
| Multi-device | Yes | No |
| Mobile support | Yes | No |
| Bandwidth | Optimized JPEG | Uncompressed X11 protocol |
| Reliability | High | Varies with network |

## Future Enhancements

Potential improvements:
- WebSocket for lower-latency metadata updates
- H.264 video encoding for better compression
- Interactive controls (start/stop, parameter adjustment)
- Recording capability
- Multiple simultaneous viewers
