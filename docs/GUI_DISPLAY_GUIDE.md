# GUI Display on Physical Monitor

## Overview

When running GUI applications via SSH or remote sessions, windows may not appear on the physical monitor. This guide shows how to display GUI windows on the connected physical monitor.

## Setup

The physical monitor is connected to display **`:1`**.

### Option 1: Per-Command (One-time)

```bash
DISPLAY=:1 your_gui_command
```

### Option 2: Export for Current Session (Recommended)

Set once in your terminal, then all GUI commands will use the physical monitor:

```bash
export DISPLAY=:1
```

Now run GUI commands directly without the prefix:

```bash
just test camera -d
python3 camera_viewer.py
zenity --info --text="Test"
```

**Note:** This setting only lasts for the current terminal session.

## Usage Examples

### Command Line

```bash
# Show info dialog
DISPLAY=:1 zenity --info --text="Your message" --title="Title"

# Open image viewer
DISPLAY=:1 eog image.jpg

# Run Python GUI script
DISPLAY=:1 python3 camera_viewer.py

# Run any OpenCV application
DISPLAY=:1 just test camera -d
```

### In Python Code

```python
import os
os.environ['DISPLAY'] = ':1'

# Now cv2.imshow will appear on physical monitor
import cv2
cv2.imshow('Camera Feed', frame)
cv2.waitKey(1)
```

### In C++ Code

```cpp
#include <cstdlib>

int main() {
    // Set display before creating windows
    setenv("DISPLAY", ":1", 1);

    // Now cv::imshow will appear on physical monitor
    cv::imshow("Camera Feed", frame);
    cv::waitKey(1);
}
```

### In Shell Scripts

```bash
#!/bin/bash
export DISPLAY=:1

# All GUI commands below will use physical monitor
just test camera -d
python3 viewer.py
```

## Verification

Test if the display is accessible:

```bash
DISPLAY=:1 xdpyinfo | head -5
```

If successful, you'll see display information. If you get an authorization error, you may need to run GUI applications as the same user who is logged into the graphical session.

## Common Applications

- **OpenCV**: `cv2.imshow()` windows
- **Image viewers**: `eog`, `feh`
- **Plot windows**: `matplotlib` with GUI backend
- **Dialog boxes**: `zenity`, `yad`
- **Any X11/Wayland application**

## Notes

- The default display for SSH sessions is typically `:10.0` (VNC/remote)
- Display `:1` is the physical monitor (HDMI/DisplayPort)
- You can check available displays with: `ls /tmp/.X11-unix/`
