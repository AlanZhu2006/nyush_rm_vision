# OpenVINO on NVIDIA Jetson

This guide shows how to install **OpenVINO** on **NVIDIA Jetson (aarch64)** using **Python `venv` + `pip`**, and configure your environment so CMake projects using:

```cmake
find_package(OpenVINO REQUIRED)
```

can successfully locate:

* `OpenVINOConfig.cmake` or
* `openvino-config.cmake`


## 1) System prerequisites

Install basic build tools and Python venv support:

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip build-essential cmake git pkg-config
```

Optional but commonly needed by C++ projects:

```bash
sudo apt install -y libopencv-dev
```


## 2) Create a venv and install OpenVINO via pip

### 2.1 Create and activate the virtual environment

```bash
python3 -m venv ~/venvs/spvision
source ~/venvs/spvision/bin/activate
python -m pip install --upgrade pip wheel setuptools
```

### 2.2 Install OpenVINO

It’s usually best to pin a version for reproducibility:

```bash
pip install "openvino==2024.6.0"
```

### 2.3 Verify OpenVINO works

```bash
python -c "from openvino import Core; print(Core().available_devices)"
```

On Jetson you’ll typically see `['CPU']`. That’s expected.

## 3) Make CMake find OpenVINO (the important part)

### 3.1 Locate `OpenVINOConfig.cmake` inside the venv

While the venv is activated, run:

```bash
python - << 'PY'
import site, os, glob
paths = site.getsitepackages()
cands = []
for p in paths:
    cands += glob.glob(os.path.join(p, "**", "OpenVINOConfig.cmake"), recursive=True)
    cands += glob.glob(os.path.join(p, "**", "openvino-config.cmake"), recursive=True)

print("site-packages:")
for p in paths:
    print(" ", p)

print("\nfound config:")
for x in cands:
    print(" ", x)
PY
```

You should see something like:

```
.../venvs/spvision/lib/python3.10/site-packages/openvino/cmake/OpenVINOConfig.cmake
```

### 3.2 Set `OpenVINO_DIR` correctly (**directory, not file**)

Correct:

```bash
export OpenVINO_DIR="$VIRTUAL_ENV/lib/python3.10/site-packages/openvino/cmake"
test -f "$OpenVINO_DIR/OpenVINOConfig.cmake" && echo "OK" || echo "NOT FOUND"
```

## 4) Use it in your CMake project (recommended workflow)

### 4.1 Do NOT hardcode OpenVINO paths in `CMakeLists.txt`

Keep:

```cmake
find_package(OpenVINO REQUIRED)
```

Then configure from the terminal (preferred):

```bash
source ~/venvs/spvision/bin/activate
export OpenVINO_DIR="$VIRTUAL_ENV/lib/python3.10/site-packages/openvino/cmake"

rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOpenVINO_DIR="$OpenVINO_DIR"
make -C build/ -j`nproc`
```