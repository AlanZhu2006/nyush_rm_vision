set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default:
  @just --list

default_build_dir := "build"
default_config := "configs/odin.yaml"
default_jobs := "12"
default_profile := "Release"
default_use_tensorrt := "OFF"
default_openvino_dir := ""

# Configure project (auto picks OpenVINO_DIR if available).
cmake build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir:
  @ov_dir="{{openvino_dir}}"; \
  if [[ -z "${ov_dir}" && -n "${OpenVINO_DIR:-}" ]]; then \
    ov_dir="${OpenVINO_DIR}"; \
  fi; \
  if [[ -z "${ov_dir}" && -d "/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake" ]]; then \
    ov_dir="/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake"; \
  fi; \
  if [[ -n "${ov_dir}" ]]; then \
    echo "[cmake] Using OpenVINO_DIR=${ov_dir}"; \
    command cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}} -DOpenVINO_DIR="${ov_dir}"; \
  else \
    echo "[cmake] OpenVINO_DIR not set, trying system default"; \
    command cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}}; \
  fi

# Build all targets or one specific target.
make target="" build_dir=default_build_dir jobs=default_jobs:
  @if [[ -n "{{target}}" ]]; then \
    command cmake --build {{build_dir}} --target {{target}} -j{{jobs}}; \
  else \
    command cmake --build {{build_dir}} -j{{jobs}}; \
  fi

# Build core targets used most often.
make-core build_dir=default_build_dir jobs=default_jobs:
  command cmake --build {{build_dir}} --target standard mt_standard imu_communication_test gimbal_response_test gimbal_test -j{{jobs}}

# One-step configure + build (runs cmake first time automatically).
build target="" build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir jobs=default_jobs:
  @if [[ ! -f "{{build_dir}}/CMakeCache.txt" ]]; then \
    ov_dir="{{openvino_dir}}"; \
    if [[ -z "${ov_dir}" && -n "${OpenVINO_DIR:-}" ]]; then \
      ov_dir="${OpenVINO_DIR}"; \
    fi; \
    if [[ -z "${ov_dir}" && -d "/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake" ]]; then \
      ov_dir="/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake"; \
    fi; \
    if [[ -n "${ov_dir}" ]]; then \
      echo "[cmake] Using OpenVINO_DIR=${ov_dir}"; \
      command cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}} -DOpenVINO_DIR="${ov_dir}"; \
    else \
      echo "[cmake] OpenVINO_DIR not set, trying system default"; \
      command cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}}; \
    fi; \
  fi
  @if [[ -n "{{target}}" ]]; then \
    command cmake --build {{build_dir}} --target {{target}} -j{{jobs}}; \
  else \
    command cmake --build {{build_dir}} -j{{jobs}}; \
  fi

# Reconfigure then build core targets.
rebuild-core build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir jobs=default_jobs:
  @ov_dir="{{openvino_dir}}"; \
  if [[ -z "${ov_dir}" && -n "${OpenVINO_DIR:-}" ]]; then \
    ov_dir="${OpenVINO_DIR}"; \
  fi; \
  if [[ -z "${ov_dir}" && -d "/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake" ]]; then \
    ov_dir="/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake"; \
  fi; \
  if [[ -n "${ov_dir}" ]]; then \
    echo "[cmake] Using OpenVINO_DIR=${ov_dir}"; \
    command cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}} -DOpenVINO_DIR="${ov_dir}"; \
  else \
    echo "[cmake] OpenVINO_DIR not set, trying system default"; \
    command cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}}; \
  fi
  command cmake --build {{build_dir}} --target standard mt_standard imu_communication_test gimbal_response_test gimbal_test -j{{jobs}}

# Clean build artifacts.
clean build_dir=default_build_dir:
  @if [[ -d "{{build_dir}}" ]]; then \
    command cmake --build {{build_dir}} --target clean; \
  fi

# Run single-thread auto-aim entry.
run-standard config=default_config build_dir=default_build_dir:
  ./{{build_dir}}/standard {{config}}

# Run multithread auto-aim entry (recommended).
run-mt config=default_config build_dir=default_build_dir:
  ./{{build_dir}}/mt_standard {{config}}

# Run IMU link test (requires hardware).
run-imu-test config=default_config build_dir=default_build_dir:
  ./{{build_dir}}/imu_communication_test {{config}}

# Unified test entry (e.g. `just test imu`, `just test camera -display`).
test name="imu" arg="" config=default_config build_dir=default_build_dir:
  @cfg="{{config}}"; display="false"; \
  if [[ "{{arg}}" == "-display" || "{{arg}}" == "--display" ]]; then \
    display="true"; \
  elif [[ -n "{{arg}}" ]]; then \
    cfg="{{arg}}"; \
  fi; \
  case "{{name}}" in \
    imu) ./{{build_dir}}/imu_communication_test "${cfg}" ;; \
    camera) if [[ "${display}" == "true" ]]; then ./{{build_dir}}/camera_test "${cfg}" -d; else ./{{build_dir}}/camera_test "${cfg}"; fi ;; \
    *) echo "[test] Unknown test '{{name}}'. Supported: imu, camera" >&2; exit 1 ;; \
  esac

# Run gimbal response test on yaw axis.
run-response-yaw config=default_config build_dir=default_build_dir amp="3" signal="triangle_wave":
  ./{{build_dir}}/gimbal_response_test -a {{amp}} -m {{signal}} -x yaw {{config}}

# Run gimbal response test on pitch axis.
run-response-pitch config=default_config build_dir=default_build_dir amp="2" signal="triangle_wave":
  ./{{build_dir}}/gimbal_response_test -a {{amp}} -m {{signal}} -x pitch {{config}}
