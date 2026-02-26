set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default:
  @just --list

default_build_dir := "build"
default_cmake_generator := "Unix Makefiles"
default_config := "configs/odin.yaml"
default_jobs := "4"
default_profile := "Release"
default_use_tensorrt := "OFF"
default_openvino_dir := ""

# Configure project (auto picks OpenVINO_DIR if available).
cmake build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir:
  @if [[ -f "{{build_dir}}/CMakeCache.txt" ]]; then \
    generator=""; \
    while IFS= read -r line; do \
      if [[ "${line}" == CMAKE_GENERATOR:INTERNAL=* ]]; then \
        generator="${line#CMAKE_GENERATOR:INTERNAL=}"; \
        break; \
      fi; \
    done < "{{build_dir}}/CMakeCache.txt"; \
    if [[ -n "${generator}" && "${generator}" != "{{default_cmake_generator}}" ]]; then \
      echo "[cmake] Switching generator from '${generator}' to {{default_cmake_generator}}, resetting cache..."; \
      rm -f "{{build_dir}}/CMakeCache.txt"; \
      rm -rf "{{build_dir}}/CMakeFiles"; \
      rm -f "{{build_dir}}/build.ninja"; \
    fi; \
  fi; \
  ov_dir="{{openvino_dir}}"; \
  if [[ -z "${ov_dir}" && -n "${OpenVINO_DIR:-}" ]]; then \
    ov_dir="${OpenVINO_DIR}"; \
  fi; \
  if [[ -z "${ov_dir}" && -d "/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake" ]]; then \
    ov_dir="/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake"; \
  fi; \
  if [[ -n "${ov_dir}" ]]; then \
    echo "[cmake] Using OpenVINO_DIR=${ov_dir}"; \
    command cmake -G "{{default_cmake_generator}}" -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}} -DOpenVINO_DIR="${ov_dir}"; \
  else \
    echo "[cmake] OpenVINO_DIR not set, trying system default"; \
    command cmake -G "{{default_cmake_generator}}" -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}}; \
  fi

# Build all targets or one specific target.
make target="" build_dir=default_build_dir jobs=default_jobs:
  @if [[ -n "{{target}}" ]]; then \
    command cmake --build {{build_dir}} --target {{target}} -j{{jobs}}; \
  else \
    command cmake --build {{build_dir}} -j{{jobs}}; \
  fi

# One-step configure + build (always runs cmake first).
build target="" build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir jobs=default_jobs:
  just cmake "{{build_dir}}" "{{profile}}" "{{use_tensorrt}}" "{{openvino_dir}}"
  @if [[ -n "{{target}}" ]]; then \
    command cmake --build {{build_dir}} --target {{target}} -j{{jobs}}; \
  else \
    command cmake --build {{build_dir}} -j{{jobs}}; \
  fi

# Force full reconfigure and rebuild all/one target.
rebuild target="" build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir jobs=default_jobs:
  @if [[ -d "{{build_dir}}" ]]; then \
    rm -f "{{build_dir}}/CMakeCache.txt"; \
    rm -rf "{{build_dir}}/CMakeFiles"; \
    rm -f "{{build_dir}}/build.ninja"; \
    rm -f "{{build_dir}}/Makefile"; \
  fi
  just cmake "{{build_dir}}" "{{profile}}" "{{use_tensorrt}}" "{{openvino_dir}}"
  @if [[ -n "{{target}}" ]]; then \
    command cmake --build {{build_dir}} --target {{target}} -j{{jobs}}; \
  else \
    command cmake --build {{build_dir}} -j{{jobs}}; \
  fi

# Clean build artifacts.
clean build_dir=default_build_dir:
  @if [[ -d "{{build_dir}}" ]]; then \
    command cmake --build {{build_dir}} --target clean; \
  fi

# Unified runtime launcher. See `just run-help` for detailed usage.
run name="mt" arg="" arg2="" arg3="" config=default_config build_dir=default_build_dir amp="" signal="triangle_wave":
  @cfg="{{config}}"; a="{{amp}}"; sig="{{signal}}"; \
  for token in "{{arg}}" "{{arg2}}" "{{arg3}}"; do \
    if [[ -z "${token}" ]]; then \
      continue; \
    elif [[ "${token}" == --amp=* ]]; then \
      a="${token#--amp=}"; \
    elif [[ "${token}" == --signal=* ]]; then \
      sig="${token#--signal=}"; \
    else \
      cfg="${token}"; \
    fi; \
  done; \
  case "{{name}}" in \
    standard) ./{{build_dir}}/standard "${cfg}" ;; \
    mt) ./{{build_dir}}/mt_standard "${cfg}" ;; \
    response-yaw) if [[ -z "${a}" ]]; then a="3"; fi; ./{{build_dir}}/gimbal_response_test -a "${a}" -m "${sig}" -x yaw "${cfg}" ;; \
    response-pitch) if [[ -z "${a}" ]]; then a="2"; fi; ./{{build_dir}}/gimbal_response_test -a "${a}" -m "${sig}" -x pitch "${cfg}" ;; \
    *) echo "[run] Unknown run target '{{name}}'. Supported: standard, mt, response-yaw, response-pitch. See: just run-help" >&2; exit 1 ;; \
  esac

# Print detailed help for unified runtime launcher.
run-help:
  @echo "just run <name> [config_path] [flags]"
  @echo ""
  @echo "Available run names:"
  @echo "  standard        - Single-thread auto-aim runtime entry"
  @echo "  mt              - Multithread auto-aim runtime entry (recommended)"
  @echo "  response-yaw    - Yaw-axis gimbal response test"
  @echo "  response-pitch  - Pitch-axis gimbal response test"
  @echo ""
  @echo "Flags (for response-*):"
  @echo "  --amp=<value>      signal amplitude (default: 3 for yaw, 2 for pitch)"
  @echo "  --signal=<mode>    signal mode, e.g. triangle_wave/step/circle"
  @echo ""
  @echo "Examples:"
  @echo "  just run standard"
  @echo "  just run mt"
  @echo "  just run mt configs/odin.yaml"
  @echo "  just run response-yaw configs/odin.yaml --amp=4 --signal=step"
  @echo "  just run response-pitch --signal=triangle_wave"

# Unified test launcher. See `just test-help` for detailed usage.
test name="imu" arg="" arg2="" arg3="" arg4="" config=default_config build_dir=default_build_dir:
  @cfg="{{config}}"; display="false"; send="false"; fire="false"; \
  for token in "{{arg}}" "{{arg2}}" "{{arg3}}" "{{arg4}}"; do \
    if [[ -z "${token}" ]]; then \
      continue; \
    elif [[ "${token}" == "-d" || "${token}" == "-display" || "${token}" == "--display" ]]; then \
      display="true"; \
    elif [[ "${token}" == "-s" || "${token}" == "--send" ]]; then \
      send="true"; \
    elif [[ "${token}" == "-f" || "${token}" == "--fire" ]]; then \
      fire="true"; \
    elif [[ "${token}" == "--no-send" ]]; then \
      send="false"; \
    else \
      cfg="${token}"; \
    fi; \
  done; \
  case "{{name}}" in \
    imu) ./{{build_dir}}/imu_communication_test "${cfg}" ;; \
    camera) if [[ "${display}" == "true" ]]; then ./{{build_dir}}/camera_test "${cfg}" -d; else ./{{build_dir}}/camera_test "${cfg}"; fi ;; \
    detect) if [[ "${send}" == "true" ]]; then ./{{build_dir}}/auto_aim_camera_test "${cfg}" --send; else ./{{build_dir}}/auto_aim_camera_test "${cfg}"; fi ;; \
    gimbal) if [[ "${fire}" == "true" ]]; then ./{{build_dir}}/gimbal_test -f "${cfg}"; else ./{{build_dir}}/gimbal_test "${cfg}"; fi ;; \
    *) echo "[test] Unknown test '{{name}}'. Supported: imu, camera, detect, gimbal. See: just test-help" >&2; exit 1 ;; \
  esac

# Print detailed help for unified test launcher.
test-help:
  @echo "just test <name> [config_path] [flags]"
  @echo ""
  @echo "Available test names:"
  @echo "  imu     - IMU communication link test (read + periodic command send)"
  @echo "  camera  - Industrial camera stream test"
  @echo "  detect  - GUI detect/track test with command overlay"
  @echo "  gimbal  - Fixed command gimbal send test"
  @echo ""
  @echo "Flags:"
  @echo "  -d, --display    camera: show GUI window"
  @echo "  -s, --send       detect: send command to lower machine"
  @echo "  --no-send        detect: disable send (default)"
  @echo "  -f, --fire       gimbal: enable fire pulse"
  @echo ""
  @echo "Examples:"
  @echo "  just test imu"
  @echo "  just test camera -d"
  @echo "  just test detect configs/odin.yaml --send"
  @echo "  just test gimbal --fire"
