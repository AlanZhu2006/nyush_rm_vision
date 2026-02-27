set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default:
  @just --list

default_build_dir := "build"
default_cmake_generator := "Unix Makefiles"
default_config := "configs/odin.yaml"
default_calib_config := "configs/calibration.yaml"
default_calib_folder := "assets/img_with_q"
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

# Configure only when build tree is missing/invalid.
ensure-configured build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir:
  @need_configure="false"; \
  reason=""; \
  if [[ ! -f "{{build_dir}}/CMakeCache.txt" ]]; then \
    need_configure="true"; \
    reason="missing CMakeCache.txt"; \
  elif [[ ! -f "{{build_dir}}/Makefile" ]]; then \
    need_configure="true"; \
    reason="missing Makefile (likely stale non-Makefiles build tree)"; \
  else \
    generator=""; \
    while IFS= read -r line; do \
      if [[ "${line}" == CMAKE_GENERATOR:INTERNAL=* ]]; then \
        generator="${line#CMAKE_GENERATOR:INTERNAL=}"; \
        break; \
      fi; \
    done < "{{build_dir}}/CMakeCache.txt"; \
    if [[ -n "${generator}" && "${generator}" != "{{default_cmake_generator}}" ]]; then \
      need_configure="true"; \
      reason="generator mismatch: ${generator}"; \
    fi; \
  fi; \
  if [[ "${need_configure}" == "true" ]]; then \
    echo "[ensure-configured] ${reason}, running cmake..."; \
    just cmake "{{build_dir}}" "{{profile}}" "{{use_tensorrt}}" "{{openvino_dir}}"; \
  else \
    echo "[ensure-configured] build tree is valid, skip cmake"; \
  fi

# One-step build (cmake only when build tree is missing/invalid).
build target="" build_dir=default_build_dir profile=default_profile use_tensorrt=default_use_tensorrt openvino_dir=default_openvino_dir jobs=default_jobs:
  just ensure-configured "{{build_dir}}" "{{profile}}" "{{use_tensorrt}}" "{{openvino_dir}}"
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

# Unified calibration launcher. See `just calibrate-help` for detailed usage.
calibrate name="capture" arg="" arg2="" arg3="" arg4="" arg5="" arg6="" build_dir=default_build_dir config=default_calib_config input_folder=default_calib_folder output_folder=default_calib_folder:
  @cfg="{{config}}"; in_folder="{{input_folder}}"; out_folder="{{output_folder}}"; cli_mode="false"; in_path=""; out_path=""; start=""; end=""; \
  for token in "{{arg}}" "{{arg2}}" "{{arg3}}" "{{arg4}}" "{{arg5}}" "{{arg6}}"; do \
    if [[ -z "${token}" ]]; then \
      continue; \
    elif [[ "${token}" == --config=* ]]; then \
      cfg="${token#--config=}"; \
    elif [[ "${token}" == --input-folder=* ]]; then \
      in_folder="${token#--input-folder=}"; \
    elif [[ "${token}" == --output-folder=* ]]; then \
      out_folder="${token#--output-folder=}"; \
    elif [[ "${token}" == --input-path=* ]]; then \
      in_path="${token#--input-path=}"; \
    elif [[ "${token}" == --output-path=* ]]; then \
      out_path="${token#--output-path=}"; \
    elif [[ "${token}" == --start=* ]]; then \
      start="${token#--start=}"; \
    elif [[ "${token}" == --end=* ]]; then \
      end="${token#--end=}"; \
    elif [[ "${token}" == --cli-mode ]]; then \
      cli_mode="true"; \
    fi; \
  done; \
  case "{{name}}" in \
    capture) \
      if [[ "${cli_mode}" == "true" ]]; then \
        ./{{build_dir}}/capture "${cfg}" --output-folder="${out_folder}" --cli-mode; \
      else \
        ./{{build_dir}}/capture "${cfg}" --output-folder="${out_folder}"; \
      fi ;; \
    camera) ./{{build_dir}}/calibrate_camera "${in_folder}" --config-path="${cfg}" ;; \
    handeye) ./{{build_dir}}/calibrate_handeye "${in_folder}" --config-path="${cfg}" ;; \
    robotworld-handeye|rwhandeye) ./{{build_dir}}/calibrate_robotworld_handeye "${in_folder}" --config-path="${cfg}" ;; \
    split-video) \
      if [[ -z "${in_path}" ]]; then \
        echo "[calibrate] split-video requires --input-path=<path>" >&2; \
        exit 1; \
      fi; \
      if [[ -z "${out_path}" ]]; then \
        out_path="${in_path}_split"; \
      fi; \
      if [[ -n "${start}" && -n "${end}" ]]; then \
        ./{{build_dir}}/split_video "${in_path}" --output-path="${out_path}" --start-index="${start}" --end-index="${end}"; \
      elif [[ -n "${start}" ]]; then \
        ./{{build_dir}}/split_video "${in_path}" --output-path="${out_path}" --start-index="${start}"; \
      elif [[ -n "${end}" ]]; then \
        ./{{build_dir}}/split_video "${in_path}" --output-path="${out_path}" --end-index="${end}"; \
      else \
        ./{{build_dir}}/split_video "${in_path}" --output-path="${out_path}"; \
      fi ;; \
    *) echo "[calibrate] Unknown target '{{name}}'. Supported: capture, camera, handeye, robotworld-handeye, split-video. See: just calibrate-help" >&2; exit 1 ;; \
  esac

# Print detailed help for unified calibration launcher.
calibrate-help:
  @echo "just calibrate <name> [flags]"
  @echo ""
  @echo "Available calibration names:"
  @echo "  capture            - Capture images + IMU quaternion files"
  @echo "  camera             - Calibrate camera intrinsics from captured images"
  @echo "  handeye            - Calibrate camera-to-gimbal extrinsics"
  @echo "  robotworld-handeye - Joint robot-world/hand-eye calibration"
  @echo "  split-video        - Split a recorded .avi/.txt by frame range"
  @echo ""
  @echo "Common flags:"
  @echo "  --config=<path>          calibration yaml (default: configs/calibration.yaml)"
  @echo "  --input-folder=<path>    image folder (default: assets/img_with_q)"
  @echo "  --output-folder=<path>   output folder for capture"
  @echo "  --cli-mode               capture: press s/q in terminal"
  @echo ""
  @echo "split-video flags:"
  @echo "  --input-path=<path>      required, base path without .avi/.txt suffix"
  @echo "  --output-path=<path>     optional, default: <input-path>_split"
  @echo "  --start=<index>          optional start frame index"
  @echo "  --end=<index>            optional end frame index"
  @echo ""
  @echo "Examples:"
  @echo "  just calibrate capture --cli-mode"
  @echo "  just calibrate camera --input-folder=assets/img_with_q"
  @echo "  just calibrate handeye --input-folder=assets/img_with_q"
  @echo "  just calibrate robotworld-handeye --input-folder=assets/img_with_q"
  @echo "  just calibrate split-video --input-path=records/demo/run1 --start=300 --end=900"

# Unified test launcher. See `just test-help` for detailed usage.
test name="imu" arg="" arg2="" arg3="" arg4="" config=default_config build_dir=default_build_dir jobs=default_jobs:
  @cfg="{{config}}"; display="false"; send="false"; fire="false"; gimbal_opts=""; local_display=""; local_xauth=""; test_target=""; \
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
    elif [[ "${token}" == "--local-display" ]]; then \
      local_display=":0"; \
    elif [[ "${token}" == --local-display=* ]]; then \
      local_display="${token#--local-display=}"; \
    elif [[ "${token}" == --xauthority=* ]]; then \
      local_xauth="${token#--xauthority=}"; \
    elif [[ "${token}" =~ ^:[0-9]+$ ]]; then \
      local_display="${token}"; \
    elif [[ "${token}" =~ ^[0-9]+$ ]]; then \
      local_display=":${token}"; \
    elif [[ "${token}" == "-v" || "${token}" == "--verbose" || "${token}" == --distance=* || "${token}" == --radius=* || "${token}" == --height=* || "${token}" == --height-amp=* || "${token}" == --omega=* || "${token}" == --hz=* ]]; then \
      gimbal_opts="${gimbal_opts} ${token}"; \
    else \
      cfg="${token}"; \
    fi; \
  done; \
  if [[ -n "${local_display}" && -z "${local_xauth}" ]]; then \
    local_xauth="${HOME}/.Xauthority"; \
  fi; \
  just ensure-configured "{{build_dir}}" "{{default_profile}}" "{{default_use_tensorrt}}" "{{default_openvino_dir}}"; \
  case "{{name}}" in \
    imu) test_target="imu_communication_test" ;; \
    camera) test_target="camera_test" ;; \
    detect) test_target="auto_aim_camera_test" ;; \
    gimbal) test_target="gimbal_test" ;; \
    *) echo "[test] Unknown test '{{name}}'. Supported: imu, camera, detect, gimbal. See: just test-help" >&2; exit 1 ;; \
  esac; \
  command cmake --build {{build_dir}} --target "${test_target}" -j{{jobs}}; \
  run_with_display() { \
    if [[ -n "${local_display}" ]]; then \
      if [[ -n "${local_xauth}" ]]; then \
        env DISPLAY="${local_display}" XAUTHORITY="${local_xauth}" "$@"; \
      else \
        env DISPLAY="${local_display}" "$@"; \
      fi; \
    else \
      "$@"; \
    fi; \
  }; \
  case "{{name}}" in \
    imu) run_with_display ./{{build_dir}}/imu_communication_test "${cfg}" ;; \
    camera) if [[ "${display}" == "true" ]]; then run_with_display ./{{build_dir}}/camera_test "${cfg}" -d; else run_with_display ./{{build_dir}}/camera_test "${cfg}"; fi ;; \
    detect) if [[ "${send}" == "true" ]]; then run_with_display ./{{build_dir}}/auto_aim_camera_test "${cfg}" --send; else run_with_display ./{{build_dir}}/auto_aim_camera_test "${cfg}"; fi ;; \
    gimbal) if [[ "${fire}" == "true" ]]; then run_with_display ./{{build_dir}}/gimbal_test -f "${cfg}" ${gimbal_opts}; else run_with_display ./{{build_dir}}/gimbal_test "${cfg}" ${gimbal_opts}; fi ;; \
  esac

# Print detailed help for unified test launcher.
test-help:
  @echo "just test <name> [config_path] [flags]"
  @echo ""
  @echo "Available test names:"
  @echo "  imu     - IMU communication link test (read + periodic command send)"
  @echo "  camera  - Industrial camera stream test"
  @echo "  detect  - GUI detect/track test with command overlay"
  @echo "  gimbal  - Vision-like target simulation and protocol send test"
  @echo ""
  @echo "Flags:"
  @echo "  -d, --display    camera: show GUI window"
  @echo "  -s, --send       detect: send command to lower machine"
  @echo "  --no-send        detect: disable send (default)"
  @echo "  -f, --fire       gimbal: enable fire pulse"
  @echo "  --distance=..    gimbal: target center distance in meters"
  @echo "  --radius=..      gimbal: horizontal motion radius in meters"
  @echo "  --height=..      gimbal: target center height in meters"
  @echo "  --height-amp=..  gimbal: height oscillation amplitude in meters"
  @echo "  --omega=..       gimbal: angular speed in rad/s"
  @echo "  --hz=..          gimbal: send frequency in Hz"
  @echo "  -v, --verbose    gimbal: enable periodic TX/RX logs"
  @echo "  --local-display[=:N] force GUI to physical X display (default :0)"
  @echo "  :N or N          shorthand display selector, e.g. '-d 0' => DISPLAY=:0"
  @echo "  --xauthority=..      Xauthority path for --local-display (default: $HOME/.Xauthority)"
  @echo "  (interactive)    gimbal: aim <yaw_deg> [pitch_deg], sim on/off, send on/off"
  @echo ""
  @echo "Examples:"
  @echo "  just test imu"
  @echo "  just test camera -d"
  @echo "  just test camera -d 0"
  @echo "  just test camera -d --local-display"
  @echo "  just test detect --local-display=:0 --xauthority=/home/nyu/.Xauthority"
  @echo "  just test detect configs/odin.yaml --send"
  @echo "  just test gimbal --fire"
  @echo "  just test gimbal --distance=4 --radius=0.8 --omega=1.5 --hz=120"
  @echo "  just test gimbal --verbose"
