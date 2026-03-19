set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default:
  @just --list

default_build_dir := "build"
default_cmake_generator := "Unix Makefiles"
default_config := "configs/odin.yaml"
default_calib_config := "configs/calibration.yaml"
default_calib_folder := "assets/img_with_q"
default_jobs := `nproc`
default_profile := "Release"
default_use_tensorrt := "ON"
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
  if [[ -z "${ov_dir}" ]]; then \
    for candidate in /opt/intel/openvino_*/runtime/cmake; do \
      if [[ -d "${candidate}" ]]; then \
        ov_dir="${candidate}"; \
        break; \
      fi; \
    done; \
  fi; \
  launcher_args=(); \
  if command -v ccache >/dev/null 2>&1; then \
    launcher_args=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache); \
    echo "[cmake] Using ccache compiler launcher"; \
  fi; \
  if [[ -n "${ov_dir}" ]]; then \
    echo "[cmake] Using OpenVINO_DIR=${ov_dir}"; \
    command cmake -G "{{default_cmake_generator}}" -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}} "${launcher_args[@]}" -DOpenVINO_DIR="${ov_dir}"; \
  else \
    echo "[cmake] OpenVINO_DIR not set, trying system default"; \
    command cmake -G "{{default_cmake_generator}}" -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{profile}} -DUSE_TENSORRT={{use_tensorrt}} "${launcher_args[@]}"; \
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
  requested_tensorrt="{{use_tensorrt}}"; \
  requested_tensorrt="${requested_tensorrt^^}"; \
  requested_ov_dir="{{openvino_dir}}"; \
  if [[ -z "${requested_ov_dir}" && -n "${OpenVINO_DIR:-}" ]]; then \
    requested_ov_dir="${OpenVINO_DIR}"; \
  fi; \
  if [[ -z "${requested_ov_dir}" && -d "/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake" ]]; then \
    requested_ov_dir="/home/nyu/venvs/spvision/lib/python3.10/site-packages/openvino/cmake"; \
  fi; \
  if [[ -z "${requested_ov_dir}" ]]; then \
    for candidate in /opt/intel/openvino_*/runtime/cmake; do \
      if [[ -d "${candidate}" ]]; then \
        requested_ov_dir="${candidate}"; \
        break; \
      fi; \
    done; \
  fi; \
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
    cache_tensorrt=""; \
    while IFS= read -r line; do \
      if [[ "${line}" == USE_TENSORRT:BOOL=* ]]; then \
        cache_tensorrt="${line#USE_TENSORRT:BOOL=}"; \
        break; \
      fi; \
    done < "{{build_dir}}/CMakeCache.txt"; \
    cache_tensorrt="${cache_tensorrt^^}"; \
    if [[ "${need_configure}" != "true" && -n "${cache_tensorrt}" && "${cache_tensorrt}" != "${requested_tensorrt}" ]]; then \
      need_configure="true"; \
      reason="USE_TENSORRT mismatch: cache=${cache_tensorrt}, requested=${requested_tensorrt}"; \
    fi; \
    cache_openvino=""; \
    while IFS= read -r line; do \
      if [[ "${line}" == OpenVINO_DIR:PATH=* ]]; then \
        cache_openvino="${line#OpenVINO_DIR:PATH=}"; \
        break; \
      fi; \
    done < "{{build_dir}}/CMakeCache.txt"; \
    if [[ "${need_configure}" != "true" && -n "${requested_ov_dir}" ]]; then \
      if [[ -z "${cache_openvino}" || "${cache_openvino}" == *-NOTFOUND || "${cache_openvino}" != "${requested_ov_dir}" ]]; then \
        need_configure="true"; \
        reason="OpenVINO_DIR mismatch: cache=${cache_openvino:-<empty>}, requested=${requested_ov_dir}"; \
      fi; \
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

# Hik SDK runtime selection for just launchers.
# - auto: use system MVS on Intel NUCs, otherwise keep loader defaults
# - default: do not override loader behavior
# - system: preload MVS from /opt/MVS/lib/64 or MVCAM_LD_PRELOAD
# - custom: preload MVCAM_LD_PRELOAD only
# Set via env, e.g. HIK_SDK_RUNTIME=system just test camera.
hik_sdk_runtime := "${HIK_SDK_RUNTIME:-auto}"

# Unified runtime launcher. See `just run-help` for detailed usage.
run name="mt" arg="" arg2="" arg3="" config=default_config build_dir=default_build_dir amp="" signal="triangle_wave":
  @cfg="{{config}}"; a="{{amp}}"; sig="{{signal}}"; hik_sdk_runtime="{{hik_sdk_runtime}}"; mvs_lib="${MVCAM_LD_PRELOAD:-}"; mvs_runenv="${MVCAM_COMMON_RUNENV:-/opt/MVS/lib}"; machine="$(uname -m)"; host_name="$(hostname 2>/dev/null || true)"; product_name="$(cat /sys/devices/virtual/dmi/id/product_name 2>/dev/null || true)"; \
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
  if [[ "${hik_sdk_runtime}" == "auto" ]]; then \
    if [[ "${machine}" == "x86_64" && ( "${host_name,,}" == *nuc* || "${product_name,,}" == *nuc* ) ]]; then \
      hik_sdk_runtime="system"; \
    else \
      hik_sdk_runtime="default"; \
    fi; \
  fi; \
  if [[ -z "${mvs_lib}" && "${hik_sdk_runtime}" == "system" ]]; then \
    for candidate in /opt/MVS/lib/64/libMvCameraControl.so*; do \
      if [[ -f "${candidate}" ]]; then \
        mvs_lib="${candidate}"; \
        break; \
      fi; \
    done; \
  fi; \
  run_command() { \
    if [[ -n "${mvs_lib}" && ( "${hik_sdk_runtime}" == "system" || "${hik_sdk_runtime}" == "custom" ) ]]; then \
      env LD_PRELOAD="${mvs_lib}${LD_PRELOAD:+:${LD_PRELOAD}}" MVCAM_COMMON_RUNENV="${mvs_runenv}" "$@"; \
    else \
      "$@"; \
    fi; \
  }; \
  case "{{name}}" in \
    standard) run_command ./{{build_dir}}/standard "${cfg}" ;; \
    mt) run_command ./{{build_dir}}/mt_standard "${cfg}" ;; \
    response-yaw) if [[ -z "${a}" ]]; then a="3"; fi; run_command ./{{build_dir}}/gimbal_response_test -a "${a}" -m "${sig}" -x yaw "${cfg}" ;; \
    response-pitch) if [[ -z "${a}" ]]; then a="2"; fi; run_command ./{{build_dir}}/gimbal_response_test -a "${a}" -m "${sig}" -x pitch "${cfg}" ;; \
    *) echo "[run] Unknown run target '{{name}}'. Supported: standard, mt, response-yaw, response-pitch. See: just run-help" >&2; exit 1 ;; \
  esac

# Print detailed help for unified runtime launcher.
run-help:
  @echo "just run <name> [config_path] [flags]"
  @echo ""
  @echo "Env: HIK_SDK_RUNTIME=auto|default|system|custom, MVCAM_LD_PRELOAD=/path/lib.so"
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
  @cfg="{{config}}"; in_folder="{{input_folder}}"; out_folder="{{output_folder}}"; cli_mode="false"; in_path=""; out_path=""; start=""; end=""; hik_sdk_runtime="{{hik_sdk_runtime}}"; mvs_lib="${MVCAM_LD_PRELOAD:-}"; mvs_runenv="${MVCAM_COMMON_RUNENV:-/opt/MVS/lib}"; machine="$(uname -m)"; host_name="$(hostname 2>/dev/null || true)"; product_name="$(cat /sys/devices/virtual/dmi/id/product_name 2>/dev/null || true)"; \
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
  if [[ "${hik_sdk_runtime}" == "auto" ]]; then \
    if [[ "${machine}" == "x86_64" && ( "${host_name,,}" == *nuc* || "${product_name,,}" == *nuc* ) ]]; then \
      hik_sdk_runtime="system"; \
    else \
      hik_sdk_runtime="default"; \
    fi; \
  fi; \
  if [[ -z "${mvs_lib}" && "${hik_sdk_runtime}" == "system" ]]; then \
    for candidate in /opt/MVS/lib/64/libMvCameraControl.so*; do \
      if [[ -f "${candidate}" ]]; then \
        mvs_lib="${candidate}"; \
        break; \
      fi; \
    done; \
  fi; \
  run_command() { \
    if [[ -n "${mvs_lib}" && ( "${hik_sdk_runtime}" == "system" || "${hik_sdk_runtime}" == "custom" ) ]]; then \
      env LD_PRELOAD="${mvs_lib}${LD_PRELOAD:+:${LD_PRELOAD}}" MVCAM_COMMON_RUNENV="${mvs_runenv}" "$@"; \
    else \
      "$@"; \
    fi; \
  }; \
  case "{{name}}" in \
    capture) \
      if [[ "${cli_mode}" == "true" ]]; then \
        run_command ./{{build_dir}}/capture "${cfg}" --output-folder="${out_folder}" --cli-mode; \
      else \
        run_command ./{{build_dir}}/capture "${cfg}" --output-folder="${out_folder}"; \
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
  @echo "Env: HIK_SDK_RUNTIME=auto|default|system|custom, MVCAM_LD_PRELOAD=/path/lib.so"
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
  @cfg="{{config}}"; display="false"; send="false"; fire="false"; use_web="false"; web_port="8888"; gimbal_opts=""; local_display=""; local_xauth=""; test_target=""; test_use_tensorrt="{{default_use_tensorrt}}"; hik_sdk_runtime="{{hik_sdk_runtime}}"; mvs_lib="${MVCAM_LD_PRELOAD:-}"; mvs_runenv="${MVCAM_COMMON_RUNENV:-/opt/MVS/lib}"; machine="$(uname -m)"; host_name="$(hostname 2>/dev/null || true)"; product_name="$(cat /sys/devices/virtual/dmi/id/product_name 2>/dev/null || true)"; \
  for token in "{{arg}}" "{{arg2}}" "{{arg3}}" "{{arg4}}"; do \
    if [[ -z "${token}" ]]; then \
      continue; \
    elif [[ "${token}" == "-d" || "${token}" == "-display" || "${token}" == "--display" ]]; then \
      display="true"; \
    elif [[ "${token}" =~ ^-d[0-9]+$ ]]; then \
      display="true"; \
      local_display=":${token:2}"; \
    elif [[ "${token}" =~ ^-d:[0-9]+$ ]]; then \
      display="true"; \
      local_display="${token:2}"; \
    elif [[ "${token}" == "-w" || "${token}" == "--web" ]]; then \
      use_web="true"; \
    elif [[ "${token}" == --web-port=* ]]; then \
      web_port="${token#--web-port=}"; \
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
    elif [[ "${token}" == -* ]]; then \
      echo "[test] Unknown flag '${token}'. See: just test-help" >&2; \
      exit 1; \
    else \
      cfg="${token}"; \
    fi; \
  done; \
  if [[ "${hik_sdk_runtime}" == "auto" ]]; then \
    if [[ "${machine}" == "x86_64" && ( "${host_name,,}" == *nuc* || "${product_name,,}" == *nuc* ) ]]; then \
      hik_sdk_runtime="system"; \
    else \
      hik_sdk_runtime="default"; \
    fi; \
  fi; \
  if [[ -z "${mvs_lib}" && "${hik_sdk_runtime}" == "system" ]]; then \
    for candidate in /opt/MVS/lib/64/libMvCameraControl.so*; do \
      if [[ -f "${candidate}" ]]; then \
        mvs_lib="${candidate}"; \
        break; \
      fi; \
    done; \
  fi; \
  if [[ -n "${local_display}" && -z "${local_xauth}" ]]; then \
    local_xauth="${HOME}/.Xauthority"; \
  fi; \
  case "{{name}}" in \
    imu) test_target="imu_communication_test" ;; \
    camera) test_target="camera_test" ;; \
    detect) test_target="auto_aim_camera_test"; test_use_tensorrt="ON" ;; \
    gimbal) test_target="gimbal_test" ;; \
    *) echo "[test] Unknown test '{{name}}'. Supported: imu, camera, detect, gimbal. See: just test-help" >&2; exit 1 ;; \
  esac; \
  just ensure-configured "{{build_dir}}" "{{default_profile}}" "${test_use_tensorrt}" "{{default_openvino_dir}}"; \
  command cmake --build "{{build_dir}}" --target "${test_target}" -j{{jobs}}; \
  run_with_display() { \
    if [[ -n "${mvs_lib}" && ( "${hik_sdk_runtime}" == "system" || "${hik_sdk_runtime}" == "custom" ) ]]; then \
      if [[ -n "${local_display}" ]]; then \
        if [[ -n "${local_xauth}" ]]; then \
          env LD_PRELOAD="${mvs_lib}${LD_PRELOAD:+:${LD_PRELOAD}}" MVCAM_COMMON_RUNENV="${mvs_runenv}" DISPLAY="${local_display}" XAUTHORITY="${local_xauth}" "$@"; \
        else \
          env LD_PRELOAD="${mvs_lib}${LD_PRELOAD:+:${LD_PRELOAD}}" MVCAM_COMMON_RUNENV="${mvs_runenv}" DISPLAY="${local_display}" "$@"; \
        fi; \
      else \
        env LD_PRELOAD="${mvs_lib}${LD_PRELOAD:+:${LD_PRELOAD}}" MVCAM_COMMON_RUNENV="${mvs_runenv}" "$@"; \
      fi; \
    elif [[ -n "${local_display}" ]]; then \
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
    detect) \
      detect_opts=""; \
      if [[ "${send}" == "true" ]]; then detect_opts="${detect_opts} --send"; fi; \
      if [[ "${use_web}" == "true" ]]; then \
        detect_opts="${detect_opts} --web --web-port=${web_port}"; \
      elif [[ "${display}" == "true" ]]; then \
        detect_opts="${detect_opts} --display"; \
      fi; \
      run_with_display ./{{build_dir}}/auto_aim_camera_test "${cfg}" ${detect_opts} ;; \
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
  @echo "  -d, --display    camera/detect: show GUI window (X11)"
  @echo "  -w, --web        detect: show web interface (http://localhost:8888)"
  @echo "  --web-port=N     detect: web server port (default: 8888)"
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
  @echo "  env HIK_SDK_RUNTIME=auto|default|system|custom   control Hik SDK selection"
  @echo "  env MVCAM_LD_PRELOAD=/path/lib.so   use a specific Hik SDK when HIK_SDK_RUNTIME=custom"
  @echo "  note: detect auto-configures USE_TENSORRT=ON (requires TensorRT/CUDA libs)"
  @echo "  (interactive)    gimbal: aim <yaw_deg> [pitch_deg], sim on/off, send on/off"
  @echo ""
  @echo "Examples:"
  @echo "  just test imu"
  @echo "  just test camera -d"
  @echo "  just test camera -d 0"
  @echo "  just test camera -d --local-display"
  @echo "  just test detect --web"
  @echo "  just test detect --web --web-port=9000"
  @echo "  just test detect --web --send"
  @echo "  just test detect --local-display=:0 --xauthority=/home/nyu/.Xauthority"
  @echo "  just test detect configs/odin.yaml --send"
  @echo "  just test gimbal --fire"
  @echo "  just test gimbal --distance=4 --radius=0.8 --omega=1.5 --hz=120"
  @echo "  just test gimbal --verbose"

# Build a TensorRT engine from an ONNX model.
trt-engine onnx="" engine="" precision="fp16":
  @if [[ -z "{{onnx}}" || -z "{{engine}}" ]]; then \
    echo "Usage: just trt-engine --onnx=assets/model.onnx --engine=assets/model.plan [--precision=fp16|fp32]" >&2; \
    exit 1; \
  fi; \
  command bash ./tools/build_trt_engine.sh "{{onnx}}" "{{engine}}" "{{precision}}"
