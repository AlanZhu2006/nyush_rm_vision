#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 <onnx_path> <engine_path> [precision]" >&2
  echo "  precision: fp16 (default) | fp32" >&2
  exit 1
fi

onnx_path="$1"
engine_path="$2"
precision="${3:-fp16}"

if [[ ! -f "${onnx_path}" ]]; then
  echo "[trt-engine] ONNX model not found: ${onnx_path}" >&2
  exit 1
fi

if [[ "${precision}" != "fp16" && "${precision}" != "fp32" ]]; then
  echo "[trt-engine] Invalid precision '${precision}', expected fp16 or fp32" >&2
  exit 1
fi

trtexec_bin=""
if command -v trtexec >/dev/null 2>&1; then
  trtexec_bin="$(command -v trtexec)"
elif [[ -x "/usr/src/tensorrt/bin/trtexec" ]]; then
  trtexec_bin="/usr/src/tensorrt/bin/trtexec"
else
  echo "[trt-engine] trtexec not found in PATH or /usr/src/tensorrt/bin/trtexec" >&2
  exit 1
fi

mkdir -p "$(dirname "${engine_path}")"

cmd=("${trtexec_bin}" "--onnx=${onnx_path}" "--saveEngine=${engine_path}")
if [[ "${precision}" == "fp16" ]]; then
  cmd+=("--fp16")
fi

echo "[trt-engine] building ${engine_path} from ${onnx_path} (${precision})"
"${cmd[@]}"
