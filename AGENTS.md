# AGENTS.md

Operational guidance for coding agents working in `sp_vision_25`.

## Scope and architecture
- Language: mostly C++17, plus a small Python audit script.
- Build system: CMake + `just` wrappers.
- Main dirs:
  - `io/`: camera, serial/CAN/gimbal I/O.
  - `tasks/auto_aim`, `tasks/auto_buff`, `tasks/omniperception`: core algorithms.
  - `tools/`: logger/math/filter/plot/record helpers.
  - `src/`: runtime entrypoints.
  - `tests/`: standalone test executables.

## Agent workflow requirements
- Make minimal framework changes by default. Prefer local fixes over broad refactors.
- Before modifying architecture-level code, read the full related call chain first (typically across `src/`, `io/`, `tasks/`, `tools/`, `tests/`).
- Avoid changing public interfaces/module boundaries unless required by the task.
- If you must make framework-level changes, explain why smaller-scope options are insufficient.

## Cursor/Copilot rule files
- Checked `.cursor/rules/`, `.cursorrules`, `.github/copilot-instructions.md`.
- Result: none found in this repository.

## Build commands
Build constraints (project facts):
- Generator must be `Unix Makefiles` (Ninja is rejected in top-level CMake).
- C++ standard is C++17.
- Top-level currently sets `CMAKE_BUILD_TYPE` to `Release`.
- Optional `USE_TENSORRT=ON` requires TensorRT/CUDA libs.
- ROS2 targets are built only if ROS2 dependencies are found.

Preferred workflow (`just`):
```bash
just --list
just build
just run mt
just run standard
```

Default build policy for agents:
- Unless the user explicitly requests otherwise, compile with `just build`.
- For faster iteration on one target after initial full build, prefer `just build <target>`.
- Do not suggest raw `cmake`/`make` command flows in normal agent instructions.

Single-target build (fast iteration):
```bash
just build auto_aim_test
just build camera_test
just build imu_communication_test
```

## Test commands
Test model in this repo:
- There is no `ctest` / `add_test` suite.
- Tests are executable targets under `tests/*.cpp`.
- Running one test should go through a `just` recipe whenever possible.
- Many tests require hardware (camera/gimbal/CAN/serial).

Unified `just` wrapper:
```bash
just test imu
just test camera
just test detect
just test camera -d
just test detect --send
just test imu configs/odin.yaml
```

Default test policy for agents:
- Prefer `just` preset commands for testing (`just test ...`, `just run ...`).
- If no `just` recipe exists yet for a C/C++ executable (test or utility), add one in `justfile` first, then run it via `just`.
- Do not suggest direct `./build/...` execution when an equivalent `just` recipe already exists.
- Most test binaries use OpenCV `CommandLineParser`; include equivalent args in the `just` recipe.

Offline protocol audit (no hardware):
```bash
python3 tests/protocol_link_audit.py
python3 tests/protocol_link_audit.py --rounds 2000 --seed 20260225
```

## Lint and format
Formatting:
- Canonical style file: `.clang-format` (Google base, 2-space indent, 100 columns).
```bash
clang-format -i src/standard.cpp
git ls-files '*.cpp' '*.hpp' | xargs clang-format -i
```

Linting:
- No dedicated lint target (`just lint`, clang-tidy, cpplint) is present.
- Effective lint gate is successful compilation of changed targets.

## Code style guidelines
File/layout:
- Header guards are the norm (`DIR__FILE_HPP`).
- Filenames are usually `snake_case`.
- Keep changes scoped to existing module boundaries.

Includes and dependencies:
- Preferred order in `.cpp`:
  1) matching project header
  2) standard library headers
  3) third-party headers (OpenCV/Eigen/fmt/spdlog/...)
  4) other project headers
- Existing code is not fully uniform; improve locally, avoid unrelated churn.

Naming:
- Types/classes/structs: `PascalCase`.
- Variables/functions/methods: mostly `snake_case`.
- Enum style is mixed (`enum class` and legacy enums); follow local style.
- Constants often use `UPPER_SNAKE_CASE`.

Formatting details:
- Use `.clang-format` defaults.
- Braces are generally on their own line for namespace/class/function/struct.
- Indentation is 2 spaces; target line width is 100.
- Pointer alignment is `Type * name`.
- `// clang-format off/on` is used only around readability-critical tables/math blocks.

Types and APIs:
- Prefer explicit types for interfaces (`std::size_t`, `Eigen::Vector3d`, chrono types).
- Prefer `const T &` for non-trivial inputs.
- Use `auto` when type is obvious from initializer.
- Keep units clear in names when relevant (`yaw_rad`, `bullet_speed`).

Error handling and logging:
- Use `tools::logger()` (`spdlog`) for runtime logging.
- Config loading typically uses `tools::load` + `tools::read<T>`.
- Unrecoverable init/config errors often fail fast (`exit(1)` or throw).
- Runtime I/O errors are commonly handled by catch + `warn` + continue/retry.
- Keep log levels meaningful (`debug/info/warn/error`).

Concurrency and realtime loops:
- Common primitives: threads, mutexes, `tools::ThreadSafeQueue`.
- Keep callbacks/read threads thread-safe.
- Avoid introducing blocking work in high-frequency loops.

## Validation expectations for changes
- Minimum: compile the targets you touched (prefer `just build <target>` after baseline `just build`).
- If touching protocol/packet/sign conventions, run `tests/protocol_link_audit.py`.
- If touching hardware interfaces, prefer running relevant `just` tests when hardware is available (e.g. `just test imu`, `just test camera`), and add missing recipes for others.

## `just` argument pitfall
- Follow README guidance: use default/positional `just` invocation.
- Avoid accidental path-like tokens such as `build_dir=build` as positional arguments; this can create unintended directories.
- Safe examples:
```bash
just build
just test imu
just test camera -d
```
