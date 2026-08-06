# Source this file, then run `v_flash_c` to build and flash the ROSflight C
# revision whose parameter-storage hash is accepted by Veloxity.

typeset -g VELOXITY_COMPATIBLE_C_COMMIT="a46527bd8e49d00a072c7efd7af9dd543910d831"
typeset -g VELOXITY_C_FIRMWARE_DIR="${${(%):-%N}:A:h:h}/../rosflight/rosflight/workspace/src/rosflight_ros_pkgs/rosflight_firmware"

function v_flash_c() {
  local command_name
  for command_name in git cmake arm-none-eabi-gcc probe-rs; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
      print -u2 -- "Missing required command: $command_name"
      return 1
    fi
  done

  if [[ ! -d "$VELOXITY_C_FIRMWARE_DIR/.git" && ! -f "$VELOXITY_C_FIRMWARE_DIR/.git" ]]; then
    print -u2 -- "ROSflight C checkout not found: $VELOXITY_C_FIRMWARE_DIR"
    return 1
  fi

  local actual_commit
  actual_commit="$(git -C "$VELOXITY_C_FIRMWARE_DIR" rev-parse HEAD)" || return 1
  if [[ "$actual_commit" != "$VELOXITY_COMPATIBLE_C_COMMIT" ]]; then
    print -u2 -- "Refusing to flash commit $actual_commit"
    print -u2 -- "Veloxity currently accepts $VELOXITY_COMPATIBLE_C_COMMIT"
    return 1
  fi

  if [[ -n "$(git -C "$VELOXITY_C_FIRMWARE_DIR" status --porcelain --untracked-files=no)" ]]; then
    print -u2 -- "Refusing to flash: the ROSflight C checkout has modified tracked files."
    git -C "$VELOXITY_C_FIRMWARE_DIR" status --short
    return 1
  fi

  print -- "Compatible ROSflight C commit: $actual_commit"
  probe-rs list || return 1

  local confirmation
  read -r "confirmation?Remove all propellers and confirm the Pixracer Pro is connected. Flash now? [y/N] "
  if [[ "$confirmation" != [yY] && "$confirmation" != [yY][eE][sS] ]]; then
    print -- "Flash cancelled."
    return 1
  fi

  (
    cd "$VELOXITY_C_FIRMWARE_DIR" || return 1

    local build_dir
    if command -v ninja >/dev/null 2>&1; then
      build_dir="build/pixracer-pro-release"
      cmake --preset pixracer-pro-release || return 1
    else
      if ! command -v make >/dev/null 2>&1; then
        print -u2 -- "Missing required build tool: install ninja or make."
        return 1
      fi
      build_dir="build/pixracer-pro-release-make"
      print -- "Ninja is unavailable; configuring the equivalent release build with Make."
      cmake -S . -B "$build_dir" \
        -G "Unix Makefiles" \
        -DCMAKE_TOOLCHAIN_FILE="$VELOXITY_C_FIRMWARE_DIR/cmake/stm32_gcc.cmake" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_FLAGS_RELEASE=-O3 \
        -DCMAKE_CXX_FLAGS_RELEASE=-O3 \
        -DBOARD_TO_BUILD=pixracer_pro || return 1
    fi

    cmake --build "$build_dir" || return 1

    local firmware_elf="$build_dir/output/pixracer_pro.elf"
    if [[ ! -f "$firmware_elf" ]]; then
      print -u2 -- "Build completed without the expected ELF: $firmware_elf"
      return 1
    fi

    probe-rs download \
      --chip STM32H743IIKx \
      --protocol swd \
      --speed 4000 \
      "$firmware_elf" || return 1
    probe-rs reset --chip STM32H743IIKx || return 1
  ) || return 1

  print -- "ROSflight C $actual_commit flashed and reset successfully."
}
