set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---------------------------------------------------------------------------
# Locate the Arm GNU toolchain (arm-none-eabi-gcc).
#
# Preference order:
#   1. ARM_GCC_ROOT environment variable (explicit override)
#   2. GCC bundled with STM32Cube / STM32CubeIDE (14.x)
#   3. arm-none-eabi-gcc found on PATH
#
# Note: some vendor builds of GCC 10.3 cannot create object/dependency files
# whose absolute path exceeds the Windows MAX_PATH limit (260 characters).
# This source tree is deep enough to hit that limit, which produces the
# misleading "opening dependency file ... No such file or directory" error.
# Prefer a 14.x toolchain when available.
# ---------------------------------------------------------------------------

set(_gcc_root "")

file(TO_CMAKE_PATH "$ENV{LOCALAPPDATA}" _local_appdata)

if(DEFINED ENV{ARM_GCC_ROOT} AND EXISTS "$ENV{ARM_GCC_ROOT}/bin/arm-none-eabi-gcc.exe")
  set(_gcc_root "$ENV{ARM_GCC_ROOT}")
endif()

if(NOT _gcc_root)
  file(GLOB _gcc_candidates
    "${_local_appdata}/stm32cube/bundles/gnu-tools-for-stm32/*"
    "${_local_appdata}/Programs/ArmGNU/xpack-arm-none-eabi-gcc-*"
    "C:/ST/STM32CubeIDE/*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools"
  )
  list(SORT _gcc_candidates)
  list(REVERSE _gcc_candidates)
  foreach(_candidate IN LISTS _gcc_candidates)
    if(EXISTS "${_candidate}/bin/arm-none-eabi-gcc.exe")
      set(_gcc_root "${_candidate}")
      break()
    endif()
  endforeach()
endif()

if(NOT _gcc_root)
  find_program(_gcc_program arm-none-eabi-gcc)
  if(_gcc_program)
    get_filename_component(_gcc_bin "${_gcc_program}" DIRECTORY)
    get_filename_component(_gcc_root "${_gcc_bin}" DIRECTORY)
  endif()
endif()

if(NOT _gcc_root)
  message(FATAL_ERROR
    "arm-none-eabi-gcc was not found. Install the Arm GNU toolchain, set "
    "ARM_GCC_ROOT, or add arm-none-eabi-gcc to PATH.")
endif()

message(STATUS "Using ARM GCC toolchain: ${_gcc_root}")

# Locate Ninja when the Ninja generator is used (this project's presets).
if(CMAKE_GENERATOR STREQUAL "Ninja" AND NOT CMAKE_MAKE_PROGRAM)
  find_program(_ninja_program ninja ninja-build)
  if(NOT _ninja_program AND WIN32)
    file(GLOB _ninja_candidates
      "${_local_appdata}/at32-tools/ninja/*/ninja.exe"
      "$ENV{ProgramFiles}/CMake/bin/ninja.exe"
      "D:/application/cmake/bin/ninja.exe"
    )
    list(SORT _ninja_candidates)
    list(REVERSE _ninja_candidates)
    foreach(_candidate IN LISTS _ninja_candidates)
      if(EXISTS "${_candidate}")
        set(_ninja_program "${_candidate}")
        break()
      endif()
    endforeach()
  endif()
  if(_ninja_program)
    set(CMAKE_MAKE_PROGRAM "${_ninja_program}" CACHE FILEPATH "Ninja build program" FORCE)
  endif()
endif()

set(CMAKE_C_COMPILER   "${_gcc_root}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${_gcc_root}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_OBJCOPY      "${_gcc_root}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE         "${_gcc_root}/bin/arm-none-eabi-size.exe")
