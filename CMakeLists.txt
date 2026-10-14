cmake_minimum_required(VERSION 3.21)
project(SiliconGhost VERSION 0.1.0 DESCRIPTION "SGI IP22 (Indy) emulator" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(SG_BUILD_TESTS "Build unit tests" ON)
option(SG_WERROR "Treat warnings as errors" OFF)

add_library(sg_warnings INTERFACE)
if(MSVC)
  target_compile_options(sg_warnings INTERFACE /W4 /permissive- /Zc:__cplusplus)
  if(SG_WERROR)
    target_compile_options(sg_warnings INTERFACE /WX)
  endif()
else()
  target_compile_options(sg_warnings INTERFACE -Wall -Wextra -Wshadow -Wnon-virtual-dtor)
  if(SG_WERROR)
    target_compile_options(sg_warnings INTERFACE -Werror)
  endif()
endif()

add_library(sg_emu STATIC
  src/core/log.cpp
  src/core/pacer.cpp
  src/core/scheduler.cpp
  src/jit/exec_buffer.cpp
  src/mem/bus.cpp
  src/cpu/r4000/r4000.cpp
  src/cpu/r4000/disasm.cpp
  src/cpu/r4000/fpu.cpp
  src/cpu/r4000/jit_x64.cpp
  src/cpu/r4000/jit_a64.cpp
  src/cpu/r4000/jit_plan.cpp
  src/devices/ds1386.cpp
  src/devices/eeprom93cxx.cpp
  src/devices/kbc8042.cpp
  src/devices/pit8254.cpp
  src/devices/scsi_disk.cpp
  src/devices/seeq8003.cpp
  src/devices/wd33c93.cpp
  src/devices/z85230.cpp
  src/machine/ip22/hal2.cpp
  src/machine/ip22/hpc3.cpp
  src/machine/ip22/indy.cpp
  src/machine/ip22/ioc2.cpp
  src/machine/ip22/mc.cpp
  src/machine/ip22/newport.cpp
  src/net/net_backend.cpp
  src/net/slirp_backend.cpp
  src/net/tap_backend.cpp
)
target_include_directories(sg_emu PUBLIC src)
target_link_libraries(sg_emu PRIVATE sg_warnings)
find_package(Threads REQUIRED)
target_link_libraries(sg_emu PUBLIC Threads::Threads)

# User-mode networking (optional)
option(SG_WITH_SLIRP "Build user-mode networking with libslirp" ON)
if(SG_WITH_SLIRP)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(SLIRP IMPORTED_TARGET slirp)
  endif()
  if(SLIRP_FOUND)
    target_compile_definitions(sg_emu PRIVATE SG_HAVE_SLIRP=1)
    target_link_libraries(sg_emu PUBLIC PkgConfig::SLIRP)
  else()
    message(WARNING "libslirp not found: building without user-mode networking")
  endif()
endif()

# The FPU relies on the host rounding mode and exception flags.
if(MSVC)
  set_source_files_properties(src/cpu/r4000/fpu.cpp PROPERTIES COMPILE_OPTIONS "/fp:strict")
else()
  set_source_files_properties(src/cpu/r4000/fpu.cpp PROPERTIES COMPILE_OPTIONS "-frounding-math;-ffp-contract=off")
endif()

add_executable(siliconghost src/app/main.cpp)
target_link_libraries(siliconghost PRIVATE sg_emu sg_warnings)

option(SG_WITH_SDL3 "Build the SDL3 display front end" ON)
if(SG_WITH_SDL3)
  find_package(SDL3 CONFIG QUIET)
  if(SDL3_FOUND)
    target_sources(siliconghost PRIVATE src/app/display_sdl.cpp)
    target_compile_definitions(siliconghost PRIVATE SG_HAVE_SDL3=1)
    target_link_libraries(siliconghost PRIVATE SDL3::SDL3)
  else()
    message(WARNING "SDL3 not found: building without a display window or sound")
  endif()
endif()

if(SG_BUILD_TESTS)
  enable_testing()
  add_executable(sg_tests
    tests/test_main.cpp
    tests/cpu_tests.cpp
    tests/misc_tests.cpp
    tests/device_tests.cpp
    tests/fpu_cache_tests.cpp
    tests/ip22_tests.cpp
    tests/newport_tests.cpp
  )
  target_link_libraries(sg_tests PRIVATE sg_emu sg_warnings)
  add_test(NAME sg_tests COMMAND sg_tests)
endif()
