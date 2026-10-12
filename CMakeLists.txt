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
  src/mem/bus.cpp
  src/cpu/r4000/r4000.cpp
  src/cpu/r4000/disasm.cpp
  src/devices/stub_device.cpp
  src/machine/ip22/indy.cpp
)
target_include_directories(sg_emu PUBLIC src)
target_link_libraries(sg_emu PRIVATE sg_warnings)

add_executable(siliconghost src/app/main.cpp)
target_link_libraries(siliconghost PRIVATE sg_emu sg_warnings)

if(SG_BUILD_TESTS)
  enable_testing()
  add_executable(sg_tests
    tests/test_main.cpp
    tests/cpu_tests.cpp
    tests/misc_tests.cpp
  )
  target_link_libraries(sg_tests PRIVATE sg_emu sg_warnings)
  add_test(NAME sg_tests COMMAND sg_tests)
endif()
