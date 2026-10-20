# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2026 Andy Taylor
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
if(MINGW)
  # The C99-conforming printf family (%zu, %lld), as the code expects.
  add_compile_definitions(__USE_MINGW_ANSI_STDIO=1)
endif()
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
  src/core/host_memory.cpp
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
  src/devices/test_device.cpp
  src/devices/wd33c93.cpp
  src/devices/z85230.cpp
  src/machine/ip22/hal2.cpp
  src/machine/ip22/hpc3.cpp
  src/machine/ip22/indy.cpp
  src/machine/ip22/ioc2.cpp
  src/machine/ip22/mc.cpp
  src/machine/ip22/impact.cpp
  src/machine/ip22/impact_ge.cpp
  src/machine/ip22/impact_gl.cpp
  src/machine/ip22/impact_te.cpp
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

# Front-end pieces without a window: settings, power control, terminal screen
add_library(sg_app STATIC
  src/app/detach.cpp
  src/app/proms.cpp
  src/app/session.cpp
  src/app/settings.cpp
  src/app/terminal.cpp
)
target_link_libraries(sg_app PUBLIC sg_emu PRIVATE sg_warnings)
if(WIN32)
  target_link_libraries(sg_app PRIVATE shell32 ole32)  # the Documents folder
endif()

add_executable(siliconghost src/app/main.cpp)
target_link_libraries(siliconghost PRIVATE sg_app sg_warnings)

option(SG_WITH_SDL3 "Build the SDL3 display front end" ON)
if(SG_WITH_SDL3)
  find_package(SDL3 CONFIG QUIET)
  if(SDL3_FOUND)
    # Dear ImGui, for the window's menus, dialogs and status bar
    add_library(sg_imgui STATIC
      third_party/imgui/imgui.cpp
      third_party/imgui/imgui_draw.cpp
      third_party/imgui/imgui_tables.cpp
      third_party/imgui/imgui_widgets.cpp
      third_party/imgui/misc/cpp/imgui_stdlib.cpp
      third_party/imgui/backends/imgui_impl_sdl3.cpp
      third_party/imgui/backends/imgui_impl_sdlrenderer3.cpp
    )
    target_include_directories(sg_imgui PUBLIC third_party/imgui)
    target_compile_definitions(sg_imgui PUBLIC IMGUI_DISABLE_DEMO_WINDOWS IMGUI_DISABLE_OBSOLETE_FUNCTIONS)
    target_link_libraries(sg_imgui PUBLIC SDL3::SDL3)

    target_sources(siliconghost PRIVATE
      src/app/display_sdl.cpp
      src/app/gui.cpp
      src/app/terminal_window.cpp
    )
    target_compile_definitions(siliconghost PRIVATE SG_HAVE_SDL3=1)
    target_link_libraries(siliconghost PRIVATE sg_imgui SDL3::SDL3)

    if(APPLE)
      # A macOS application: "Silicon Ghost.app", with the system menu bar and
      # its icon (made from the 1024x1024 artwork with sips and iconutil).
      # build/siliconghost links to the program inside, for the terminal.
      # `cmake --install build --prefix DIR` makes a self-contained copy in
      # DIR, with the libraries it uses (SDL3, libslirp, ...) inside it.
      enable_language(OBJCXX)
      target_sources(siliconghost PRIVATE src/app/menu_mac.mm)
      set_source_files_properties(src/app/menu_mac.mm PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
      target_link_libraries(siliconghost PRIVATE "-framework Cocoa")

      set(SG_ICON_PNG ${CMAKE_CURRENT_SOURCE_DIR}/graphics/SiliconGhost_icon_1024.png)
      set(SG_ICONSET ${CMAKE_CURRENT_BINARY_DIR}/SiliconGhost.iconset)
      set(SG_ICNS ${CMAKE_CURRENT_BINARY_DIR}/SiliconGhost.icns)
      set(SG_ICON_COMMANDS)
      foreach(size 16 32 128 256 512)
        math(EXPR double "${size} * 2")
        list(APPEND SG_ICON_COMMANDS
          COMMAND sips -z ${size} ${size} ${SG_ICON_PNG} --out ${SG_ICONSET}/icon_${size}x${size}.png
          COMMAND sips -z ${double} ${double} ${SG_ICON_PNG} --out ${SG_ICONSET}/icon_${size}x${size}@2x.png)
      endforeach()
      add_custom_command(OUTPUT ${SG_ICNS}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${SG_ICONSET}
        ${SG_ICON_COMMANDS}
        COMMAND iconutil -c icns ${SG_ICONSET} -o ${SG_ICNS}
        DEPENDS ${SG_ICON_PNG}
        COMMENT "Making the application icon"
        VERBATIM)
      target_sources(siliconghost PRIVATE ${SG_ICNS})
      set_source_files_properties(${SG_ICNS} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)

      configure_file(cmake/Info.plist.in ${CMAKE_CURRENT_BINARY_DIR}/Info.plist @ONLY)
      set_target_properties(siliconghost PROPERTIES
        MACOSX_BUNDLE TRUE
        OUTPUT_NAME "Silicon Ghost"
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_BINARY_DIR}/Info.plist)
      add_custom_command(TARGET siliconghost POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rm -f ${CMAKE_CURRENT_BINARY_DIR}/siliconghost
        COMMAND ${CMAKE_COMMAND} -E create_symlink "Silicon Ghost.app/Contents/MacOS/Silicon Ghost"
                ${CMAKE_CURRENT_BINARY_DIR}/siliconghost
        VERBATIM)

      install(TARGETS siliconghost BUNDLE DESTINATION .)
      # The Info.plist is installed explicitly as well, and the libraries are
      # gathered from the program's own path, so neither depends on the other.
      install(FILES ${CMAKE_CURRENT_BINARY_DIR}/Info.plist DESTINATION "Silicon Ghost.app/Contents")
      install(CODE [[
        include(BundleUtilities)
        set(app "${CMAKE_INSTALL_PREFIX}/Silicon Ghost.app")
        fixup_bundle("${app}/Contents/MacOS/Silicon Ghost" "" "")
        # Signed for this machine (ad hoc): copying the libraries in changed them.
        execute_process(COMMAND codesign --force --deep --sign - "${app}" COMMAND_ERROR_IS_FATAL ANY)
        execute_process(COMMAND codesign --verify --deep --strict "${app}" COMMAND_ERROR_IS_FATAL ANY)
      ]])
    endif()
  else()
    message(WARNING "SDL3 not found: building without a display window or sound")
  endif()
endif()

if(SG_BUILD_TESTS)
  enable_testing()
  add_executable(sg_tests
    tests/test_main.cpp
    tests/cpu_tests.cpp
    tests/cpu_model_tests.cpp
    tests/r10000_tests.cpp
    tests/misc_tests.cpp
    tests/device_tests.cpp
    tests/fpu_cache_tests.cpp
    tests/ip22_tests.cpp
    tests/impact_tests.cpp
    tests/newport_tests.cpp
    tests/app_tests.cpp
  )
  target_link_libraries(sg_tests PRIVATE sg_app sg_warnings)
  # Tests keep whole machines (over 1 MB each) on the stack; Windows gives
  # the main thread 1 MB by default.
  if(WIN32)
    if(MSVC)
      target_link_options(sg_tests PRIVATE /STACK:16777216)
    else()
      target_link_options(sg_tests PRIVATE -Wl,--stack,16777216)
    endif()
  endif()
  add_test(NAME sg_tests COMMAND sg_tests)
endif()
