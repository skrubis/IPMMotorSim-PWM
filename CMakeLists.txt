cmake_minimum_required(VERSION 3.16)

project(IPMMotorSim LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/IPMMotorSim/stm32-sine")
  message(FATAL_ERROR "Missing submodule: IPMMotorSim/stm32-sine. Run: git submodule update --init --recursive")
endif()

find_package(Qt6 QUIET COMPONENTS Widgets Charts)
if(Qt6_FOUND)
  set(QT_LIBS Qt6::Widgets Qt6::Charts)
  set(QT_CORE_LIBS Qt6::Core)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)
  set(QT_LIBS Qt5::Widgets Qt5::Charts)
  set(QT_CORE_LIBS Qt5::Core)
endif()

set(_ipmmotorsim_exe_flags "")
if(WIN32)
  list(APPEND _ipmmotorsim_exe_flags WIN32)
elseif(APPLE)
  list(APPEND _ipmmotorsim_exe_flags MACOSX_BUNDLE)
endif()

add_executable(IPMMotorSim ${_ipmmotorsim_exe_flags}
  IPMMotorSim/main.cpp
  IPMMotorSim/mainwindow.cpp
  IPMMotorSim/run_orchestrator.cpp
  IPMMotorSim/run_orchestrator.h
  IPMMotorSim/scenario_window.cpp
  IPMMotorSim/scenario_window.h
  IPMMotorSim/scenario_window.ui
  IPMMotorSim/chart.cpp
  IPMMotorSim/chartview.cpp
  IPMMotorSim/datagraph.cpp
  IPMMotorSim/motormodel.cpp
  IPMMotorSim/idiqgraph.cpp
  IPMMotorSim/app_logging.cpp
  IPMMotorSim/terminal_stubs.cpp
  IPMMotorSim/cpp_teststubs.cpp
  IPMMotorSim/teststubs.c
  src/sim/controller.cpp
  src/sim/modulator.cpp
  src/sim/modulator_diagnostics.cpp
  src/sim/inverter_switching_model.cpp
  src/sim/metrics/metric_pack.cpp
  src/sim/util/process_utils.cpp
  src/sim/sweep/sweep_config.cpp
  src/sim/sweep/sweep_runner.cpp
  src/sim/sim_runner.cpp
  src/sim/pwm_timeline.cpp
  src/sim/power_module.cpp
  src/sim/strategy/strategy_supervisor.cpp
  src/sim/strategy/strategy_spec.cpp
  src/sim/lut/lut_builder.cpp
  IPMMotorSim/mainwindow.ui
  IPMMotorSim/stm32-sine/libopeninv/src/params.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/picontroller.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/sine_core.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/my_string.c
  IPMMotorSim/stm32-sine/libopeninv/src/errormessage.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/foc.cpp
  IPMMotorSim/stm32-sine/src/pwmgeneration-foc.cpp
  IPMMotorSim/stm32-sine/src/pwmgeneration.cpp
)

target_include_directories(IPMMotorSim PRIVATE
  IPMMotorSim
  src
  src/sim
  IPMMotorSim/stm32-sine/include
  IPMMotorSim/stm32-sine/libopencm3/include
  IPMMotorSim/stm32-sine/libopeninv/include
)

target_compile_definitions(IPMMotorSim PRIVATE
  QT_DEPRECATED_WARNINGS
  CTRL_FOC=1
  CTRL_SINE=0
  CONTROL=1
  STM32F1
  GITHUB_RUN_NUMBER=0
)

if(MSVC)
  target_compile_options(IPMMotorSim PRIVATE /Zc:__cplusplus)
endif()

target_link_libraries(IPMMotorSim PRIVATE ${QT_LIBS})

enable_testing()

add_executable(unit_tests
  tests/test_main.cpp
  IPMMotorSim/motormodel.cpp
  IPMMotorSim/terminal_stubs.cpp
  IPMMotorSim/run_orchestrator.cpp
  IPMMotorSim/run_orchestrator.h
  IPMMotorSim/cpp_teststubs.cpp
  IPMMotorSim/teststubs.c
  src/sim/controller.cpp
  src/sim/modulator.cpp
  src/sim/modulator_diagnostics.cpp
  src/sim/inverter_switching_model.cpp
  src/sim/metrics/metric_pack.cpp
  src/sim/util/process_utils.cpp
  src/sim/sweep/sweep_config.cpp
  src/sim/sweep/sweep_runner.cpp
  src/sim/sim_runner.cpp
  src/sim/pwm_timeline.cpp
  src/sim/power_module.cpp
  src/sim/strategy/strategy_supervisor.cpp
  src/sim/strategy/strategy_spec.cpp
  src/sim/lut/lut_builder.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/params.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/picontroller.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/sine_core.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/my_string.c
  IPMMotorSim/stm32-sine/libopeninv/src/errormessage.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/foc.cpp
  IPMMotorSim/stm32-sine/src/pwmgeneration-foc.cpp
  IPMMotorSim/stm32-sine/src/pwmgeneration.cpp
)

target_include_directories(unit_tests PRIVATE
  IPMMotorSim
  src
  src/sim
  IPMMotorSim/stm32-sine/include
  IPMMotorSim/stm32-sine/libopencm3/include
  IPMMotorSim/stm32-sine/libopeninv/include
)

target_compile_definitions(unit_tests PRIVATE
  QT_DEPRECATED_WARNINGS
  CTRL_FOC=1
  CTRL_SINE=0
  CONTROL=1
  STM32F1
  GITHUB_RUN_NUMBER=0
)

if(MSVC)
  target_compile_options(unit_tests PRIVATE /Zc:__cplusplus)
endif()

target_link_libraries(unit_tests PRIVATE ${QT_CORE_LIBS})

add_test(NAME unit_tests COMMAND unit_tests)

# CLI tool
add_executable(simtool
  src/tools/simtool/main.cpp
  IPMMotorSim/motormodel.cpp
  IPMMotorSim/terminal_stubs.cpp
  IPMMotorSim/cpp_teststubs.cpp
  IPMMotorSim/teststubs.c
  src/sim/controller.cpp
  src/sim/modulator.cpp
  src/sim/modulator_diagnostics.cpp
  src/sim/inverter_switching_model.cpp
  src/sim/metrics/metric_pack.cpp
  src/sim/util/process_utils.cpp
  src/sim/util/powerstage_presets.cpp
  src/sim/sweep/sweep_config.cpp
  src/sim/sweep/sweep_runner.cpp
  src/sim/sim_runner.cpp
  src/sim/pwm_timeline.cpp
  src/sim/power_module.cpp
  src/sim/strategy/strategy_supervisor.cpp
  src/sim/strategy/strategy_spec.cpp
  src/sim/lut/lut_builder.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/params.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/picontroller.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/sine_core.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/my_string.c
  IPMMotorSim/stm32-sine/libopeninv/src/errormessage.cpp
  IPMMotorSim/stm32-sine/libopeninv/src/foc.cpp
  IPMMotorSim/stm32-sine/src/pwmgeneration-foc.cpp
  IPMMotorSim/stm32-sine/src/pwmgeneration.cpp
)

target_include_directories(simtool PRIVATE
  IPMMotorSim
  src
  src/sim
  IPMMotorSim/stm32-sine/include
  IPMMotorSim/stm32-sine/libopencm3/include
  IPMMotorSim/stm32-sine/libopeninv/include
)

target_compile_definitions(simtool PRIVATE
  QT_DEPRECATED_WARNINGS
  CTRL_FOC=1
  CTRL_SINE=0
  CONTROL=1
  STM32F1
  GITHUB_RUN_NUMBER=0
)

if(MSVC)
  target_compile_options(simtool PRIVATE /Zc:__cplusplus)
endif()

target_link_libraries(simtool PRIVATE ${QT_CORE_LIBS})

# Windows: copy Qt runtime DLLs next to built executables so they run from the build folder
# (and so ctest/unit_tests doesn't fail with missing Qt*.dll).
if(WIN32)
  # Try to locate windeployqt from the Qt installation used by CMake.
  set(_qt_qmake "")
  if(Qt6_FOUND)
    get_target_property(_qt_qmake Qt6::qmake IMPORTED_LOCATION)
  else()
    get_target_property(_qt_qmake Qt5::qmake IMPORTED_LOCATION)
  endif()

  set(_qt_bin_hint "")
  if(_qt_qmake)
    get_filename_component(_qt_bin_hint "${_qt_qmake}" DIRECTORY)
  endif()

  find_program(WINDEPLOYQT_EXECUTABLE
    NAMES windeployqt windeployqt.exe
    HINTS "${_qt_bin_hint}"
  )

  function(_deploy_qt_for_target _tgt)
    if(NOT WINDEPLOYQT_EXECUTABLE)
      message(STATUS "windeployqt not found; ${_tgt} will require Qt bin on PATH to run.")
      return()
    endif()
    add_custom_command(TARGET ${_tgt} POST_BUILD
      COMMAND "${WINDEPLOYQT_EXECUTABLE}"
              --no-translations
              --no-system-d3d-compiler
              "$<TARGET_FILE:${_tgt}>"
      VERBATIM
    )
  endfunction()

  _deploy_qt_for_target(IPMMotorSim)
  _deploy_qt_for_target(unit_tests)
  _deploy_qt_for_target(simtool)
endif()

# Ship power-stage presets next to the executable so the UI preset dropdown works
# when running from the build output directory (and when packaging the build folder).
add_custom_command(TARGET IPMMotorSim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/powerstages.yml
          $<TARGET_FILE_DIR:IPMMotorSim>/powerstages.yml
  VERBATIM
)

add_custom_command(TARGET simtool POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/powerstages.yml
          $<TARGET_FILE_DIR:simtool>/powerstages.yml
  VERBATIM
)

# Ship example sweep configs + driving cycles + python tools next to the executables
# so the GUI/CLI pipeline works when launched from the build output directory (or a packaged ZIP).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/configs")
  add_custom_command(TARGET IPMMotorSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/configs
            $<TARGET_FILE_DIR:IPMMotorSim>/configs
    VERBATIM
  )
  add_custom_command(TARGET simtool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/configs
            $<TARGET_FILE_DIR:simtool>/configs
    VERBATIM
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cycles")
  add_custom_command(TARGET IPMMotorSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/cycles
            $<TARGET_FILE_DIR:IPMMotorSim>/cycles
    VERBATIM
  )
  add_custom_command(TARGET simtool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/cycles
            $<TARGET_FILE_DIR:simtool>/cycles
    VERBATIM
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools")
  add_custom_command(TARGET IPMMotorSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/tools
            $<TARGET_FILE_DIR:IPMMotorSim>/tools
    VERBATIM
  )
  add_custom_command(TARGET simtool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/tools
            $<TARGET_FILE_DIR:simtool>/tools
    VERBATIM
  )
endif()

# Ship OpenInverter parameter presets next to the executable so the UI preset dropdown works
# when running from the build output directory (and when packaging the build folder).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/params")
  add_custom_command(TARGET IPMMotorSim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/params
            $<TARGET_FILE_DIR:IPMMotorSim>/params
    VERBATIM
  )
endif()
