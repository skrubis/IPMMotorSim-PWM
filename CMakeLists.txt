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
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets Charts)
  set(QT_LIBS Qt5::Widgets Qt5::Charts)
endif()

add_executable(IPMMotorSim WIN32
  IPMMotorSim/main.cpp
  IPMMotorSim/mainwindow.cpp
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
  src/sim/inverter_switching_model.cpp
  src/sim/power_module.cpp
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

# Ship power-stage presets next to the executable so the UI preset dropdown works
# when running from the build output directory (and when packaging the build folder).
add_custom_command(TARGET IPMMotorSim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/powerstages.yaml
          $<TARGET_FILE_DIR:IPMMotorSim>/powerstages.yaml
  VERBATIM
)

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
