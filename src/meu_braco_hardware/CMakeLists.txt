cmake_minimum_required(VERSION 3.8)
project(meu_braco_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontra as dependências do ROS2
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)

# Procura pelo diretório que contém o header SerialPort.h
find_path(LIBSERIAL_INCLUDE_DIR
  NAMES libserial/SerialPort.h
)

# Procura pela biblioteca compilada (libserial.so)
find_library(LIBSERIAL_LIBRARY
  NAMES serial
)

# Verifica se ambos foram encontrados
if (NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
  message(FATAL_ERROR "Could not find LibSerial. Please ensure 'libserial-dev' is installed.")
endif()

add_library(${PROJECT_NAME}_system_interface SHARED
  src/meu_braco_system_interface.cpp
)

# Inclui as pastas de header (do próprio pacote e da LibSerial)
target_include_directories(${PROJECT_NAME}_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIR}
)

# Adiciona as dependências do ROS2 ao alvo
ament_target_dependencies(${PROJECT_NAME}_system_interface
  PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
)

# Linka com a biblioteca LibSerial
target_link_libraries(${PROJECT_NAME}_system_interface
  PUBLIC
  ${LIBSERIAL_LIBRARY}
)

# Exporta o plugin para o ros2_control
pluginlib_export_plugin_description_file(hardware_interface ros2_control.xml)

# Regras de instalação
install(
  TARGETS ${PROJECT_NAME}_system_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Configuração para testes (linters)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()