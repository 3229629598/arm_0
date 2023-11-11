cmake_minimum_required(VERSION 3.8)
project(serial_cpp)

find_package (ament_cmake_auto REQUIRED) #寻找依赖项
ament_auto_find_build_dependencies ()

include_directories(include) #头文件目录

ament_auto_add_library (${PROJECT_NAME} SHARED  #制作动态链接库，将可执行文件和自定义的库文件链接
  src/serial_cpp.cpp
  src/serial_port.cpp
  src/serial_driver.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node src/serial_cpp.cpp)

ament_auto_package (  #安装package.xml，安装配置文件
  INSTALL_TO_SHARE
  launch
  config
)

