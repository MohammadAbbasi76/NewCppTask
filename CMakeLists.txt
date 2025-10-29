cmake_minimum_required(VERSION 3.16)

project(
  ArduinoCodeConvertToRenesas
  VERSION 1.0.0
  LANGUAGES CXX
  DESCRIPTION "convert Arduino code to abstract layer for the Renesas MCU"
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# include(Common)

add_subdirectory(App)
