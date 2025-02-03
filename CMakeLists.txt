cmake_minimum_required(VERSION 3.22)

project(BME68x_SensorAPI)

add_library(${PROJECT_NAME}
  "bme68x.c"
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ""
)
