cmake_minimum_required(VERSION 3.0.2)
project(pMicrostrainIMU VERSION 0.0.1 LANGUAGES CXX)

add_compile_options(-std=c++11)

## MOOS
set(MOOS_DIR "/opt/moos-ivp/build/MOOS/MOOSCore")
find_package(MOOS REQUIRED)
include_directories(${MOOS_INCLUDE_DIRS})
link_directories(${MOOS_LIBRARY_DIRS})

## MSCL
find_library(MSCL_LIB_PATH NAMES libmscl.so PATHS "/usr/share/c++-mscl" DOC "MSCL Library" NO_DEFAULT_PATH)
set(MSCL_LIB "usr/share/c++-mscl")
set(MSCL_INC_PATH "/usr/share/c++-mscl/source")
include_directories(
  ${MIPLIBINC}
  ${MIPUTILINC}
  ${MSCL_INC_PATH}
  ${MSCL_LIB}
  ${MSCL_INC}
)

file(GLOB SRC
  "src/*.cpp"
)

add_executable(${PROJECT_NAME}
  ${SRC}
)

target_link_libraries(${PROJECT_NAME}
  ${MOOS_LIBRARIES}
  ${MSCL_LIB_PATH}
)

file(
  COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/pTestImu.moos
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)