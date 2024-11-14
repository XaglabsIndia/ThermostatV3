# Adafruit GFX Library
# https://github.com/adafruit/Adafruit-GFX-Library
# BSD License

cmake_minimum_required(VERSION 3.5)

file(GLOB_RECURSE COMPONENT_SRCS "src/*.cpp" "src/*.c") 
idf_component_register(SRCS ${COMPONENT_SRCS}
                       INCLUDE_DIRS "src"
                       REQUIRES  Adafruit_BusIO Adafruit_GFX)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)