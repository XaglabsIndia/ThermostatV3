# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Espressif/frameworks/esp-idf-v5.2.2/components/ulp/cmake"
  "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main"
  "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main-prefix"
  "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main-prefix/tmp"
  "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp"
  "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main-prefix/src"
  "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Lijith/Downloads/HUBV3/HUBV3/donotuse/ThermostatV3/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp${cfgdir}") # cfgdir has leading slash
endif()
