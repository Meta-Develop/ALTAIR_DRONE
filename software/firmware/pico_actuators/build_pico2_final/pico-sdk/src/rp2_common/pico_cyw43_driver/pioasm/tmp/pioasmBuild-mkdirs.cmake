# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/_deps/pico_sdk-src/tools/pioasm")
  file(MAKE_DIRECTORY "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/_deps/pico_sdk-src/tools/pioasm")
endif()
file(MAKE_DIRECTORY
  "D:/home/6.kennkyuu/ALTAIR_DRONE/software/firmware/pico_actuators/build_pico2_final/pioasm"
  "D:/home/6.kennkyuu/ALTAIR_DRONE/software/firmware/pico_actuators/build_pico2_final/pioasm-install"
  "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/tmp"
  "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
  "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src"
  "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/home/6.kennkyuu/ALTAIR_DRONE/Software/firmware/pico_actuators/build_pico2_final/pico-sdk/src/rp2_common/pico_cyw43_driver/pioasm/src/pioasmBuild-stamp${cfgdir}") # cfgdir has leading slash
endif()
