# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "."
                       REQUIRES json esp_driver_rmt
                       EMBED_TXTFILES config.min.html)
                       
target_add_binary_data(${COMPONENT_TARGET} "config.min.html" TEXT)