cmake_minimum_required(VERSION 2.8.4)
project(libadsdr)

set(VERSION_MAJOR "0")
set(VERSION_MINOR "3")
set(VERSION_PATCH "0")
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/modules")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu99")


add_definitions(-DHAVE_VERBOSE_MESSAGES)


# ADSDR Library
find_package(libusb-1.0 REQUIRED)

include_directories(${LIBUSB_1_INCLUDE_DIR})

file(GLOB LIBADSDR_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/libadsdr.c
)

file(GLOB LIBADSDR_INCLUDE_FILES
    ${PROJECT_SOURCE_DIR}/include/adsdr.h
    ${PROJECT_SOURCE_DIR}/include/adsdr_export.h
)

file(GLOB LIBAD9361_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/ad9361/*.h
    ${PROJECT_SOURCE_DIR}/src/ad9361/*.c
    ${PROJECT_SOURCE_DIR}/src/ad9361/platform_fx3/*.h
    ${PROJECT_SOURCE_DIR}/src/ad9361/platform_fx3/*.c
)

add_library(
    adsdr
    SHARED
    ${LIBAD9361_SRC_FILES}
    ${LIBADSDR_SRC_FILES}
    ${LIBADSDR_INCLUDE_FILES}
)

include_directories(${PROJECT_SOURCE_DIR}/src/ad9361)
include_directories(${PROJECT_SOURCE_DIR}/src/ad9361/platform_fx3)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("-- WARNING: Configured in DEBUG mode. This will *very heavily* impact performance!")
endif()

set_target_properties(adsdr PROPERTIES VERSION ${VERSION} SOVERSION ${VERSION_MAJOR})
target_link_libraries(adsdr ${LIBUSB_1_LIBRARIES})

# Include
include_directories(${PROJECT_SOURCE_DIR}/include)

# Install library
install(TARGETS adsdr LIBRARY DESTINATION lib)
install(FILES ${LIBADSDR_INCLUDE_FILES} DESTINATION include)

# Install adsdr-ctl and adsdr-io utilities
#--install(TARGETS adsdr-ctl adsdr-io RUNTIME DESTINATION bin)

######################################################################
#   Build utility
######################################################################
add_executable(ad_sdr ${PROJECT_SOURCE_DIR}/src/adsdr.c)

target_link_libraries(ad_sdr
    adsdr
)


######################################################################
# uninstall target
######################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)



