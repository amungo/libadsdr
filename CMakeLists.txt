cmake_minimum_required(VERSION 2.8.4)
project(libadsdr)

set(VERSION_MAJOR "0")
set(VERSION_MINOR "3")
set(VERSION_PATCH "0")
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/modules")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")


# ADSDR Library
find_package(libusb-1.0 REQUIRED)

include_directories(${LIBUSB_1_INCLUDE_DIR})

file(GLOB_RECURSE LIBADSDR_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/*.hpp
    ${PROJECT_SOURCE_DIR}/src/*.cpp
)

file(GLOB_RECURSE LIBADSDR_INCLUDE_FILES
    ${PROJECT_SOURCE_DIR}/include/*.hpp
)

file(GLOB_RECURSE READERWRITERQUEUE_INCLUDE_FILES
    ${PROJECT_SOURCE_DIR}/src/readerwriterqueue/*.h
)

file(GLOB_RECURSE LIBAD9364_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/ad9364/*.h
    ${PROJECT_SOURCE_DIR}/src/ad9364/*.cpp
    ${PROJECT_SOURCE_DIR}/src/ad9364/types/*.h
    ${PROJECT_SOURCE_DIR}/src/ad9364/types/*.cpp
)

add_library(
    adsdr
    SHARED
    ${LIBADSDR_SRC_FILES}
    ${LIBADSDR_INCLUDE_FILES}
    ${LIBAD9364_SRC_FILES}
)

include_directories(${PROJECT_SOURCE_DIR}/src/ad9364)
include_directories(${PROJECT_SOURCE_DIR}/src/ad9364/types)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message("-- WARNING: Configured in DEBUG mode. This will *very heavily* impact performance!")
endif()

set_target_properties(adsdr PROPERTIES VERSION ${VERSION} SOVERSION ${VERSION_MAJOR})
target_link_libraries(adsdr ${LIBUSB_1_LIBRARIES})

# Examples
include_directories(${PROJECT_SOURCE_DIR}/include)

# Install library
install(TARGETS adsdr LIBRARY DESTINATION lib)
install(FILES ${LIBADSDR_INCLUDE_FILES} DESTINATION include)

# Install adsdr-ctl and adsdr-io utilities
#--install(TARGETS adsdr-ctl adsdr-io RUNTIME DESTINATION bin)

########################################################################
# uninstall target
########################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)



