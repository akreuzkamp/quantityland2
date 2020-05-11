cmake_minimum_required(VERSION 3.0.0)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_BUILD_DIR ${PROJECT_SOURCE_DIR}/build)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BUILD_DIR}/lib)
message( STATUS "LIBRARY_OUTPUT_PATH: " ${LIBRARY_OUTPUT_PATH} )

set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
message( STATUS "LIBRARY_INSTALL_PATH: " ${LIBRARY_INSTALL_PATH} )

file(MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})


add_library(quantityland2 INTERFACE)
target_compile_features(quantityland2 INTERFACE cxx_std_20)
install(TARGETS quantityland2 LIBRARY DESTINATION lib)

target_include_directories(quantityland2
    INTERFACE $<INSTALL_INTERFACE:include/quantityland2> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
)


set(HEADERS_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include/quantityland2)
message(STATUS "HEADERS_INSTALL_PATH : " ${HEADERS_INSTALL_PATH})


install(TARGETS quantityland2 EXPORT Quantityland2 LIBRARY DESTINATION lib PUBLIC_HEADER DESTINATION include/quantityland2)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/ DESTINATION include/quantityland2 FILES_MATCHING PATTERN "*.h*")
install(EXPORT Quantityland2 NAMESPACE Quantityland2:: DESTINATION lib/cmake/quantityland2 FILE Quantityland2Config.cmake)
