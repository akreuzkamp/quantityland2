cmake_minimum_required(VERSION 3.0.0)
project(Quantityland2 CXX)
include(CheckCXXCompilerFlag)

set(CMAKE_CXX_STANDARD 20)
#set(CMAKE_CXX_FLAGS -fconcepts)


#Let's build shared libs (*.so)
OPTION (BUILD_SHARED_LIBS "Build shared libraries." ON)

add_subdirectory(src)
add_subdirectory(tests)
enable_testing()


