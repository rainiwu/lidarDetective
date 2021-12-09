cmake_minimum_required(VERSION 3.10) 
project(lidarDetective LANGUAGES CXX CUDA)              
set(CMAKE_CXX_STANDARD 17)           
set(CMAKE_CUDA_STANDARD 11)
find_package(CUDA REQUIRED)
include_directories("${CUDA_INCLUDE_DIRS}")

include_directories(hdr)
add_subdirectory(src)
