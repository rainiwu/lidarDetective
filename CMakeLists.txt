cmake_minimum_required(VERSION 3.10) 
project(lidarDetective LANGUAGES CXX CUDA)              
set(CMAKE_CXX_STANDARD 17)           

include_directories(hdr)
add_subdirectory(src)
