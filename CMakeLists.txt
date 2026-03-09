cmake_minimum_required(VERSION 3.21 FATAL_ERROR)

project(bmpio)

add_executable(bmp bmp.cc)
set_property(TARGET bmp PROPERTY CXX_STANDARD 20)

add_executable(embed embed.cc)
set_property(TARGET embed PROPERTY CXX_STANDARD 20)
