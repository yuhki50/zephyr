cmake_minimum_required(VERSION 3.8.2)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(peripheral)

file(GLOB_RECURSE app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

zephyr_library_include_directories(src)
