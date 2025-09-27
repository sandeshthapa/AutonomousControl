cmake_minimum_required(VERSION 4.0)
project(decode_CAN)

set(CMAKE_CXX_STANDARD 20)

add_executable(decode_CAN main.cpp
       #can_msg.c
      #can_msg.h
      #moving_average.c
        filter_lib.c)
        #moving_avg_v1.c)
