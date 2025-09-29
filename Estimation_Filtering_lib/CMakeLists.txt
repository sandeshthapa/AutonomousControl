cmake_minimum_required(VERSION 3.10)
project(estimation_filtering C)

set(CMAKE_C_STANDARD 11)   # or 17 if you like

add_executable(low_pass_mov_avg
    moving_average.c          # <-- your file with int main()
    filter_lib.c    # <-- implementation of your filter
)
