add_executable(scanner main.cpp)
target_link_libraries(scanner LINK_PUBLIC librplidar)

target_compile_options(scanner
    PRIVATE
        "-Wall" "-Wpedantic" "-Wextra" "-fexceptions"
        "$<$<CONFIG:DEBUG>:-O0;-g3;-ggdb>"
)