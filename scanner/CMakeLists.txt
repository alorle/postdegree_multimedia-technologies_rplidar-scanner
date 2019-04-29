find_package(PCL 1.8 REQUIRED COMPONENTS common io)

add_executable(scanner
    scanner.cpp
    main.cpp)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(scanner LINK_PUBLIC librplidar ${PCL_LIBRARIES})

target_compile_options(scanner
    PRIVATE
        "-Wall" "-Wpedantic" "-Wextra" "-fexceptions"
        "$<$<CONFIG:DEBUG>:-O0;-g3;-ggdb>"
)