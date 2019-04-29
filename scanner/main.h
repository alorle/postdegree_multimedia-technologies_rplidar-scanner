#ifndef MAIN_H
#define MAIN_H

#include "scanner.h"

#include <stdio.h>
#include <string>
#include <iostream>

using namespace rp::standalone::rplidar;

struct arguments_t
{
#ifdef _WIN32
    std::string com_path = std::string("\\\\.\\com3");
#else
    std::string com_path = std::string("/dev/ttyUSB0");
#endif
    std::string out_file = "scan.pcd";
    int z_low = 0;
    int z_high = 100;
    int z_step = 5;
};

std::shared_ptr<Scanner> scanner;

/**
 * Show usage messsage and exit program
 */
void showUsageAndExit(std::string);

/**
 * Parse arguments from command line
 */
void parseArgs(arguments_t *, int, const char *[]);

#endif
