#include "main.h"

#include <csignal>
#include <map>

void showUsageAndExit(std::string arg0)
{
    std::cout << "Usage: " << arg0 << " [COM_PATH] OUT_FILE " << std::endl;
    exit(-1);
}

void parseArgs(arguments_t *args, int argc, const char *argv[])
{
    if (argc == 5)
    {
        args->out_file = argv[1];
        args->z_low = std::atoi(argv[2]);
        args->z_high = std::atoi(argv[3]);
        args->z_step = std::atoi(argv[4]);
        return;
    }

    if (argc == 6)
    {
        args->com_path = argv[1];
        args->out_file = argv[2];
        args->z_low = std::atoi(argv[3]);
        args->z_high = std::atoi(argv[4]);
        args->z_step = std::atoi(argv[5]);
        return;
    }

    if (argc < 5 || argc > 6)
    {
        showUsageAndExit(argv[0]);
    }
}

int main(int argc, const char *argv[])
{
    struct arguments_t args;

    // Parse user arguments
    parseArgs(&args, argc, argv);

    // Create Scanner
    scanner = Scanner::make_shared(args.com_path);

    // Connect Scanner
    if (!scanner->connect())
    {
        std::cerr << "Could not connect Sacnner to RPlidar device" << std::endl;
        return -2;
    }
    scanner->showInfo();

    // Check Scanner health
    if (!scanner->isHealthy())
    {
        std::cerr << "RPlidar device status is not correct. Restart device and try again" << std::endl;
        return -3;
    }

    scanner->scan(args.z_low, args.z_high, args.z_step);
    scanner->save(args.out_file);
    return 0;
}
