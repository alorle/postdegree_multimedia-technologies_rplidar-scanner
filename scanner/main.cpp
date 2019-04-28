#include <iostream>
#include <string>

#include "rplidar.h"

using namespace rp::standalone::rplidar;

void showUsage(std::string arg0)
{
    std::cout << "Usage: " << arg0 << " SERIAL_PORT" << std::endl;
}

int main(int argc, const char *argv[])
{
    const char *opt_com_path = "/dev/ttyUSB0";
    const _u32 baudrateArray[2] = {115200, 256000};
    _u32 arg_baud_rate = 0;
    u_result op_result;

    if (argc != 2)
    {
        showUsage(argv[0]);
        return 1;
    }
    std::cout << "RPLIDAR_SDK_VERSION: " RPLIDAR_SDK_VERSION << std::endl;

    // Read serial port from the command line...
    opt_com_path = argv[1];

    // Create the driver instance
    RPlidarDriver *drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv)
    {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
    for (size_t i = 0; i < baudRateArraySize; ++i)
    {
        if (!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result))
            {
                connectSuccess = true;
                break;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    if (!connectSuccess)
    {

        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos)
    {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n",
           devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF, (int)devinfo.hardware_version);

    // done!
on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}
