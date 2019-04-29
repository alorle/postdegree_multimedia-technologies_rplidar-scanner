#include "scanner.h"

#include <iostream>

Scanner::Scanner(const std::string &com_path) : com_path(com_path)
{
    initialize();
}

Scanner::~Scanner()
{
    delete rplidar;
}

std::shared_ptr<Scanner> Scanner::make_shared(const std::string &com_path)
{
    return std::make_shared<Scanner>(com_path);
}

void Scanner::initialize()
{
    rplidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!rplidar)
    {
        throw std::runtime_error("Could not create RPlidar driver because out of memory");
    }
}

uint32_t Scanner::valid_baud_rates[] = {115200, 256000};

bool Scanner::connect()
{
    for (auto baud_rate : valid_baud_rates)
    {
        if (rplidar->connect(com_path.c_str(), baud_rate) == 0 && rplidar->getDeviceInfo(devinfo) == 0)
        {
            return true;
        }
    }
    return false;
}

bool Scanner::isHealthy()
{
    return (rplidar->getHealth(healthinfo) == 0 && healthinfo.status != RPLIDAR_STATUS_ERROR);
}

void Scanner::showInfo()
{
    printf("RPlidar device info: \n");
    printf("- S/N: ");
    for (int pos = 0; pos < 16; ++pos)
    {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");
    printf("- Firmware Ver: %d.%02d\n", devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF);
    printf("- Hardware Rev: %d\n\n", (int)devinfo.hardware_version);
}

void Scanner::scan(const long int z_low, const long int z_high, const long int z_step)
{
    point_cloud.clear();
    point_cloud.is_dense = true;
    std::cout << "* Scanning ..." << std::endl;
    for (int z = z_low; z <= z_high; z += z_step)
    {
        std::cout << "- z = " << z << " mm" << std::endl;
        scanLevel(z);
    }
    std::cout << "* Done: " << point_cloud << std::endl;
}

void Scanner::save(const std::string &out_file)
{
    pcl::io::savePCDFileASCII(out_file, point_cloud);
}

void Scanner::scanLevel(const long int z, const long unsigned int min_points_count)
{

    rplidar->startMotor();
    rplidar->startScanNormal(true);

    rplidar_response_measurement_node_hq_t nodes[SCAN_SAMPLES];
    size_t count = SCAN_SAMPLES;

    uint nodes_in_level = 0;
    while (nodes_in_level < min_points_count)
    {
        if (rplidar->grabScanDataHq(nodes, count) == 0)
        {
            // rplidar->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count; ++pos)
            {
                float angle = nodes[pos].angle_z_q14 * 90.f / (1 << 14);
                float distance = nodes[pos].dist_mm_q2 / (1 << 2);
                if (distance > 0 && nodes[pos].quality > 0)
                {
                    angle *= M_PI / 180;
                    auto x = distance * std::cos(angle);
                    auto y = distance * std::sin(angle);
                    point_cloud.push_back(pcl::PointXYZ(x, y, z));
                    nodes_in_level++;
                }
            }
        }
    }

    rplidar->stop();
    rplidar->stopMotor();
}