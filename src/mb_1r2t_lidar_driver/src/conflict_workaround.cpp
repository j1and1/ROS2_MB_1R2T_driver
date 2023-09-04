#include "mb_1r2t_lidar_driver/conflict_workaround.hpp"

#include <sys/ioctl.h>

namespace workaround
{
    int ioctl (int fd, unsigned long int request, void* arg)
    {
        return ::ioctl(fd, request, arg);
    }
}