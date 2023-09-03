#pragma once

/**
 * This header contains defenition for IOCTL as ioctl conflicts with asm/termios header
*/

namespace workaround
{
    // TODO: make the argument list dynamic
    int ioctl (int fd, unsigned long int request, void* arg);
}