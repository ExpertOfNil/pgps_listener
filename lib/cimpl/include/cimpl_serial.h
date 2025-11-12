#ifndef CIMPL_SERIAL_H
#define CIMPL_SERIAL_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "cimpl_core.h"

#ifndef SERIAL_DEVICE
#define SERIAL_DEVICE "/dev/tty/ACM0"
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

CimplReturn serial_configure(i32, u32);
i32 serial_start(const char*);

#ifdef CIMPL_IMPLEMENTATION
CimplReturn serial_configure(i32 fd, u32 baud) {
    struct termios tty;
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "Failed to get file descriptor attributes");
        return RETURN_ERR;
    }

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    // on linux can set speed directly w/ a number

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Failed to set file descriptor attributes");
        return RETURN_ERR;
    }
    return RETURN_OK;
}

i32 serial_start(const char* device) {
    i32 serial_fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        fprintf(stderr, "Failed to create serial fd");
        return -1;
    }

    if (serial_configure(serial_fd, SERIAL_BAUD) != RETURN_OK) {
        close(serial_fd);
        return -1;
    }

    // Flush existing data in the port
    tcflush(serial_fd, TCIOFLUSH);
    return serial_fd;
}
#endif /* CIMPL_IMPLEMENTATION */

#endif /* CIMPL_SERIAL */
