#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>

#include "../inc/ppi.h"

static int port_open(struct _ppi *ctx, const char* dev, int arg1, int arg2)
{
    int fd;
    struct termios tem;

    fd = open(dev, O_NONBLOCK);
    if (fd < 0)
        return -1;

    if (ioctl(fd, TIOCSOBUFSZ, (void *)300) != 0)
        goto _errset;
    if (ioctl(fd, TIOCSIBUFSZ, (void *)300) != 0)
        goto _errset;
    
    if (tcgetattr(fd, &tem) != 0)
        goto _errset;

    cfsetospeed(&tem, B9600);
    cfsetispeed(&tem, B9600);

    /* PARENB       Enable parity bit
       PARODD       Use odd parity instead of even */
    if ((arg2 >> 16) == 'N') 
    {
        tem.c_cflag &=~ PARENB;
    }
    else if ((arg2 >> 16) == 'E') 
    {
        /* Even */
        tem.c_cflag |= PARENB;
        tem.c_cflag &=~ PARODD;
    } 
    else 
    {
        /* Odd */
        tem.c_cflag |= PARENB;
        tem.c_cflag |= PARODD;
    }

    if (tcsetattr(fd, TCSANOW, &tem) != 0)
        goto _errset;

    ctx->fd = fd;

    return 0;

_errset:
    close(fd);

    return -1;
}

static void port_close(struct _ppi *ctx)
{
    close(ctx->fd);
}

static int port_read(struct _ppi *ctx, void *buf, int size, int timeout_ms)
{
    int ret = -1;
    fd_set set;
    struct timeval tv;

    FD_ZERO(&set);
    FD_SET(ctx->fd, &set);
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms * 1000;

    if (select(ctx->fd + 1, &set, RT_NULL, RT_NULL, &tv) == 1)
    {
        ret = read(ctx->fd, buf, size);
    }

    return ret;
}

static int port_write(struct _ppi *ctx, void *buf, int size, int timeout_ms)
{
    int ret = -1;
    fd_set set;
    struct timeval tv;

    FD_ZERO(&set);
    FD_SET(ctx->fd, &set);
    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms * 1000;

    rt_thread_mdelay(10);

    if (select(ctx->fd + 1, RT_NULL, &set, RT_NULL, &tv) == 1)
    {
        ret = write(ctx->fd, buf, size);
    }

    return ret;
}

static void port_ioflush(struct _ppi *ctx)
{
    tcflush(ctx->fd, TCIOFLUSH);
}

static const struct ppi_driver _ser_drv =
{
    port_open,
    port_close,
    port_read,
    port_write,
    port_ioflush
};

int ppi_port_driver_attach(struct _ppi *ctx, int iotype)
{
    ctx->ops = &_ser_drv;
    ctx->fd = -1;

    return 0;
}

void ppi_port_driver_detach(struct _ppi *ctx)
{
    ctx->fd = -1;
    ctx->ops = 0;
}
