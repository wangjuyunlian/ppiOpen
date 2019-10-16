#include "../inc/ppi.h"
#include <serial.h>
#include <os.h>

static int port_open(struct _ppi *ctx, const char* dev, int arg1, int arg2)
{
    SERIAL_IF_NBR fd;
    SERIAL_ERR err;
    SERIAL_IF_CFG cfg;
    CPU_INT08U parity;

    parity = (CPU_INT08U)(arg2 >> 16);
    switch (parity)
    {
    case 'N':
        parity = SERIAL_PARITY_NONE;
    break;
    case 'E':
        parity = SERIAL_PARITY_EVEN;
    break;
    case 'O':
        parity = SERIAL_PARITY_ODD;
    break;    
    }

    cfg.Baudrate = arg1;
    cfg.FlowCtrl = SERIAL_FLOW_CTRL_NONE;
    cfg.DataBits = SERIAL_STOPBITS_1;
    cfg.Parity = parity;
    cfg.DataBits = SERIAL_DATABITS_8;

    fd = Serial_Open((CPU_CHAR*)dev, &cfg, &err);
    Serial_SetLineDrv(fd, &SerialLine_Dflt, &err);
    ctx->fd = fd;

    return 0;
}

static void port_close(struct _ppi *ctx)
{
    SERIAL_IF_NBR fd;
    SERIAL_ERR err;
    
    fd = (SERIAL_IF_NBR)ctx->fd;
    Serial_Close(fd, &err);
}

static int port_read(struct _ppi *ctx, void *buf, int size, int timeout_ms)
{
    int ret;
    SERIAL_IF_NBR fd;
    SERIAL_ERR err;

    fd = (SERIAL_IF_NBR)ctx->fd;
    ret = Serial_Rd(fd, buf, size, timeout_ms, &err);

    return ret;
}

static int port_write(struct _ppi *ctx, void *buf, int size, int timeout_ms)
{
    int ret;
    SERIAL_IF_NBR fd;
    SERIAL_ERR err;

    OSTimeDlyHMSM(0, 0, 0, 15, OS_OPT_TIME_DLY, &err);
    fd = (SERIAL_IF_NBR)ctx->fd;
    ret = Serial_Wr(fd, buf, size, timeout_ms, &err);

    return ret;
}

static void port_ioflush(struct _ppi *ctx)
{

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
