#include "../inc/ppi_port.h"
#include "../inc/ppi.h"

#include <QtSerialPort>

static int port_open(struct _ppi *ctx, const char* dev, int arg1, int arg2)
{
    QSerialPort *ser = (QSerialPort*)ctx->fd;

    ser->setPortName(dev);
    ser->setBaudRate(arg1);
    ser->setDataBits(QSerialPort::Data8);
    ser->setParity(QSerialPort::EvenParity);
    ser->setStopBits(QSerialPort::OneStop);
    ser->setFlowControl(QSerialPort::NoFlowControl);

    if (ser->open(QIODevice::ReadWrite))
        return 0;

    return -1;
}

static void port_close(struct _ppi *ctx)
{
    QSerialPort *ser = (QSerialPort*)ctx->fd;

    ser->close();
}

static int port_read(struct _ppi *ctx, void *buf, int size, int timeout_ms)
{
    QSerialPort *ser = (QSerialPort*)ctx->fd;

    ser->waitForReadyRead(timeout_ms);

    return ser->read((char*)buf, size);
}

static int port_write(struct _ppi *ctx, void *buf, int size, int timeout_ms)
{
    QSerialPort *ser = (QSerialPort*)ctx->fd;
    int ret;

    QThread::msleep(10);
    ret = ser->write((const char*)buf, size);
    ser->waitForBytesWritten(timeout_ms);

    return ret;
}

static void port_ioflush(struct _ppi *ctx)
{
    QSerialPort *ser = (QSerialPort*)ctx->fd;

    ser->clear();
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
    ctx->fd = (long) new QSerialPort;

    return 0;
}

void ppi_port_driver_detach(struct _ppi *ctx)
{
    QSerialPort *ser = (QSerialPort*)ctx->fd;

    delete ser;
    ctx->fd = 0;
    ctx->ops = 0;
}
