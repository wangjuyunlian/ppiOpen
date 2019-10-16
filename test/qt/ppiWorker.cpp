#include "ppiWorker.h"

#include "../../inc/ppi.h"

#include <QLocalSocket>

ppiWorker::ppiWorker()
{

}

ppiWorker::~ppiWorker()
{

}

void ppiWorker::run()
{
    ppi_t ctx;
    QLocalSocket ls;
    QByteArray msg;
    QString str;
    QStringList slist;

    ls.setServerName("ppi");
    ls.connectToServer();
    if (!ls.waitForConnected())
        return;

    ppi_contex_init(&ctx, NULL);
    ppi_port_driver_attach(&ctx, 0);

    while (1)
    {
        if (!ls.waitForReadyRead())
            continue;

        msg = ls.readAll();
        str = msg.toStdString().c_str();
        slist = str.split(" ");
        if (slist[0] == "open")
        {
            ppi_set_args(&ctx, slist[1].toStdString().c_str(), 0, 0);
            if (ppi_open(&ctx) != 0)
            {
                ls.write("open fail");
            }
            else
            {
                ls.write("open ok");
            }

            continue;
        }

        if (slist[0] == "close")
        {
            ppi_close(&ctx);
            ls.write("closed");
            continue;
        }

        if (slist[0] == "read")
        {
            slist.removeFirst();
            str = doread(&ctx, slist);
            str.insert(0, "ret ");
            ls.write(str.toStdString().c_str());
            continue;
        }

        if (slist[0] == "write")
        {
            slist.removeFirst();
            str = dowrite(&ctx, slist);
            str.insert(0, "ret ");
            ls.write(str.toStdString().c_str());
            continue;
        }
    }
}

void ppiWorker::ppi_gettype(QString &smt, QString &sdt, int &mt, int &dt)
{
    QMap<QString, int> mmt;
    QMap<QString, int> mdt;

    mmt["Q"] = PPI_MT_Q;
    mmt["I"] = PPI_MT_I;
    mmt["V"] = PPI_MT_V;
    mmt["S"] = PPI_MT_S;
    mmt["M"] = PPI_MT_M;

    mdt["BIT"] = PPI_DT_BIT;
    mdt["BYTE"] = PPI_DT_BYTE;
    mdt["WORD"] = PPI_DT_WORD;
    mdt["DWORD"] = PPI_DT_DWORD;

    mt = mmt[smt];
    dt = mdt[sdt];
}

QString ppiWorker::doread(void *ctx, QStringList &args)
{
    QString sret;
    int ret= -1;

    int byteoff, bitoff, num;
    QString str;
    int mt, dt;
    int da;
    char buf[128];

    /* args: da memtype dattype byteoff bitoff num content */
    da = args[0].toInt();
    ppi_gettype(args[1], args[2], mt, dt);
    byteoff = args[3].toInt();
    bitoff = args[4].toInt();
    num = args[5].toInt();

    switch (dt)
    {
    case PPI_DT_BYTE:
    {
        ret = ppi_master_read_bytes((ppi_t*)ctx, da, mt, byteoff, buf, num);
        if (ret > 0)
        {
            QByteArray tmp;

            tmp = tmp.append(buf, ret);
            tmp = tmp.toHex(' ');
            sret = tmp.data();
        }
    }break;
    case PPI_DT_DWORD:
    {
        uint32_t val;

        ret = ppi_master_read_bytes((ppi_t*)ctx, da, mt, byteoff, buf, 4);
        ppi_data_to_host_dword(&val, buf, 0);
        if (ret > 0)
        {
            sret = sret.sprintf("0x%X", val);
        }
    }break;
    case PPI_DT_BIT:
    {
        ret = ppi_master_read_bit((ppi_t*)ctx, da, mt, byteoff, bitoff, (uint8_t*)buf);
        if (ret > 0)
        {
            sret = buf[0]? "1" : "0";
        }
    }break;
    case PPI_DT_WORD:
    {
        uint16_t val;

        ret = ppi_master_read_bytes((ppi_t*)ctx, da, mt, byteoff, buf, 2);
        ppi_data_to_host_word(&val, buf, 0);
        if (ret > 0)
        {
            sret = sret.sprintf("0x%X", val);
        }
    }break;
    }

    if (ret < 0)
    {
        sret = "读取失败";
    }

    return sret;
}

QString ppiWorker::dowrite(void *ctx, QStringList &args)
{
    QString sret;
    int ret = -1;
    QMap<QString, int> mmt;
    QMap<QString, int> mdt;
    int byteoff, bitoff, num;
    QString str;
    int mt, dt;
    int da;
    char buf[128];

    mmt["Q"] = PPI_MT_Q;
    mmt["I"] = PPI_MT_I;
    mmt["V"] = PPI_MT_V;
    mmt["S"] = PPI_MT_S;

    mdt["BIT"] = PPI_DT_BIT;
    mdt["BYTE"] = PPI_DT_BYTE;
    mdt["WORD"] = PPI_DT_WORD;
    mdt["DWORD"] = PPI_DT_DWORD;

    /* args: da memtype dattype byteoff bitoff num content */
    da = args[0].toInt();
    mt = mmt[args[1]];
    dt = mdt[args[2]];
    byteoff = args[3].toInt();
    bitoff = args[4].toInt();
    num = args[5].toInt();

    switch (dt)
    {
    case PPI_DT_BYTE:
    {
        if (num == 1)
        {
            buf[0] = (char)args[6].toInt();
        }
        else
        {
            num = args[6].size();
            memcpy(buf, args[6].toStdString().c_str(), (size_t)num);
        }

        ret = ppi_master_write_bytes((ppi_t*)ctx, da, mt, byteoff, buf, num);
    }break;
    case PPI_DT_DWORD:
    {
        int32_t val;

        val = args[6].toInt();
        ret = ppi_master_write_dword((ppi_t*)ctx, da, mt, byteoff, &val);
    }break;
    case PPI_DT_BIT:
    {
        buf[0] = (char)args[6].toInt();
        ret = ppi_master_write_bit((ppi_t*)ctx, da, mt, byteoff, bitoff, (uint8_t*)buf);
    }break;
    case PPI_DT_WORD:
    {
        uint16_t val;

        val = args[6].toUShort();
        ppi_data_from_host_word(&val, buf, 0);
        ret = ppi_master_write_bytes((ppi_t*)ctx, da, mt, byteoff, buf, 2);
    }break;
    }

    if (ret > 0)
    {
        sret = "写入成功";
    }
    else
    {
        sret = "写入失败";
    }

    return sret;
}

void ppiWorker::_debug(void *obj, const char* fmt, ...)
{
    va_list args;
    char buf[128];

    va_start(args, fmt);
    vsnprintf(buf, 128, fmt, args);
    qDebug(buf);
    va_end(args);
}

void ppiWorker::test_multMemRead(void *ctx)
{
    ppi_reqinfo_t ri[3];
    uint8_t buf1[12];
    uint8_t buf2[12];
    int ret;

    ri[0].buf = buf1;
    ri[0].num = 12;
    ri[0].byteoff = 0;
    ri[0].dattype = PPI_DT_BYTE;
    ri[0].memtype = PPI_MT_M;

    ri[1].buf = buf2;
    ri[1].num = 12;
    ri[1].byteoff = 100;
    ri[1].dattype = PPI_DT_BYTE;
    ri[1].memtype = PPI_MT_V;

    ppi_set_debug((ppi_t*)ctx, this, _debug);
    ret = ppi_master_read((ppi_t*)ctx, 2, ri, 2);
    qDebug("mult read[%d, %s]", ret, ri[1].buf);
}

void ppiWorker::test_bitRead(void *ctx)
{
    uint8_t buf[12];
    int ret;

    memset(buf, 0, sizeof(buf));
    ret = ppi_master_read_bit((ppi_t*)ctx, 2, PPI_MT_V, 1, 2, buf);
    qDebug("read bit [%X]", buf[0]);
}

void ppiWorker::test_errRead(void *ctx)
{
    ppi_reqinfo_t ri[3];
    uint8_t buf1[12];
    uint8_t buf2[12];
    int ret;

    ri[0].buf = buf1;
    ri[0].num = 2;
    ri[0].byteoff = 0;
    ri[0].bitoff = 0;
    ri[0].dattype = PPI_DT_BIT;
    ri[0].memtype = PPI_MT_M;

    ri[1].buf = buf2;
    ri[1].num = 12;
    ri[1].byteoff = 100;
    ri[1].dattype = PPI_DT_BYTE;
    ri[1].memtype = PPI_MT_V;

    ppi_set_debug((ppi_t*)ctx, this, _debug);
    ret = ppi_master_read((ppi_t*)ctx, 2, ri, 2);
    qDebug("err read[%d, %s]", ret, ri[1].buf);
}
