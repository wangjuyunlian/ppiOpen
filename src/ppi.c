#include "../inc/ppi.h"
#include "ppi_proto.h"

#include <string.h>

#define ppi_debug(ctx, ...)                   \
    if (ctx->debug)                           \
    {                                         \
        ctx->debug(ctx->dbgdev, __VA_ARGS__); \
    }

#define PPI_RSPHEAD_POS 7
#define PPI_PBSTART_POS 17
#define PPI_RSPPB_POS 19
#define PPI_RSPDB_POS 21

static uint16_t _cpu_to_be16(uint16_t v)
{
    uint16_t e = 0x1234;
    uint8_t *p = (uint8_t *)&e;

    if (p[0] == 0x34)
        v = (uint16_t)((v << 8) | (v >> 8));

    return v;
}

static uint16_t _cpu_from_be16(uint16_t v)
{
    uint16_t e = 0x1234;
    uint8_t *p = (uint8_t *)&e;

    if (p[0] == 0x34)
        v = (uint16_t)((v << 8) | (v >> 8));

    return v;
}

static uint32_t _cpu_to_be32(uint32_t v)
{
    uint16_t e = 0x1234;
    uint8_t *p = (uint8_t *)&e;

    if (p[0] == 0x34)
    {
        v = (v << 24) | ((v&0xff00) << 8) | \
            ((v&0xff0000) >> 8) | (v >> 24);
    }

    return v;
}

static uint8_t ppi_checksum(uint8_t *buf, int size)
{
    uint8_t sum = 0;
    int i;

    for (i = 0; i < size; i++)
        sum += buf[i];

    return sum;
}

static int _pthead_set(ppi_t *ctx, uint8_t da)
{
    ppi_pthead_t *h;

    h = (ppi_pthead_t *)(ctx->buf);

    h->sd_0 = PPI_PTFM_SD2;
    h->sd_1 = PPI_PTFM_SD2;
    h->da = da;
    h->sa = ctx->id;
    h->fc = 0x6c;

    return 3;
}

static int _ptend_set(ppi_t *ctx, uint8_t le)
{
    ppi_pthead_t *h = (ppi_pthead_t *)ctx->buf;

    h->le = le;
    h->ler = le;
    ctx->buf[le + 4] = ppi_checksum((uint8_t *)&h->da, le);
    ctx->buf[le + 5] = PPI_PTFM_ED;

    return (le + 4 + 2);
}

static int _reqhead_set(ppi_t *ctx, int nreqs)
{
    ppi_reqhead_t *rh;

    rh = (ppi_reqhead_t *)&(ctx->buf[7]);

    ctx->counter++;
    rh->protoid = PPI_PROTOID_S7_200;
    rh->posctr = PPI_ROSCTR_ACKEDREQ;
    rh->redid = 0;
    rh->pduref = ctx->counter;
    rh->parlg = _cpu_to_be16((uint16_t)(sizeof(ppi_pbstart_t) +
                                        (sizeof(ppi_anypointer_t) + sizeof(ppi_pbattr_t)) * (unsigned)nreqs));
    rh->datlg = 0;

    return sizeof(ppi_reqhead_t);
}

static void _reqhead_datlg_set(ppi_t *ctx, uint16_t datlg)
{
    ppi_reqhead_t *rh;

    rh = (ppi_reqhead_t *)&(ctx->buf[7]);

    rh->datlg = _cpu_to_be16(datlg);
}

static int _reqpar_set(ppi_t *ctx, uint8_t svcid, ppi_reqinfo_t *req, int nreqs)
{
    ppi_pbstart_t *r = (ppi_pbstart_t *)&(ctx->buf[PPI_PBSTART_POS]);
    ppi_anypointer_t *ap;
    ppi_pbattr_t *pa;
    int i;
    int size = 0;
    int pos;

    r->serviceid = svcid;
    r->numvar = (uint8_t)nreqs;

    for (i = 0; i < nreqs; i++)
    {
        uint32_t offset;

        pos = PPI_PBSTART_POS + (int)sizeof(ppi_pbstart_t) + size;
        pa = (ppi_pbattr_t *)&(ctx->buf[pos]);
        ap = (ppi_anypointer_t *)((uint8_t *)pa + sizeof(*pa));
        if (req->dattype == PPI_DT_BIT)
            offset = req->byteoff * 8 + req->bitoff;
        else
            offset = req->byteoff * 8;

        pa->varspec = 0x12;
        pa->vaddrlg = 0x0a;
        ap->syntaxid = 0x10;
        ap->type = req->dattype;
        ap->numele = _cpu_to_be16(req->num);
        ap->subarea = _cpu_to_be16((req->memtype == PPI_MT_V) ? 1 : 0); /* V:1 */
        ap->area = req->memtype;
        ap->offset[0] = (uint8_t)((offset & 0xff0000) >> 16);
        ap->offset[1] = (uint8_t)((offset & 0xff00) >> 8);
        ap->offset[2] = (uint8_t)((offset & 0xff) >> 0);

        size += (sizeof(ppi_pbattr_t) + sizeof(ppi_anypointer_t));
        if (size > PPI_MPL)
        {
            ppi_debug(ctx, "[E/PPI] too many reqs\n");
            return -PPI_E_NOMEM;
        }

        req++;
    }

    return (size + (int)sizeof(ppi_pbstart_t));
}

static int _reqdb_set(ppi_t *ctx, ppi_reqinfo_t *req, int nreqs, int le)
{
    ppi_reqdb_t *rd;
    int pos = 0;
    int i;
    int ret = 0;

    pos = le + 4 + ret;

    for (i = 0; i < nreqs;)
    {
        uint16_t len;

        if (pos > 240) //todo
            return -PPI_E_NOMEM;

        rd = (ppi_reqdb_t *)&ctx->buf[pos];

        rd->reserved = 0;
        if (req->dattype == PPI_DT_BIT)
        {
            len = req->num;
            rd->dtype = PPI_DB_DTYPE_BITS;
            rd->varlg = _cpu_to_be16((uint16_t)len);
            len = (len + 7) / 8;
        }
        else
        {
            if (req->dattype == PPI_DT_BYTE)
                len = 1;
            else if (req->dattype == PPI_DT_WORD)
                len = 2;
            else if (req->dattype == PPI_DT_DWORD)
                len = 4;

            len *= req->num;
            rd->dtype = PPI_DB_DTYPE_BYTES;
            rd->varlg = _cpu_to_be16((uint16_t)(len * 8));
        }

        memcpy(rd->value, req->buf, len);

        i++;
        if ((i != nreqs) && (len & 0x01))
        {
            len++;
        }

        ret += (len + sizeof(ppi_reqdb_t) - 1);
        pos = le + 4 + ret;
    }

    _reqhead_datlg_set(ctx, (uint16_t)ret);

    return ret;
}

static int ppi_rdframe_make(ppi_t *ctx, uint8_t da, ppi_reqinfo_t *req, int nreqs)
{
    int ret = 0;
    uint8_t le = 0;

    le += _pthead_set(ctx, da);
    le += _reqhead_set(ctx, nreqs);
    ret = _reqpar_set(ctx, PPI_SVCID_READ, req, nreqs);
    if (ret < 0)
        goto _out;

    le += ret;
    ret = _ptend_set(ctx, le);

_out:
    return ret;
}

static int ppi_wrframe_make(ppi_t *ctx, uint8_t da, ppi_reqinfo_t *req, int nreqs)
{
    int ret;
    int le = 0;

    le += _pthead_set(ctx, da);
    le += _reqhead_set(ctx, 1);
    ret = _reqpar_set(ctx, PPI_SVCID_WRITE, req, nreqs);
    if (ret < 0)
        goto _out;

    le += ret;
    ret = _reqdb_set(ctx, req, nreqs, le);
    if (ret < 0)
        goto _out;

    le += ret;
    ret = _ptend_set(ctx, le);

_out:
    return ret;
}

static int ppi_lock(ppi_t *ctx)
{
    if (!ctx->mutex)
        return 0;

    return ctx->mutex(ctx->mutex, 1);
}

static void ppi_unlock(ppi_t *ctx)
{
    if (ctx->mutex)
        ctx->mutex(ctx->mtxdev, 0);
}

static void ppi_rs485rde(ppi_t *ctx, int mode)
{
    if (ctx->rde)
        ctx->rde(ctx->fd, mode);
}

static int ppi_send(ppi_t *ctx, void *buf, int size, int ms)
{
    uint8_t *p;
    int ret;

    ppi_rs485rde(ctx, PPI_RS485RDE_TX);

    p = (uint8_t *)buf;
    ret = ctx->ops->write(ctx, p, size, ms);
    if (ret <= 0)
        goto _out;

    size -= ret;
    p += ret;

    while (size > 0)
    {
        ret = ctx->ops->write(ctx, p, size, 50);

        if (ret <= 0)
            break;

        p += ret;
        size -= ret;
    }

_out:
    ppi_rs485rde(ctx, PPI_RS485RDE_RX);

    return (int)(p - (uint8_t *)buf);
}

static int ppi_recv(ppi_t *ctx, void *buf, int size, int ms)
{
    uint8_t *p = (uint8_t *)buf;
    int ret;

    ret = ctx->ops->read(ctx, p, size, ms);
    if (ret <= 0)
        goto _out;

    size -= ret;
    p += ret;

    while (size > 0)
    {
        ret = ctx->ops->read(ctx, p, size, 50);

        if (ret <= 0)
            break;

        p += ret;
        size -= ret;
    }

_out:
    return (int)(p - (uint8_t *)buf);
}

static void ppi_flush(ppi_t *ctx)
{
    if (ctx->ops->ioflush)
        ctx->ops->ioflush(ctx);
}

static int ppi_req_do(ppi_t *ctx, int fmle)
{
    uint8_t rsp[6] = {0};

    ppi_flush(ctx);

    if (ppi_send(ctx, ctx->buf, fmle, 100) <= 0)
    {
        ppi_debug(ctx, "[E/PPI] send fail\n");
        return -PPI_E_IO;
    }

    if (ppi_recv(ctx, rsp, 1, 50) != 1)
    {
        ppi_debug(ctx, "[E/PPI] r:SC fail\n");
        return -PPI_E_TIMEOUT;
    }

    if (rsp[0] != PPI_PTFM_SC && rsp[0] != PPI_PTFM_SD1)
    {
        ppi_debug(ctx, "PPI: ack fail[%X]\n", rsp[0]);

        ppi_recv(ctx, rsp, 6, 20);
        return -PPI_E_IO;
    }

    return 0;
}

static int ppi_rsp_poll(ppi_t *ctx, uint8_t da)
{
    uint8_t req[] = {0x10, 0, 0, 0x5C, 0, 0x16};

    req[1] = da;
    req[2] = ctx->id;
    req[4] = ppi_checksum(&req[1], 3);

    ppi_flush(ctx);
    ppi_send(ctx, req, 6, 50);

    return 0;
}

static int ppi_rsp_wait(ppi_t *ctx, uint8_t da)
{
    int ret = 0;
    uint8_t *p;
    ppi_pthead_t *h;
    ppi_rsphead_t *rh;
    int le;
    int retry;

    p = ctx->buf;
    h = (ppi_pthead_t *)p;
    rh = (ppi_rsphead_t *)&ctx->buf[PPI_RSPHEAD_POS];

    for (retry = 0;;)
    {
        ppi_rsp_poll(ctx, da);

        ret = ppi_recv(ctx, p, sizeof(ppi_pthead_t), ctx->timeout);
        if ((ret != sizeof(ppi_pthead_t)) && (ret != sizeof(ppi_ptfm1_t)))
        {
            ppi_debug(ctx, "PPI recv head fail\n");
            return -PPI_E_BADMSG;
        }

        if (h->sd_1 == PPI_PTFM_SD2)
            break;

        if (retry++ > 3)
        {
            ppi_debug(ctx, "PPI poll fail\n");
            return -PPI_E_BADMSG;
        }
    }

    if (h->sd_0 != PPI_PTFM_SD2 || h->sd_1 != PPI_PTFM_SD2 || h->le != h->ler)
    {
        ppi_debug(ctx, "PPI rsp head fail\n");
        return -PPI_E_BADMSG;
    }

    le = p[1];
    p += ret;

    ret = ppi_recv(ctx, p, le - 1, 50);
    if (ret != (le - 1))
    {
        ppi_debug(ctx, "PPI recv rsp fail\n");
        return -PPI_E_BADMSG;
    }

    if (rh->pduref != ctx->counter)
    {
        ppi_debug(ctx, "PPI PDUREF error\n");
        return -PPI_E_BADMSG;
    }

    if (ctx->buf[le + 4] != ppi_checksum(&ctx->buf[4], le))
    {
        ppi_debug(ctx, "PPI chksum fail\n");
        return -PPI_E_BADMSG;
    }

    ret += 2;

    return ret;
}

static int ppi_rdrsp_parse(ppi_t *ctx, int fmle, ppi_reqinfo_t *req, int nreqs)
{
    ppi_rsphead_t *rh;
    ppi_pbstart_t *rpb;
    ppi_rrspdb_t *rdb;
    int pos;
    int cnt = 0;

    if (fmle < 9)
    {
        ppi_debug(ctx, "[W/PPI] too short\n");
        return -PPI_E_BADMSG;
    }

    rh = (ppi_rsphead_t *)&ctx->buf[PPI_RSPHEAD_POS];
    if (rh->errcls != 0 || rh->errcod != 0)
    {
        ppi_debug(ctx, "[W/PPI] rd errcls[%X,%X]\n", rh->errcls, rh->errcod);
        return -PPI_E_ACCES;
    }

    rpb = (ppi_pbstart_t *)&ctx->buf[PPI_RSPPB_POS];
    if (rpb->serviceid != 0x04 || rpb->numvar != nreqs)
        return -PPI_E_BADMSG;

    for (pos = PPI_RSPDB_POS, cnt = 0; cnt < rpb->numvar;)
    {
        int len;

        rdb = (ppi_rrspdb_t *)&ctx->buf[pos];

        req->rspret = rdb->accrslt;

        len = _cpu_from_be16(rdb->varlg);
        len = (len + 7) / 8;
        memcpy(req->buf, rdb->value, (size_t)len);

        cnt++;
        req++;

        if ((cnt != rpb->numvar) && (len & 0x01))
            len++;

        pos += ((int)sizeof(ppi_rrspdb_t) - 1 + len);
    }

    return cnt;
}

static int ppi_wrrsp_parse(ppi_t *ctx, int fmle, ppi_reqinfo_t *req, int nreqs)
{
    ppi_rsphead_t *rh;
    ppi_wrsppb_t *rpb;
    int cnt;

    if (fmle < 9)
    {
        ppi_debug(ctx, "[W/PPI] too short\n");
        return -PPI_E_BADMSG;
    }

    rh = (ppi_rsphead_t *)&ctx->buf[PPI_RSPHEAD_POS];
    if (rh->errcls != 0 || rh->errcod != 0)
    {
        ppi_debug(ctx, "[W/PPI] wr errcls[%X,%X]\n", rh->errcls, rh->errcod);
        return -PPI_E_ACCES;
    }

    rpb = (ppi_wrsppb_t *)&ctx->buf[PPI_RSPPB_POS];
    if (rpb->serviceid != PPI_SVCID_WRITE || rpb->numvar != nreqs)
        return -PPI_E_BADMSG;

    for (cnt = 0; cnt < rpb->numvar; cnt++)
    {
        req[cnt].rspret = rpb->accrslt[cnt];
    }

    return rpb->numvar;
}

int ppi_master_read(ppi_t *ctx, uint8_t da, ppi_reqinfo_t *req, int nreqs)
{
    int ret = -1;

    if (ppi_lock(ctx) != 0)
        return -1;

    ret = ppi_rdframe_make(ctx, da, req, nreqs);
    if (ret < 0)
        goto _out;

    ret = ppi_req_do(ctx, ret);
    if (ret != 0)
        goto _out;

    ret = ppi_rsp_wait(ctx, da);
    if (ret < 0)
        goto _out;

    ret = ppi_rdrsp_parse(ctx, ret, req, nreqs);

_out:
    ppi_unlock(ctx);

    return ret;
}

int ppi_master_write(ppi_t *ctx, uint8_t da, ppi_reqinfo_t *req, int nreqs)
{
    int ret;

    if (ppi_lock(ctx) != 0)
        return -1;

    ret = ppi_wrframe_make(ctx, da, req, nreqs);
    if (ret < 0)
        goto _out;

    ret = ppi_req_do(ctx, ret);
    if (ret < 0)
        goto _out;

    ret = ppi_rsp_wait(ctx, da);
    if (ret < 0)
        goto _out;

    ret = ppi_wrrsp_parse(ctx, ret, req, nreqs);

_out:
    ppi_unlock(ctx);

    return ret;
}

void ppi_contex_init(ppi_t *ctx, char *dev)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->arg1 = 9600;
    ctx->arg2 = PPI_SERIAL_ARG2_E81;
    if (dev)
        strncpy(ctx->iodev, dev, sizeof(ctx->iodev) - 1);
    ctx->timeout = 500;
}

void ppi_set_args(ppi_t *ctx, const char *dev, int arg1, int arg2)
{
    if (arg1 > 0)
        ctx->arg1 = arg1;
    if (arg2 > 0)
        ctx->arg2 = arg2;
    if (dev)
        strncpy(ctx->iodev, dev, sizeof(ctx->iodev) - 1);
}

int ppi_set_mutex(ppi_t *ctx, void *mtxdev, int (*mutex)(void *, int))
{
    ctx->mtxdev = mtxdev;

    if (!ctx || !mutex)
        return -1;

    ctx->mutex = mutex;

    return 0;
}

void ppi_set_debug(ppi_t *ctx, void *obj, void (*out)(void *, const char *, ...))
{
    ctx->dbgdev = obj;
    ctx->debug = out;

    ppi_debug(ctx, "ppi set debug[%X]\n", out);
}

void ppi_set_rs485rde(ppi_t *ctx, void (*rde)(long fd, int mode))
{
    ctx->rde = rde;
}

void ppi_set_timeout_rsp(ppi_t *ctx, int ms)
{
    ctx->timeout = (uint16_t)ms;
}

int ppi_open(ppi_t *ctx)
{
    if (!ctx->ops || !ctx->ops->open)
        return -1;

    return ctx->ops->open(ctx, ctx->iodev, ctx->arg1, ctx->arg2);
}

void ppi_close(ppi_t *ctx)
{
    if (!ctx || !ctx->ops || !ctx->ops->open)
        return;

    ctx->ops->close(ctx);
}

int ppi_master_write_dword(ppi_t *ctx, int da, int memtype, int pos,
                           void *_edcpu_val)
{
    int ret = -1;
    ppi_reqinfo_t ri;
    uint32_t src;

    src = _cpu_to_be32(*((uint32_t*)_edcpu_val));

    ri.num = 1;
    ri.bitoff = 0;
    ri.byteoff = (uint16_t)pos;
    ri.dattype = PPI_DT_DWORD;
    ri.memtype = (uint8_t)memtype;
    ri.buf = &src;

    if (ppi_master_write(ctx, (uint8_t)da, &ri, 1) == 1)
    {
        if (ri.rspret == PPI_RSPRET_OK)
            ret = 1;
    }

    return ret;
}

int ppi_master_read_bytes(ppi_t *ctx, int da, int memtype, int pos,
                           void *dst, int num)
{
    int ret = -1;
    ppi_reqinfo_t ri;

    ri.num = (uint8_t)num;
    ri.bitoff = 0;
    ri.byteoff = (uint16_t)pos;
    ri.dattype = PPI_DT_BYTE;
    ri.memtype = (uint8_t)memtype;
    ri.buf = dst;

    if (ppi_master_read(ctx, (uint8_t)da, &ri, 1) == 1)
    {
        if (ri.rspret == PPI_RSPRET_OK)
            ret = num;
    }

    return ret;
}

int ppi_master_write_bytes(ppi_t *ctx, int da, int memtype, int pos,
                           void *src, int num)
{
    int ret = -1;
    ppi_reqinfo_t ri;

    ri.num = (uint8_t)num;
    ri.bitoff = 0;
    ri.byteoff = (uint16_t)pos;
    ri.dattype = PPI_DT_BYTE;
    ri.memtype = (uint8_t)memtype;
    ri.buf = src;

    if (ppi_master_write(ctx, (uint8_t)da, &ri, 1) == 1)
    {
        if (ri.rspret == PPI_RSPRET_OK)
            ret = num;
    }

    return ret;
}

int ppi_master_read_bit(ppi_t *ctx, int da, int memtype, int pos,
                        int bpos, uint8_t *src)
{
    int ret = -1;
    ppi_reqinfo_t ri;

    ri.num = 1;
    ri.bitoff = (uint8_t)bpos;
    ri.byteoff = (uint16_t)pos;
    ri.dattype = PPI_DT_BIT;
    ri.memtype = (uint8_t)memtype;
    ri.buf = src;

    if (ppi_master_read(ctx, (uint8_t)da, &ri, 1) == 1)
    {
        if (ri.rspret == PPI_RSPRET_OK)
            ret = 1;
    }

    return ret;
}

int ppi_master_write_bit(ppi_t *ctx, int da, int memtype, int pos,
                         int bpos, uint8_t *src)
{
    int ret = -1;
    ppi_reqinfo_t ri;

    ri.num = 1;
    ri.bitoff = (uint8_t)bpos;
    ri.byteoff = (uint16_t)pos;
    ri.dattype = PPI_DT_BIT;
    ri.memtype = (uint8_t)memtype;
    ri.buf = src;

    if (ppi_master_write(ctx, (uint8_t)da, &ri, 1) == 1)
    {
        if (ri.rspret == PPI_RSPRET_OK)
            ret = 1;
    }

    return ret;
}

int ppi_master_readreq_continue(int *zi, int da, unsigned datsize)
{
    uint8_t len;
    uint8_t _da;
    uint32_t tmp;

    tmp = *zi;
    len = tmp & 0xff;
    _da = (tmp >> 8) & 0xff;

    if (len > PPI_MPL)
        return 0;
    if (_da != da && _da != 0)
        return 0;

    len += ((sizeof(ppi_pbattr_t)) + sizeof(ppi_anypointer_t));

    if (len > PPI_MPL)
        return 0;

    tmp = (_da << 8) | len;
    *zi = tmp;

    return 1;
}

void ppi_data_to_host_dword(void *dst, void *buf, int pos)
{
    uint16_t e = 0x1234;
    uint8_t *p = (uint8_t *)&e;
    uint8_t *src;

    src = (uint8_t*)buf + pos;

    if (p[0] == 0x34)
    {
        p = (uint8_t*)dst;

        p[0] = src[3];
        p[1] = src[2];
        p[2] = src[1];
        p[3] = src[0];
    }
    else
    {
        memcpy(dst, src, 4);
    }
}

void ppi_data_to_host_word(void *dst, void *buf, int pos)
{
    uint16_t e = 0x1234;
    uint8_t *p = (uint8_t *)&e;
    uint8_t *src;

    src = (uint8_t*)buf + pos;

    if (p[0] == 0x34)
    {
        p = (uint8_t*)dst;

        p[0] = src[1];
        p[1] = src[0];
    }
    else
    {
        memcpy(dst, src, 2);
    }
}

void ppi_data_from_host_dword(void *src, void *buf, int pos)
{
    uint16_t e = 0x1234;
    uint8_t *p = (uint8_t *)&e;
    uint8_t *dst;

    dst = (uint8_t*)buf + pos;

    if (p[0] == 0x34)
    {
        p = (uint8_t*)src;

        dst[0] = p[3];
        dst[1] = p[2];
        dst[2] = p[1];
        dst[3] = p[0];
    }
    else
    {
        memcpy(dst, src, 4);
    }
}

void ppi_data_from_host_word(void *src, void *buf, int pos)
{
    uint16_t e = 0x1234;
    uint8_t *p = (uint8_t *)&e;
    uint8_t *dst;

    dst = (uint8_t*)buf + pos;

    if (p[0] == 0x34)
    {
        p = (uint8_t*)src;

        dst[0] = p[1];
        dst[1] = p[0];
    }
    else
    {
        memcpy(dst, src, 2);
    }
}
