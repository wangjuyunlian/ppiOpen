#ifndef _PPI_OPEN_H
#define _PPI_OPEN_H

#include <stdint.h>
#include "ppi_port.h"

#define PPI_BUFSZ    261

#define PPI_IOTYPE_SERIAL       0
#define PPI_SERIAL_ARG2_E81     ('E' << 16 | 8 << 8 | 1)
#define PPI_SERIAL_ARG2(p,d,s)  ((unsigned char)(p) << 16 | (unsigned char)(d) << 8 | (unsigned char)s)

#define PPI_RS485RDE_RX    0
#define PPI_RS485RDE_TX    1

#define PPI_MPL    (200) /* max payload */

/* error code for API */
#define PPI_E_NOMEM    0x01
#define PPI_E_IO       0x02
#define PPI_E_TIMEOUT  0x03
#define PPI_E_BADMSG   0x04
#define PPI_E_ACCES    0x05
#define PPI_E_INVAL    0x06
/* access status */
#define PPI_RSPRET_OK    0xff /* no error */
#define PPI_RSPRET_HF    0x01 /* hardware fault */
#define PPI_RSPRET_IOA   0x03 /* illegal object access */
#define PPI_RSPRET_DTNS  0x06 /* data type not supported */
#define PPI_RSPRET_ONELE 0x0a /* object not exist or length error */

struct ppi_driver;

struct _ppi
{
    uint8_t id;
    uint16_t counter;
    uint16_t timeout; /* waitfor response timeout(default 500ms) */
    uint8_t buf[PPI_BUFSZ];
    int arg1;
    int arg2;
    void *mtxdev;
    void *dbgdev;
    char iodev[24+1];

    void (*debug)(void *dbgdev, const char *fmt, ...);
    int (*mutex)(void *mtxdev, int state);    /* return 0:success; state: 1 lock, 0 unlock */
    void (*rde)(long fd, int mode); /* mode: @PPI_RS485RDE_RX/TX */

    /* set by driver */
    const struct ppi_driver *ops;
    long fd;
};
typedef struct _ppi ppi_t;

struct ppi_driver
{
    /* return 0: success; (SERIAL: arg1 - baudrate, arg2 - parity&databits&stopbits) */
    int (*open)(struct _ppi *ctx, const char* dev, int arg1, int arg2);
    void (*close)(struct _ppi *ctx);
    int (*read)(struct _ppi *ctx, void *buf, int size, int timeout_ms);
    int (*write)(struct _ppi *ctx, void *buf, int size, int timeout_ms);
    void (*ioflush)(struct _ppi *ctx);
};

#define PPI_DT_BIT   0x01
#define PPI_DT_BYTE  0x02
#define PPI_DT_WORD  0x04
#define PPI_DT_DWORD 0x06

#define PPI_MT_S   0x04
#define PPI_MT_SM  0x05
#define PPI_MT_AI  0x06
#define PPI_MT_AQ  0x07
#define PPI_MT_C   0x1E
#define PPI_MT_I   0x81
#define PPI_MT_Q   0x82
#define PPI_MT_M   0x83
#define PPI_MT_V   0x84
#define PPI_MT_T   0x1F

typedef struct
{
    uint8_t memtype;  /* @PPI_MT_S, ... */
    uint8_t dattype;  /* @PPI_DT_BYTE, ... */
    uint16_t byteoff;
    uint8_t bitoff;   /* rang: 0~7, valid when 'dattype' is PPI_DT_BIT*/
    uint8_t num;      /* number of variables (note: not sizeof) */
    void *buf;
    uint8_t rspret;   /* @PPI_RSPRET_OK, ... */
}ppi_reqinfo_t;

#ifdef __cplusplus
extern "C" {
#endif

void ppi_contex_init(ppi_t *ctx, char *dev);
void ppi_set_args(ppi_t *ctx, const char *dev, int arg1, int arg2);
int ppi_set_mutex(ppi_t *ctx, void *mtxdev, int (*mutex)(void*, int));
void ppi_set_debug(ppi_t *ctx, void *dbgdev, void (*out)(void*, const char*, ...));
void ppi_set_rs485rde(ppi_t *ctx, void (*rde)(long fd, int mode));
void ppi_set_timeout_rsp(ppi_t *ctx, int ms);

int ppi_open(ppi_t *ctx);
void ppi_close(ppi_t *ctx);

int ppi_master_read(ppi_t *ctx, uint8_t da, ppi_reqinfo_t *req, int nreqs);
int ppi_master_write(ppi_t *ctx, uint8_t da, ppi_reqinfo_t *req, int nreqs);
int ppi_master_read_bytes(ppi_t *ctx, int da, int memtype, int pos,
                          void *dst, int num);
int ppi_master_write_bytes(ppi_t *ctx, int da, int memtype, int pos,
                          void *src, int num);
int ppi_master_read_bit(ppi_t *ctx, int da, int memtype, int pos,
                        int bpos, uint8_t *src);

int ppi_master_write_bit(ppi_t *ctx, int da, int memtype, int pos,
                        int bpos, uint8_t *src);

int ppi_master_write_dword(ppi_t *ctx, int da, int memtype, int pos,
                           void *_edcpu_val);

int ppi_master_readreq_continue(int *zi, int da, unsigned datsize);
void ppi_data_to_host_dword(void *dst, void *buf, int pos);
void ppi_data_to_host_word(void *dst, void *buf, int pos);
void ppi_data_from_host_dword(void *src, void *buf, int pos);
void ppi_data_from_host_word(void *src, void *buf, int pos);

#ifdef __cplusplus
}
#endif

#endif
