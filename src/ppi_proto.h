#ifndef PPI_PROTO_H
#define PPI_PROTO_H

#include <stdint.h>

#define PPI_PTFM_FC_FMC 0x6c /* first message cycle */
#define PPI_PTFM_FC_POLL 0x5c

#define PPI_PTFM_SD1    0x10 /* for status response */
#define PPI_PTFM_SD2    0x68 /* for data transfers */
#define PPI_PTFM_SC     0xe5 /* short acknowledge */
#define PPI_PTFM_ED     0x16

#define PPI_PTFM_FC_NAKRR    0x02

#define PPI_SVCID_READ  0x04
#define PPI_SVCID_WRITE 0x05

#define PPI_PROTOID_S7_200  0x32

#define PPI_ROSCTR_ACKEDREQ 0x01
#define PPI_ROSCTR_ACKNOPD  0x02
#define PPI_ROSCTR_ACKPD    0x03

#define PPI_DB_DTYPE_ERR    0x00
#define PPI_DB_DTYPE_BITS   0x03
#define PPI_DB_DTYPE_BYTES  0x04


#pragma pack(1)
typedef struct
{
    uint8_t sd_0;
    uint8_t le;
    uint8_t ler;
    uint8_t sd_1;
    uint8_t da;
    uint8_t sa;
    uint8_t fc;
    /* DU FCS ED */
}ppi_pthead_t;

typedef struct
{
    uint8_t sd;
    uint8_t da;
    uint8_t sa;
    uint8_t fc;
    uint8_t fcs;
    uint8_t ed;   
}ppi_ptfm1_t;

typedef struct
{
    uint8_t protoid;
    uint8_t posctr;
    uint16_t redid;
    uint16_t pduref;
    uint16_t parlg;
    uint16_t datlg;
}ppi_reqhead_t;

typedef struct
{
    uint8_t serviceid; 
    uint8_t numvar;
}ppi_pbstart_t;

typedef struct
{
    uint8_t varspec; /* 0x12 */
    uint8_t vaddrlg; /* 0x0A:sizeof(ppi_anypointer_t) */
}ppi_pbattr_t;

typedef struct
{
    uint8_t syntaxid; /* 0x10 */
    uint8_t type;
    uint16_t numele;
    uint16_t subarea; /* V:1 */
    uint8_t area;
    uint8_t offset[3];
}ppi_anypointer_t;

typedef struct
{
    uint8_t protoid;
    uint8_t posctr;
    uint16_t redid;
    uint16_t pduref;
    uint16_t parlg;
    uint16_t datlg;
    uint8_t errcls;
    uint8_t errcod;
}ppi_rsphead_t;

typedef struct
{
    uint8_t serviceid;
    uint8_t numvar;
    uint8_t accrslt[1]; /* variable length; 0xFF:no error */
}ppi_wrsppb_t;

typedef struct
{
    uint8_t accrslt;  /* 0xFF:no error */
    uint8_t dtype;    /* 3:bits 4:bytes */
    uint16_t varlg;   /* number of bits */
    uint8_t value[1]; /* variable */
}ppi_rrspdb_t;

typedef struct
{
    uint8_t reserved;
    uint8_t dtype;
    uint16_t varlg;   /* number of bits */
    uint8_t value[1]; /* variable */
}ppi_reqdb_t;

#pragma pack()

#endif
