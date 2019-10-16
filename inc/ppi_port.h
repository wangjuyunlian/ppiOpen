#ifndef _PPI_PORT_H
#define _PPI_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

struct _ppi;

int ppi_port_driver_attach(struct _ppi *ctx, int iotype);
void ppi_port_driver_detach(struct _ppi *ctx);

#ifdef __cplusplus
}
#endif

#endif
