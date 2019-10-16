#include <rtthread.h>
#include <stdio.h>

#include "../inc/ppi.h"

static void ppi_write(const char *dev, int mt, int pos, const char *str)
{
    ppi_t ctx;
    int ret;

    ppi_contex_init(&ctx, dev, 9600, PPI_SERIAL_ARG2_E81);
    ppi_port_driver_attach(&ctx, 0);
    if (ppi_open(&ctx) != 0)
    {
        printf("open dev fail\n");
        return;
    }

    ret = ppi_master_write_bytes(&ctx, 2, mt, pos, rt_strlen(str), (void*)str);
    printf("ppi BYTE write SD[%d] ret(%d)\n", 2, ret);

    ppi_close(&ctx);
    ppi_port_driver_detach(&ctx);
}

static void ppi_read(const char *dev, int mt, int pos, int size)
{
    char buf[64] = {0};
    ppi_t ctx;
    int ret;

    ppi_contex_init(&ctx, dev, 9600, PPI_SERIAL_ARG2_E81);
    ppi_port_driver_attach(&ctx, 0);
    if (ppi_open(&ctx) != 0)
    {
        printf("open dev fail\n");
        return;
    }

    ret = ppi_master_read_bytes(&ctx, 2, mt, pos, 10, buf);
    printf("ppi BYTE read SD[%d] ret(%d) string(%s)\n", 2, ret, buf);

    ppi_close(&ctx);
    ppi_port_driver_detach(&ctx);
}

static void ppi(int argc, char **argv)
{
    if (argc < 4)
    {
        printf("Usage: ppi w devname VB0 [string]    --  write mem[V] with 'string'\n");
        printf("Usage: ppi r devname VB0 [12]          --  read mem[V] 12 bytes\n");
    }

    if (argv[1][0] == 'w')
    {
        if (argc < 5)
        {
            ppi_write(argv[2], PPI_MT_V, 0, "write test");
        }
        else
        {
            ppi_write(argv[2], PPI_MT_V, 0, argv[4]);
        }
    }
    else if (argv[1][0] == 'r')
    {
        if (argc < 5)
        {
            ppi_read(argv[2], PPI_MT_V, 0, 1);
        }
        else
        {
            ppi_read(argv[2], PPI_MT_V, 0, 12);
        }
    }
}
MSH_CMD_EXPORT(ppi, ppi test);
