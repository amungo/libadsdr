#include <stdio.h>

#include <adsdr.h>
#include <parameters.h>
#include <unistd.h>

int main(int argc, char** argv)
{
    int r;
    adsdr_dev_t* ad_device = 0;
    const char* fw_file = "adsdrfx3.img";


    int id = adsdr_find(0);
    if(id == ADSDR_PRODUCT_ID) {
        fprintf(stderr, "Fx3 already flashed\nresetting\n");
        adsdr_reset(0);
        sleep(1);
        id = adsdr_find(0);
    }

    if(id == ADSDR_BOOT_ID) {
        fprintf(stderr, "Fx3 in bootloader mode\n");
        adsdr_flash(fw_file, 0);
        sleep(1);
    }
    else {
        fprintf(stderr, "Fx3 not found\n");
        return -1;
    }


    r = adsdr_open(&ad_device, 0);
    if(r == 0)
    {
        fprintf(stdout, "--- AD SDR device opened ---\n");
        fprintf(stdout, "--- start init sdr ---\n");
        r = adsdr_init(ad_device, adsdr_get_init_params(), adsdr_get_rx_fir_config(), adsdr_get_tx_fir_config());

        if(r == 0) {
            fprintf(stdout, "--- stop init sdr. Ok ---\n");

            //r = adsdr_init(ad_device, adsdr_get_init_params(), adsdr_get_rx_fir_config(), adsdr_get_tx_fir_config());

        }
        else
            fprintf(stdout, "--- error init sdr ---\n");

        adsdr_close(ad_device);
    }
    else
        fprintf(stderr, "Error open sdr_device: %d \n", r);

    return 0;
}

