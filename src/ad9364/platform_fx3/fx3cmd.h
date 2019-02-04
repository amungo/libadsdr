//
// Created by gaudima on 9/3/18.
//

#ifndef LIBADSDR_FX3CMD_H
#define LIBADSDR_FX3CMD_H

typedef enum {
    REG_SPI_READ8                      = 0xB5,
    REG_SPI_WRITE8                     = 0xB6,
    DEVICE_START                       = 0xBA,
    DEVICE_STOP                        = 0xBB,
    DEVICE_RESET                       = 0xB3,
    REG_SPI_WRITE_READ_MULTIPLE_STAGE1 = 0xC0,
    REG_SPI_WRITE_READ_MULTIPLE_STAGE2 = 0xC1,
    DEVICE_GPIO_AD_ENABLE_DISABLE      = 0xC2
} fx3cmd;

#endif //LIBADSDR_FX3CMD_H
