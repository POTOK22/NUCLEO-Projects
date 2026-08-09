/*
 * bh1750.h
 *
 *  Created on: 9 sie 2026
 *      Author: Grzegorz
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include "main.h"          /* ciagnie wlasciwy stm32xxxx_hal.h dla Twojej serii */
#include <stdint.h>

/* Adresy 7-bitowe przesuniete o 1 w lewo - HAL oczekuje adresu 8-bitowego */
#define BH1750_ADDR_LOW         (0x23 << 1)   /* ADDR = GND lub pin niepodlaczony */
#define BH1750_ADDR_HIGH        (0x5C << 1)   /* ADDR = VCC */

/* Rozkazy (opcodes) */
#define BH1750_POWER_DOWN       0x00
#define BH1750_POWER_ON         0x01
#define BH1750_RESET            0x07  /* kasuje rejestr wyniku, wymaga POWER_ON */
#define BH1750_CONT_H_RES       0x10  /* ciagly, 1 lx,   ~120 ms */
#define BH1750_CONT_H_RES2      0x11  /* ciagly, 0.5 lx, ~120 ms */
#define BH1750_CONT_L_RES       0x13  /* ciagly, 4 lx,   ~16 ms  */
#define BH1750_ONE_H_RES        0x20  /* jednorazowy, potem power down */
#define BH1750_ONE_H_RES2       0x21
#define BH1750_ONE_L_RES        0x23

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;       /* BH1750_ADDR_LOW / BH1750_ADDR_HIGH */
    uint8_t mode;       /* aktualny tryb pomiarowy */
} BH1750_t;

HAL_StatusTypeDef BH1750_Init(BH1750_t *dev, I2C_HandleTypeDef *hi2c,
                              uint8_t addr, uint8_t mode);
HAL_StatusTypeDef BH1750_SendCmd(BH1750_t *dev, uint8_t cmd);
HAL_StatusTypeDef BH1750_ReadRaw(BH1750_t *dev, uint16_t *raw);
HAL_StatusTypeDef BH1750_ReadLux(BH1750_t *dev, float *lux);

#endif /* INC_BH1750_H_ */
