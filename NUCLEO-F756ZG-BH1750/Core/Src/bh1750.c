/*
 * bh1750.c
 *
 *  Created on: 9 sie 2026
 *      Author: Grzegorz
 */
#include "bh1750.h"

#define BH1750_TIMEOUT   100U   /* ms */

/* Wyslanie pojedynczego bajtu-rozkazu (BH1750 nie ma rejestrow) */
HAL_StatusTypeDef BH1750_SendCmd(BH1750_t *dev, uint8_t cmd) {
	return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, &cmd, 1,
			BH1750_TIMEOUT);
}

HAL_StatusTypeDef BH1750_Init(BH1750_t *dev, I2C_HandleTypeDef *hi2c,
		uint8_t addr, uint8_t mode) {
	HAL_StatusTypeDef st;

	dev->hi2c = hi2c;
	dev->addr = addr;
	dev->mode = mode;

	/* Sprawdzenie, czy uklad w ogole odpowiada ACK-iem */
	st = HAL_I2C_IsDeviceReady(hi2c, addr, 3, BH1750_TIMEOUT);
	if (st != HAL_OK)
		return st;

	st = BH1750_SendCmd(dev, BH1750_POWER_ON);
	if (st != HAL_OK)
		return st;

	st = BH1750_SendCmd(dev, BH1750_RESET);
	if (st != HAL_OK)
		return st;

	st = BH1750_SendCmd(dev, mode);
	if (st != HAL_OK)
		return st;

	/* Pierwszy pomiar w trybie H-res trwa ok. 120 ms */
	HAL_Delay(180);
	return HAL_OK;
}

/* Odczyt 2 bajtow - najpierw starszy, potem mlodszy */
HAL_StatusTypeDef BH1750_ReadRaw(BH1750_t *dev, uint16_t *raw) {
	uint8_t buf[2];
	HAL_StatusTypeDef st;

	st = HAL_I2C_Master_Receive(dev->hi2c, dev->addr, buf, 2, BH1750_TIMEOUT);
	if (st != HAL_OK)
		return st;

	*raw = ((uint16_t) buf[0] << 8) | buf[1];
	return HAL_OK;
}

/* Przeliczenie na luksy: lx = raw / 1.2 (w trybie 2 dodatkowo / 2) */
HAL_StatusTypeDef BH1750_ReadLux(BH1750_t *dev, float *lux) {
	uint16_t raw;
	HAL_StatusTypeDef st = BH1750_ReadRaw(dev, &raw);
	if (st != HAL_OK)
		return st;

	*lux = (float) raw / 1.2f;

	if (dev->mode == BH1750_CONT_H_RES2 || dev->mode == BH1750_ONE_H_RES2) {
		*lux /= 2.0f;
	}
	return HAL_OK;
}

