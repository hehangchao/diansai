/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2025-08-20     Administrator    the first version (RT-Thread style header)
 */
#ifndef APPLICATIONS_I2C_EE_H_
#define APPLICATIONS_I2C_EE_H_

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AT24C02 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address (7-bit base: 0x50; 8-bit write/read: 0xA0/0xA1)
 * 1 0 1 0 A2 A1 A0 R/W
 */

/* AT24C01/02 每页 8 字节；AT24C04/08A/16A 每页 16 字节 */
#define EEPROM_DEV_ADDR     0xA0    /* 24xx02 的 8-bit 设备地址 (写:0xA0/读:0xA1) */
#define EEPROM_PAGE_SIZE    8       /* 24xx02 的页面大小 */
#define EEPROM_SIZE         256     /* 24xx02 总容量（字节） */

/* API */
uint8_t ee_CheckOk(void);
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize);
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize);
void    ee_Erase(void);
uint8_t ee_Test(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_I2C_EE_H_ */
