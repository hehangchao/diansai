/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2025-08-20     Administrator    the first version (RT-Thread style header)
 */
#ifndef APPLICATIONS_I2C_GPIO_H_
#define APPLICATIONS_I2C_GPIO_H_

#include <stdint.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C 控制位：与 24C02 等器件控制字节配合使用 */
#define EEPROM_I2C_WR   0   /* 写控制 bit */
#define EEPROM_I2C_RD   1   /* 读控制 bit */

/* ---- 基础 I2C 位时序 API（软件 I2C 由 .c 实现） ---- */
void     i2c_CfgGpio(void);           /* 软件 I2C 引脚与上拉配置（内部固定为 PB10/PB11 或在 .c 中修改） */
void     i2c_Start(void);
void     i2c_Stop(void);
void     i2c_SendByte(uint8_t byte);
uint8_t  i2c_ReadByte(void);
uint8_t  i2c_WaitAck(void);
void     i2c_Ack(void);
void     i2c_NAck(void);
uint8_t  i2c_CheckDevice(uint8_t addr_8bit);

/* ---- HP203B 相关 API（基于上述软件 I2C） ---- */
char           HP203B_Reset(void);
char           HP203B_StartConv(void);
uint8_t        HP203B_ReadReg(uint8_t reg_addr);
char           HP203B_ReadData(float *T, float *P, float *A);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_I2C_GPIO_H_ */
