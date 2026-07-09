/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2025-08-20     Administrator    RT-Thread style port of bit-bang I2C & HP203B access
 *
 * 说明：
 * - 软件 I2C：PB10 -> SCL，PB11 -> SDA（可在本文件顶部宏里修改）
 * - 提供基础位操作：i2c_Start/Stop/SendByte/ReadByte/Ack/NAck/WaitAck/CheckDevice
 * - 提供 HP203B 访问函数：Reset/StartConv/ReadReg/WriteReg/ReadData
 * - 与头文件 i2c_gpio.h 配套使用
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <stdint.h>

#include "i2c_gpio.h"

/* ------------ 可在此处修改 I2C 软总线引脚映射 ------------ */
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN   GET_PIN(B, 10)   /* PB10 as SCL */
#endif
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN   GET_PIN(B, 11)   /* PB11 as SDA */
#endif

/* ------------ 引脚电平读写的本地宏（只在本 .c 使用） ------------ */
#define EEPROM_I2C_SCL_1()      rt_pin_write(I2C_SCL_PIN, PIN_HIGH)
#define EEPROM_I2C_SCL_0()      rt_pin_write(I2C_SCL_PIN, PIN_LOW)
#define EEPROM_I2C_SDA_1()      rt_pin_write(I2C_SDA_PIN, PIN_HIGH)
#define EEPROM_I2C_SDA_0()      rt_pin_write(I2C_SDA_PIN, PIN_LOW)
#define EEPROM_I2C_SDA_READ()   (rt_pin_read(I2C_SDA_PIN) ? 1 : 0)

/* ----------------------------------------------------------------------------
 * 位延时：优先使用 rt_hw_us_delay()，如未开启可改为 for-loop 微延时
 * ---------------------------------------------------------------------------- */
rt_weak  void i2c_Delay(void)
{
    /* 典型 I2C 400kHz 级别，2us 足够；可按需要调大或调小 */
#ifdef RT_USING_RTC   /* 仅为示意：绝大多数 BSP 都支持 rt_hw_us_delay */
    rt_hw_us_delay(2);
#else
    volatile int n = 24; while (n--) ; /* fallback 占空延时，视频率调 */
#endif
}

/* ----------------------------------------------------------------------------
 * 基础时序 API
 * ---------------------------------------------------------------------------- */
void i2c_Start(void)
{
    /* SCL=1 时，SDA 从 1 -> 0 为 START */
    EEPROM_I2C_SDA_1();
    EEPROM_I2C_SCL_1();
    i2c_Delay();
    EEPROM_I2C_SDA_0();
    i2c_Delay();
    EEPROM_I2C_SCL_0();
    i2c_Delay();
}

void i2c_Stop(void)
{
    /* SCL=1 时，SDA 从 0 -> 1 为 STOP */
    EEPROM_I2C_SDA_0();
    EEPROM_I2C_SCL_1();
    i2c_Delay();
    EEPROM_I2C_SDA_1();
    i2c_Delay();
}

void i2c_SendByte(uint8_t _ucByte)
{
    for (int i = 0; i < 8; i++)
    {
        if (_ucByte & 0x80) EEPROM_I2C_SDA_1();
        else                EEPROM_I2C_SDA_0();

        i2c_Delay();
        EEPROM_I2C_SCL_1();
        i2c_Delay();
        EEPROM_I2C_SCL_0();

        if (i == 7)
        {
            /* 释放 SDA，准备收 ACK */
            EEPROM_I2C_SDA_1();
        }
        _ucByte <<= 1;
        i2c_Delay();
    }
}

uint8_t i2c_ReadByte(void)
{
    uint8_t value = 0;

    for (int i = 0; i < 8; i++)
    {
        value <<= 1;
        EEPROM_I2C_SCL_1();
        i2c_Delay();
        if (EEPROM_I2C_SDA_READ())
            value++;
        EEPROM_I2C_SCL_0();
        i2c_Delay();
    }
    return value;
}

uint8_t i2c_WaitAck(void)
{
    uint8_t re;

    EEPROM_I2C_SDA_1();      /* 释放 SDA，等待从机拉低 */
    i2c_Delay();
    EEPROM_I2C_SCL_1();
    i2c_Delay();

    re = EEPROM_I2C_SDA_READ() ? 1 : 0;  /* 0:ACK, 1:NACK */

    EEPROM_I2C_SCL_0();
    i2c_Delay();
    return re;
}

void i2c_Ack(void)
{
    EEPROM_I2C_SDA_0();
    i2c_Delay();
    EEPROM_I2C_SCL_1();
    i2c_Delay();
    EEPROM_I2C_SCL_0();
    i2c_Delay();
    EEPROM_I2C_SDA_1();      /* 释放 SDA */
}

void i2c_NAck(void)
{
    EEPROM_I2C_SDA_1();
    i2c_Delay();
    EEPROM_I2C_SCL_1();
    i2c_Delay();
    EEPROM_I2C_SCL_0();
    i2c_Delay();
}

void i2c_CfgGpio(void)
{
    /* Open-Drain 输出，上拉由外部或内部上拉提供 */
    rt_pin_mode(I2C_SCL_PIN, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(I2C_SDA_PIN, PIN_MODE_OUTPUT_OD);

    /* 默认拉高，释放总线 */
    EEPROM_I2C_SCL_1();
    EEPROM_I2C_SDA_1();
    i2c_Stop();  /* 给一个 STOP，复位总线设备 */
}

/* CPU 发地址并等待应答，判断设备是否在线
 * 返回：0 有设备；1 无应答
 */
uint8_t i2c_CheckDevice(uint8_t _Address)
{
    uint8_t ack;

    i2c_CfgGpio();

    i2c_Start();
    i2c_SendByte(_Address | EEPROM_I2C_WR);
    ack = i2c_WaitAck();
    i2c_Stop();

    return ack;
}

/* ----------------------------------------------------------------------------
 * HP203B 访问层
 * 设备 8-bit 地址：写 0xEC，读 0xED（7-bit 基址 0x76）
 * ---------------------------------------------------------------------------- */
char HP203B_Reset(void)
{
    i2c_Start();
    i2c_SendByte(0xEC);               /* addr + W */
    if (i2c_WaitAck() == 1) return 1;
    i2c_SendByte(0x06);               /* RESET cmd */
    if (i2c_WaitAck() == 1) return 2;
    i2c_Stop();
    return 0;
}

char HP203B_StartConv(void)
{
    i2c_Start();
    i2c_SendByte(0xEC);               /* addr + W */
    if (i2c_WaitAck() == 1) return 1;
    i2c_SendByte(0x40);               /* CONV cmd: OSR4096, P/T channel */
    if (i2c_WaitAck() == 1) return 2;
    i2c_Stop();
    return 0;
}

uint8_t HP203B_ReadReg(uint8_t reg_addr)
{
    uint8_t val;

    i2c_Start();
    i2c_SendByte(0xEC);               /* addr + W */
    if (i2c_WaitAck() == 1) return 1;
    i2c_SendByte(0x80 | reg_addr);    /* Read register command + addr */
    if (i2c_WaitAck() == 1) return 2;
    i2c_Stop();

    i2c_Start();
    i2c_SendByte(0xED);               /* addr + R */
    if (i2c_WaitAck() == 1) return 3; /* 更严格：原代码没等待，此处补上更稳妥 */
    val = i2c_ReadByte();
    i2c_NAck();
    i2c_Stop();

    return val;
}

char HP203B_WriteReg(uint8_t reg_addr, uint8_t reg_val)
{
    i2c_Start();
    i2c_SendByte(0xEC);               /* addr + W */
    if (i2c_WaitAck() == 1) return 1;

    i2c_SendByte(0xC0 | reg_addr);    /* Write register command + addr */
    if (i2c_WaitAck() == 1) return 2;

    i2c_SendByte(reg_val);
    if (i2c_WaitAck() == 1) return 3;

    i2c_Stop();
    return 0;
}

char HP203B_ReadData(float *T, float *P, float *A)
{
    uint8_t buf[6];
    int32_t data;

    /* -------- Read Pressure -------- */
    i2c_Start();
    i2c_SendByte(0xEC);
    if (i2c_WaitAck() == 1) return 1;
    i2c_SendByte(0x30);               /* read P */
    if (i2c_WaitAck() == 1) return 2;
    i2c_Stop();

    i2c_Start();
    i2c_SendByte(0xED);
    if (i2c_WaitAck() == 1) return 3;
    buf[0] = i2c_ReadByte(); i2c_Ack();
    buf[1] = i2c_ReadByte(); i2c_Ack();
    buf[2] = i2c_ReadByte(); i2c_NAck();
    i2c_Stop();

    data  = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    data &= 0xFFFFF;
    if (data & 0x80000) data |= 0xFFF00000;  /* 符号扩展 */
    if (P) *P = data / 100.0f;               /* mbar */

    /* -------- Read Temperature -------- */
    i2c_Start();
    i2c_SendByte(0xEC);
    if (i2c_WaitAck() == 1) return 4;
    i2c_SendByte(0x32);               /* read T */
    if (i2c_WaitAck() == 1) return 5;
    i2c_Stop();

    i2c_Start();
    i2c_SendByte(0xED);
    if (i2c_WaitAck() == 1) return 6;
    buf[0] = i2c_ReadByte(); i2c_Ack();
    buf[1] = i2c_ReadByte(); i2c_Ack();
    buf[2] = i2c_ReadByte(); i2c_NAck();     /* 最后 1 字节应回 NACK */
    i2c_Stop();

    data  = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    data &= 0xFFFFF;
    if (data & 0x80000) data |= 0xFFF00000;
    if (T) *T = data / 100.0f;               /* 摄氏度 */

    /* -------- Read Altitude -------- */
    i2c_Start();
    i2c_SendByte(0xEC);
    if (i2c_WaitAck() == 1) return 7;
    i2c_SendByte(0x31);               /* read A */
    if (i2c_WaitAck() == 1) return 8;
    i2c_Stop();

    i2c_Start();
    i2c_SendByte(0xED);
    if (i2c_WaitAck() == 1) return 9;
    buf[0] = i2c_ReadByte(); i2c_Ack();
    buf[1] = i2c_ReadByte(); i2c_Ack();
    buf[2] = i2c_ReadByte(); i2c_NAck();
    i2c_Stop();

    data  = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    data &= 0xFFFFF;
    if (data & 0x80000) data |= 0xFFF00000;
    if (A) *A = data / 100.0f;               /* 0.01 m -> /100 => m */

    return 0;
}
