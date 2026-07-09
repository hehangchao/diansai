/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2025-08-20     Administrator    Ported to RT-Thread style (.c file)
 */

#include <rtthread.h>
#include <stdint.h>
#include "i2c_ee.h"
#include "i2c_gpio.h"

/*
 * 说明：
 * - 本文件实现 AT24C02 的基础读写/擦除/自检。
 * - I2C 底层时序由你的 software I2C（i2c_gpio.c）提供：
 *   i2c_Start, i2c_Stop, i2c_SendByte, i2c_ReadByte, i2c_WaitAck, i2c_Ack, i2c_NAck, i2c_CheckDevice
 */

/* 判断串行 EEPROM 是否存在并可应答
 * 返回：1 正常；0 异常
 */
uint8_t ee_CheckOk(void)
{
    if (i2c_CheckDevice(EEPROM_DEV_ADDR) == 0)
    {
        return 1;
    }
    else
    {
        /* 失败后务必发 Stop，避免占线 */
        i2c_Stop();
        return 0;
    }
}

/* 从 EEPROM 指定地址连续读取 _usSize 字节到 _pReadBuf
 * 返回：1 成功；0 失败
 */
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize)
{
    uint16_t i;

    /* Step1: START */
    i2c_Start();

    /* Step2: 发送器件地址 + 写 */
    i2c_SendByte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);
    if (i2c_WaitAck() != 0)          /* 无应答 */
        goto cmd_fail;

    /* Step3: 发送字节地址（24C02: 单字节地址即可） */
    i2c_SendByte((uint8_t)_usAddress);
    if (i2c_WaitAck() != 0)
        goto cmd_fail;

    /* Step4: 重新 START，改成读操作 */
    i2c_Start();
    i2c_SendByte(EEPROM_DEV_ADDR | EEPROM_I2C_RD);
    if (i2c_WaitAck() != 0)
        goto cmd_fail;

    /* Step5: 连续读取 */
    for (i = 0; i < _usSize; i++)
    {
        _pReadBuf[i] = i2c_ReadByte();
        if (i != _usSize - 1)
            i2c_Ack();      /* 中间字节回 ACK */
        else
            i2c_NAck();     /* 最后 1 字节回 NACK */
    }

    i2c_Stop();
    return 1;

cmd_fail:
    i2c_Stop();
    return 0;
}

/* 向 EEPROM 指定地址写入 _usSize 字节（页写优化）
 * 返回：1 成功；0 失败
 */
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize)
{
    uint16_t i, m;
    uint16_t usAddr = _usAddress;

    /* 采用 page write 提升效率；24C02 的 page size = 8 字节 */
    for (i = 0; i < _usSize; i++)
    {
        /* 页首或首字节：发起一次写序列（器件地址 + 写 + 字节地址） */
        if ((i == 0) || ((usAddr & (EEPROM_PAGE_SIZE - 1)) == 0))
        {
            /* 停止，触发内部写 */
            i2c_Stop();

            /* 轮询器件应答，等待内部写完成（典型 < 5ms） */
            for (m = 0; m < 1000; m++)
            {
                i2c_Start();
                i2c_SendByte(EEPROM_DEV_ADDR | EEPROM_I2C_WR);
                if (i2c_WaitAck() == 0)
                {
                    break;  /* 写周期完成 */
                }
            }
            if (m == 1000)
            {
                goto cmd_fail;   /* 超时 */
            }

            /* 发送字节地址 */
            i2c_SendByte((uint8_t)usAddr);
            if (i2c_WaitAck() != 0)
            {
                goto cmd_fail;
            }
        }

        /* 发送数据字节 */
        i2c_SendByte(_pWriteBuf[i]);
        if (i2c_WaitAck() != 0)
        {
            goto cmd_fail;
        }

        usAddr++;  /* 地址自增 */
    }

    i2c_Stop();
    return 1;

cmd_fail:
    i2c_Stop();
    return 0;
}

/* 整片擦除：写 0xFF 覆盖全容量 */
void ee_Erase(void)
{
    uint16_t i;
    uint8_t buf[EEPROM_SIZE];

    for (i = 0; i < EEPROM_SIZE; i++)
    {
        buf[i] = 0xFF;
    }

    if (ee_WriteBytes(buf, 0, EEPROM_SIZE) == 0)
    {
        rt_kprintf("EEPROM erase failed!\r\n");
        return;
    }
    else
    {
        rt_kprintf("EEPROM erase ok.\r\n");
    }
}

/* 自检：整片写入 (0..255)，再读出比对
 * 返回：1 成功；0 失败
 */
uint8_t ee_Test(void)
{
    uint16_t i;
    uint8_t write_buf[EEPROM_SIZE];
    uint8_t read_buf[EEPROM_SIZE];

    /* 检测器件 */
    if (ee_CheckOk() == 0)
    {
        rt_kprintf("No EEPROM detected!\r\n");
        return 0;
    }

    /* 准备测试数据 */
    for (i = 0; i < EEPROM_SIZE; i++)
    {
        write_buf[i] = (uint8_t)i;
    }

    /* 写入 */
    if (ee_WriteBytes(write_buf, 0, EEPROM_SIZE) == 0)
    {
        rt_kprintf("EEPROM write error!\r\n");
        return 0;
    }
    else
    {
        rt_kprintf("EEPROM write ok.\r\n");
    }

    /* 写后保险延时（页写内部周期已轮询，这里 10ms 再保守些） */
    rt_thread_mdelay(10);

    /* 读出 */
    if (ee_ReadBytes(read_buf, 0, EEPROM_SIZE) == 0)
    {
        rt_kprintf("EEPROM read error!\r\n");
        return 0;
    }
    else
    {
        rt_kprintf("EEPROM read ok, data:\r\n");
    }

    /* 校验 */
    for (i = 0; i < EEPROM_SIZE; i++)
    {
        if (read_buf[i] != write_buf[i])
        {
            rt_kprintf("0x%02X ", read_buf[i]);
            rt_kprintf("Mismatch at %u (exp 0x%02X, got 0x%02X)\r\n",
                       i, write_buf[i], read_buf[i]);
            return 0;
        }

        rt_kprintf(" %02X", read_buf[i]);
        if ((i & 15) == 15)
            rt_kprintf("\r\n");
    }

    rt_kprintf("EEPROM R/W test PASS\r\n");
    return 1;
}
