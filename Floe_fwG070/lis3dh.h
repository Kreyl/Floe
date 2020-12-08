/*
 * lis3dh.h
 *
 *  Created on: 4 ����. 2017 �.
 *      Author: Kreyl
 */

#pragma once

#include "kl_i2cG070.h"
#include "uartG070.h"

#define LIS_i2C             i2c1
#define LIS_I2C_ADDR        0x19

#define LIS_IRQ_EN          TRUE

//#define LIS_DEBUG_PINS

#ifdef LIS_DEBUG_PINS
#define LIS_DBG_PIN         GPIOA, 3
#define DBG1_SET()          PinSetHi(LIS_DBG_PIN)
#define DBG1_CLR()          PinSetLo(LIS_DBG_PIN)
#else
#define DBG1_SET()
#define DBG1_CLR()
#endif

// Reg addresses (do not change)
#define LIS_RA_WHO_AM_I     0x0F
#define LIS_RA_CTRL_REG1    0x20
#define LIS_RA_CTRL_REG2    0x21
#define LIS_RA_CTRL_REG3    0x22
#define LIS_RA_CTRL_REG4    0x23
#define LIS_RA_CTRL_REG5    0x24
#define LIS_RA_CTRL_REG6    0x25
#define LIS_RA_FIFO_CTRL    0x2E
#define LIS_RA_OUT_X_L      0x28

// Reg values (do not change)
#define LIS_SCALE_2G        0x00
#define LIS_SCALE_4G        0x10
#define LIS_SCALE_8G        0x20
#define LIS_SCALE_16G       0x30

#define LIS_HIRES_EN        0x08


struct Accelerations_t {
    int16_t ax[3] = { 0 };
};

#if LIS_IRQ_EN
class Lis3d_t : public IrqHandler_t {
#else
class Lis3d_t {
#endif
private:
    i2c_t *pi2c;
    uint8_t ReadReg(uint8_t RegAddr, uint8_t *Value) {
        return pi2c->WriteRead(LIS_I2C_ADDR, &RegAddr, 1, Value, 1);
    }
    uint8_t WriteReg(uint8_t RegAddr, uint8_t Value) {
        uint8_t arr[2];
        arr[0] = RegAddr;
        arr[1] = Value;
        return pi2c->Write(LIS_I2C_ADDR, arr, 2);
    }
public:
    Lis3d_t(i2c_t *pi2c) : pi2c(pi2c) {}
    Accelerations_t a;
    uint8_t Init() {
//        Printf("Lis3D Init\r");
#ifdef LIS_DEBUG_PINS
        PinSetupOut(LIS_DBG_PIN, omPushPull);
#endif
        // Check if Lis connected
        uint8_t b = 0, rslt;
        rslt = ReadReg(LIS_RA_WHO_AM_I, &b);
        if(rslt != retvOk) {
            Printf("Lis3D ReadReg fail\r");
            return retvFail;
        }
        if(b != 0x33) {
            Printf("Lis3D fail: %02X\r", b);
            return retvFail;
        }
        // CFG1: Output data rate = 100Hz, normal mode, XYZ enable
        if(WriteReg(LIS_RA_CTRL_REG1, 0b01010111) != retvOk) return retvFail;
        // CFG2: HPF normal mode, filter bypassed
        if(WriteReg(LIS_RA_CTRL_REG2, 0b10000000) != retvOk) return retvFail;
        // DRDY irq 0n INT1
        if(WriteReg(LIS_RA_CTRL_REG3, 0b00010000) != retvOk) return retvFail;
        // CFG4: continuos update, LSB, FullScale=8g, HighRes, no selftest
//        if(WriteReg(LIS_RA_CTRL_REG4, (LIS_SCALE_8G + LIS_HIRES_EN)) != retvOk) return;
        // CFG4: Block data update, LSB, FullScale=8g, HighRes, no selftest
        if(WriteReg(LIS_RA_CTRL_REG4, (0x80 + LIS_SCALE_8G + LIS_HIRES_EN)) != retvOk) return retvFail;
        // CFG5: no reboot, FIFO dis, no IRQ latch
        if(WriteReg(LIS_RA_CTRL_REG5, 0b00000000) != retvOk) return retvFail;
        Printf("Lis init ok\r");
        return retvOk;
    }
    uint8_t ReadData() {
        uint8_t RegAddr = LIS_RA_OUT_X_L | 0x80;
        return pi2c->WriteRead(LIS_I2C_ADDR, &RegAddr, 1, (uint8_t*)&a, 6);
    }
#if LIS_IRQ_EN
    void IIrqHandler();
#endif
};

/*
 Put this in right place:
Lis3d_t Lis;
PinIrq_t LisIrq(ACC_INT_PIN, &Lis);
...
LisIrq.EnableIrq(IRQ_PRIO_MEDIUM);
*/
