/*
 * FloeMotion.cpp
 *
 *  Created on: 5 дек. 2020 г.
 *      Author: layst
 */

#include "FloeMotion.h"
#include <stdint.h>
#include "kl_i2cG070.h"
#include "ch.h"

static thread_reference_t ThdRef = nullptr;
static int16_t ax[3] = { 0 };
void AcgIrqHandler();
static const PinIrq_t IIrq{ACG_IRQ_PIN, pudPullDown, AcgIrqHandler};

namespace Motion {
#define a3mid               -4095
// ==== Knock params ====
// Change this:
#define KnockStartThreshold 1206L
#define KnockEndThreshold   108L
#define KnockDurationHigh   10L
#define KnockDurationLow    3L


enum State_t { mstIdle, mstKnockWaitingHigh, mstKnockWaitingLow } State = mstIdle;

inline int32_t a3dif() { return (ax[2] > a3mid)? (ax[2] - a3mid) : (a3mid - ax[2]); }

int32_t DurationHigh = 0, DurationLow = 0;

void Update() {
//        Printf("%d\t%d\t%d\r\n", ax[0],ax[1],ax[2]);
    switch(State) {
        case mstIdle:
            Printf("dif1: %d\r", a3dif());
            if(a3dif() > KnockStartThreshold) {
                State = mstKnockWaitingHigh;
                DurationHigh = 0;
            }
            break;

        case mstKnockWaitingHigh:
            DurationHigh++;
            // Every KnockDurationHigh, check if calmed down
            if(DurationHigh % KnockDurationHigh == 0) {
                Printf("dif2: %d\r", a3dif());
                if(a3dif() < KnockEndThreshold) {
                    State = mstKnockWaitingLow;
                    DurationLow = 0;
                }
            }
            break;

        case mstKnockWaitingLow:
            Printf("dif3: %d\r", a3dif());
            if(a3dif() < KnockEndThreshold) {
                DurationLow++;
                if(DurationLow >= KnockDurationLow) { // Calmed down
                    State = mstIdle;
                    Printf("DurationHigh: %d\r", DurationHigh);
//                    if(DurationHigh
//                    Printf(
                }
            }
            else State = mstKnockWaitingHigh;
            break;
    } // switch

}

};


#if 1 // ============================ LIS3D ====================================
#define LIS_I2C_ADDR        0x19

// Reg addresses (do not change)
#define LIS_RA_WHO_AM_I     0x0F
#define LIS_RA_CTRL_REG1    0x20
#define LIS_RA_CTRL_REG2    0x21
#define LIS_RA_CTRL_REG3    0x22
#define LIS_RA_CTRL_REG4    0x23
#define LIS_RA_CTRL_REG5    0x24
#define LIS_RA_CTRL_REG6    0x25
#define LIS_RA_STATUS       0x27
#define LIS_RA_FIFO_CTRL    0x2E
#define LIS_RA_OUT_X_L      0x28

// Reg values (do not change)
#define LIS_SCALE_2G        0x00
#define LIS_SCALE_4G        0x10
#define LIS_SCALE_8G        0x20
#define LIS_SCALE_16G       0x30

#define LIS_HIRES_EN        0x08

static uint8_t ReadReg(uint8_t RegAddr, uint8_t *Value) {
    return acgi2c.WriteRead(LIS_I2C_ADDR, &RegAddr, 1, Value, 1);
}
static uint8_t WriteReg(uint8_t RegAddr, uint8_t Value) {
    uint8_t arr[2];
    arr[0] = RegAddr;
    arr[1] = Value;
    return acgi2c.Write(LIS_I2C_ADDR, arr, 2);
}

static uint8_t ReadData() {
    uint8_t RegAddr = LIS_RA_OUT_X_L | 0x80;
    return acgi2c.WriteRead(LIS_I2C_ADDR, &RegAddr, 1, (uint8_t*)ax, 6);
}
#endif

// Thread
static THD_WORKING_AREA(waAcgThread, 512);
static void AcgThread(void *arg) {
    chRegSetThreadName("Acg");
    while(true) {
        chSysLock();
        chThdSuspendS(&ThdRef); // Wait IRQ
        chSysUnlock();
        if(ReadData() == retvOk) Motion::Update();
        else Printf("Err\r");
    }
}


uint8_t FloeMotionInit() {
#if 1 // ==== Lis ====
    // Check if Lis connected
    uint8_t b = 0;
    // Try it several times
    uint8_t Rslt = retvFail;
    for(int i=0; i<99; i++) {
        if(ReadReg(LIS_RA_WHO_AM_I, &b) == retvOk) {
            Rslt = retvOk;
            break;
        }
        chThdSleepMilliseconds(1);
    }
    if(Rslt != retvOk) {
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
    // CFG3 (irqs): DRDY irq on INT1
    if(WriteReg(LIS_RA_CTRL_REG3, 0b00010000) != retvOk) return retvFail;
    // CFG4: Block data update (output registers not updated until MSB and LSB read), LSB, FullScale=8g, HighRes, no selftest
    if(WriteReg(LIS_RA_CTRL_REG4, (0x80 | LIS_SCALE_8G | LIS_HIRES_EN)) != retvOk) return retvFail;
    // CFG5: no reboot, FIFO dis, no IRQ latch
    if(WriteReg(LIS_RA_CTRL_REG5, 0) != retvOk) return retvFail;
    // CFG6: Click and irqs dis
    if(WriteReg(LIS_RA_CTRL_REG6, 0) != retvOk) return retvFail;
    // Get status and read data if available
    if(ReadReg(LIS_RA_STATUS, &b) != retvOk) return retvFail;
    if(b) ReadData();
    Printf("Lis init done\r");
#endif
    chThdCreateStatic(waAcgThread, sizeof(waAcgThread), NORMALPRIO, (tfunc_t)AcgThread, NULL);
    IIrq.Init(risefallRising);
    IIrq.EnableIrq(IRQ_PRIO_MEDIUM);
    return retvOk;
}

void AcgIrqHandler() {
    chSysLockFromISR();
    chThdResumeI(&ThdRef, MSG_OK);
    chSysUnlockFromISR();
}
