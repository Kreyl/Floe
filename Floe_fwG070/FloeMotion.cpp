/*
 * FloeMotion.cpp
 *
 *  Created on: 5 дек. 2020 г.
 *      Author: layst
 */

#include "FloeMotion.h"
#include "kl_i2cG070.h"
#include "ch.h"

static thread_reference_t ThdRef = nullptr;

class Acc_t {
public:
    int16_t x[3] = { 0 };
    Acc_t() {}
    Acc_t(int16_t a0, int16_t a1, int16_t a2) { x[0] = a0; x[1] = a1; x[2] = a2; }
    void Print() { Printf("%d\t%d\t%d\r\n", x[0],x[1],x[2]); }

    Acc_t& operator = (const Acc_t& R) {
        x[0] = R.x[0];
        x[1] = R.x[1];
        x[2] = R.x[2];
        return *this;
    }

    int16_t& operator[](const int32_t Indx) { return x[Indx]; }

    friend Acc_t operator - (const Acc_t &L, const Acc_t &R) {
        return Acc_t(L.x[0] - R.x[0], L.x[1] - R.x[1], L.x[2] - R.x[2]);
    }

//    friend Acc_t operator * (const int32_t v) {
//        return Acc_t(
//    }

    bool AllLessThan(const int32_t R) {
        if(x[0] >= R) return false;
        if(x[1] >= R) return false;
        if(x[2] >= R) return false;
        return true;
    }

    bool AnyBiggerThan(const Acc_t &R) {
        if(x[0] > R.x[0]) return true;
        if(x[1] > R.x[1]) return true;
        if(x[2] > R.x[2]) return true;
        return false;
    }

    Acc_t Percent(int32_t Percent) {
        return Acc_t(((int32_t)x[0] * Percent) / 100L, ((int32_t)x[1] * Percent) / 100L, ((int32_t)x[2] * Percent) / 100L);
    }
};

Acc_t Module(Acc_t Acc) {
    if(Acc[0] < 0) Acc[0] = - Acc[0];
    if(Acc[1] < 0) Acc[1] = - Acc[1];
    if(Acc[2] < 0) Acc[2] = - Acc[2];
    return Acc;
}

static Acc_t vNow, vPrev, vMid;
static Acc_t vDifVal;

void AcgIrqHandler();
static const PinIrq_t IIrq{ACG_IRQ_PIN, pudPullDown, AcgIrqHandler};

//#define a3mid               -4095
// ==== Knock params ====
#define DifPercent          50L
#define Delta               100L
#define MaxKnockLength      5L

enum State_t { mstMoving, mstIdle, mstWaiting, mstKnockWaitingLow } State = mstMoving;

int32_t DurationHigh = 0, DurationLow = 0;
int32_t CalmCounter = 0;

static void Update() {
    switch(State) {
        case mstMoving:
//            Printf("Mv\r");
            if(Module(vNow - vPrev).AllLessThan(Delta)) {
                CalmCounter++;
                if(CalmCounter >= 10) {
                    State = mstIdle;
                    vMid = vNow;
                    // Calculate values to compare with
                    vDifVal = Module(vMid.Percent(DifPercent));
                    Printf("vDifVal ");
                    vDifVal.Print();
                    CalmCounter = 0;
                }
            }
            else CalmCounter = 0;
            break;

        case mstIdle:
//            Printf("Idle\r");
            if(Module(vNow - vMid).AnyBiggerThan(vDifVal)) {
                State = mstWaiting;
                DurationHigh = 0;
                Printf("Mod ");
                Module(vNow - vMid).Print();
            }
            break;

        case mstWaiting:
//            Printf("Wait\r");
            DurationHigh++;
            if(DurationHigh >= MaxKnockLength) {
                // Check if calmed down
                if(Module(vNow - vPrev).AllLessThan(Delta)) {
                    CalmCounter++;
                    if(CalmCounter >= 3) { // Knock occured
                        Printf("Knock\r");
                        State = mstMoving;
                        CalmCounter = 0;
                    }
                }
                else { // Not calm
                    State = mstMoving;
                    CalmCounter = 0;
                }
                Printf("ModW ");
                Module(vNow - vPrev).Print();
            }
            break;

        case mstKnockWaitingLow:
//            Printf("dif3: %d\r", a3dif());
//            if(a3dif() < KnockEndThreshold) {
//                DurationLow++;
//                if(DurationLow >= KnockDurationLow) { // Calmed down
//                    State = mstIdle;
//                    Printf("DurationHigh: %d\r", DurationHigh);
////                    if(DurationHigh
////                    Printf(
//                }
//            }
//            else State = mstKnockWaitingHigh;
            break;
    } // switch
    // Save new values
    vPrev = vNow;
}

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
    return acgi2c.WriteRead(LIS_I2C_ADDR, &RegAddr, 1, (uint8_t*)vNow.x, 6);
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
        if(ReadData() == retvOk) Update();
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
