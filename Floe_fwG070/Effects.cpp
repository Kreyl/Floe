/*
 * Effects.cpp
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#include "Effects.h"
#include "LEDs.h"
#include "ch.h"
#include "MsgQ.h"

#define BOTTOM_MIN_V    4
#define BOTTOM_MAX_V    18
#define TOP_MIN_V       45
#define TOP_MAX_V       100

static Color_t RGBs[LED_CNT];
static thread_reference_t ThdRef = nullptr;
static Effect_t EffEmpty{306, {0, 0, 100}};
static Effect_t *PCurrEff = &EffEmpty;

static Color_t BlinkClr;
static bool MustBlink = false;

static void ITmrCallback(void *p);

class Pixel_t {
private:
    uint32_t Delay = 0, SmoothV = 360;
    ColorHSV_t CurrClr = hsvBlack, TargetClr = hsvBlack;
    bool DoBlink = false;
    uint32_t CurrV = BOTTOM_MIN_V;
    virtual_timer_t ITmr;
public:
    Color_t ToRGB() { return CurrClr.ToRGB(); }

    void RestartI() {
        chVTResetI(&ITmr);
        TargetClr = PCurrEff->GetNewClr();
        TargetClr.V = Random::Generate(TOP_MIN_V, TOP_MAX_V);
        CurrClr.H = TargetClr.H;
        SmoothV = PCurrEff->SmoothValue;
        chVTSetI(&ITmr, TIME_MS2I(1), ITmrCallback, this);
    }

    void IOnTmrI() {
        Delay = CurrClr.AdjustAndGetDelay(TargetClr, SmoothV);
        if(Delay == 0) {
            Delay = 1;
            // if on top brt, go down
            if(CurrClr.V >= TOP_MIN_V) {
                TargetClr.V = BOTTOM_MIN_V;
            }
            // if on bottom brt, change color and go up
            else {
                TargetClr = PCurrEff->GetNewClr();
                SmoothV = PCurrEff->SmoothValue;
                TargetClr.V = Random::Generate(TOP_MIN_V, TOP_MAX_V);
                CurrClr.H = TargetClr.H;
            }
        }
        chVTSetI(&ITmr, TIME_MS2I(Delay), ITmrCallback, this);
        chThdResumeI(&ThdRef, MSG_OK);
    }
};

void ITmrCallback(void *p) {
    chSysLockFromISR();
    ((Pixel_t*)p)->IOnTmrI();
    chSysUnlockFromISR();
}

Pixel_t Pixels[LED_CNT];

// Thread
static THD_WORKING_AREA(waEffThread, 256);
static void EffThread(void *arg) {
    chRegSetThreadName("Eff");
    while(true) {
        chSysLock();
        chThdSuspendS(&ThdRef); // Wait IRQ
        chSysUnlock();

        if(MustBlink) {
            MustBlink = false;
            for(uint32_t i=0; i<LED_CNT; i++) RGBs[i] = BlinkClr;
            Leds::ShowPic(RGBs);
            chThdSleepMilliseconds(72);
        }

        // Show it
        for(uint32_t i=0; i<LED_CNT; i++) RGBs[i] = Pixels[i].ToRGB();
        Leds::ShowPic(RGBs);
    } // while
}


namespace Effects {

void Init() {
    chThdCreateStatic(waEffThread, sizeof(waEffThread), NORMALPRIO, (tfunc_t)EffThread, NULL);
    chSysLock();
    for(auto &Pix : Pixels) Pix.RestartI();
    chSysUnlock();
}

void Set(Effect_t &Eff) {
    PCurrEff = &Eff;
}

void Blink(ColorHSV_t Clr) {
    chSysLock();
    BlinkClr = Clr.ToRGB();
    MustBlink = true;
    chThdResumeI(&ThdRef, MSG_OK);
    chSysUnlock();
}

} // namespace
