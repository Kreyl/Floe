/*
 * Effects.cpp
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#include "Effects.h"
#include "LEDs.h"
#include "ch.h"

#define BOTTOM_MIN_V    4
#define BOTTOM_MAX_V    18
#define TOP_MIN_V       45
#define TOP_MAX_V       100

static Color_t RGBs[LED_CNT];
static thread_reference_t ThdRef = nullptr;
static Effect_t EffEmpty{306, {0, 100, 100}, {120, 100, 100}, {240, 100, 100}};
static Effect_t &CurrEff = EffEmpty;

static void ITmrCallback(void *p);

class Pixel_t {
private:
    uint32_t tStart = 0;
    ColorHSV_t CurrClr = hsvBlack, TargetClr = hsvBlack;
    uint32_t CurrV = BOTTOM_MIN_V;
    virtual_timer_t ITmr;
public:
    Color_t ToRGB() { return CurrClr.ToRGB(); }

    void Restart() {
        chVTReset(&ITmr);
        TargetClr = CurrEff.GetNewClr();
        TargetClr.V = Random::Generate(TOP_MIN_V, TOP_MAX_V);
        CurrClr.H = TargetClr.H;
        uint32_t Delay = CurrClr.AdjustAndGetDelay(TargetClr, CurrEff.SmoothValue);
        if(Delay == 0) Delay = 1;
        chVTSet(&ITmr, TIME_MS2I(1), ITmrCallback, this);
    }

    void IOnTmrI() {
        uint32_t Delay = CurrClr.AdjustAndGetDelay(TargetClr, CurrEff.SmoothValue);
        if(Delay == 0) {
            Delay = 1;
            // if on top brt, go down
            if(CurrClr.V >= TOP_MIN_V) {
//                TargetClr.V = Random::Generate(BOTTOM_MIN_V, BOTTOM_MAX_V);
                TargetClr.V = BOTTOM_MIN_V;
            }
            // if on bottom brt, change color and go up
            else {
                TargetClr = CurrEff.GetNewClr();
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

void ToRGBs(Color_t *PDst) {
    for(uint32_t i=0; i<LED_CNT; i++) RGBs[i] = Pixels[i].ToRGB();
}

// Thread
static THD_WORKING_AREA(waEffThread, 256);
static void EffThread(void *arg) {
    chRegSetThreadName("Eff");
    while(true) {
        chSysLock();
        chThdSuspendS(&ThdRef); // Wait IRQ
        chSysUnlock();
        // Show it
        ToRGBs(RGBs);
        Leds::ShowPic(RGBs);
    }
}


namespace Effects {

void Init() {
    chThdCreateStatic(waEffThread, sizeof(waEffThread), NORMALPRIO, (tfunc_t)EffThread, NULL);
    for(auto &Pix : Pixels) Pix.Restart();
}

void Set(Effect_t &Eff) { CurrEff = Eff; }

} // namespace
