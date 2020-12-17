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
static Effect_t EffEmpty{306, {0, 0, 100}};
static Effect_t *PCurrEff = &EffEmpty;
static ColorHSV_t BlinkClr;

enum EffEvt_t {effevtProcessPix, effevtBlink};
EvtMsgQ_t<EffEvt_t, 3> EvtQEff;

static void ITmrCallback(void *p);

class Pixel_t {
private:
    uint32_t Delay = 0;
    ColorHSV_t CurrClr = hsvBlack, TargetClr = hsvBlack;
    bool DoBlink = false;
    uint32_t CurrV = BOTTOM_MIN_V;
    virtual_timer_t ITmr;
public:
    Color_t ToRGB() { return CurrClr.ToRGB(); }

    void Restart() {
        chVTReset(&ITmr);
        TargetClr = PCurrEff->GetNewClr();
        TargetClr.V = Random::Generate(TOP_MIN_V, TOP_MAX_V);
        CurrClr.H = TargetClr.H;
        uint32_t Delay = CurrClr.AdjustAndGetDelay(TargetClr, PCurrEff->SmoothValue);
        if(Delay == 0) Delay = 1;
        chVTSet(&ITmr, TIME_MS2I(1), ITmrCallback, this);
    }

    void IOnTmrI() {
        Delay = CurrClr.AdjustAndGetDelay(TargetClr, PCurrEff->SmoothValue);
        if(Delay == 0) {
            Delay = 1;
            // if on top brt, go down
            if(CurrClr.V >= TOP_MIN_V) {
//                TargetClr.V = Random::Generate(BOTTOM_MIN_V, BOTTOM_MAX_V);
                TargetClr.V = BOTTOM_MIN_V;
            }
            // if on bottom brt, change color and go up
            else {
                TargetClr = PCurrEff->GetNewClr();
                TargetClr.V = Random::Generate(TOP_MIN_V, TOP_MAX_V);
                CurrClr.H = TargetClr.H;
            }
        }
        chVTSetI(&ITmr, TIME_MS2I(Delay), ITmrCallback, this);
        EvtQEff.SendNowOrExitI(effevtProcessPix);
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
        EffEvt_t Evt = EvtQEff.Fetch(TIME_INFINITE);
        switch(Evt) {
            case effevtProcessPix:
                for(uint32_t i=0; i<LED_CNT; i++) RGBs[i] = Pixels[i].ToRGB();
                Leds::ShowPic(RGBs);
                break;

            case effevtBlink:

                break;
        } // switch
    } // while
}


namespace Effects {

void Init() {
    EvtQEff.Init();
    chThdCreateStatic(waEffThread, sizeof(waEffThread), NORMALPRIO, (tfunc_t)EffThread, NULL);
    for(auto &Pix : Pixels) Pix.Restart();
}

void Set(Effect_t &Eff) { PCurrEff = &Eff; }

void Blink(ColorHSV_t Clr) {
    BlinkClr = Clr;
    EvtQEff.SendNowOrExit(effevtBlink);
}

} // namespace
