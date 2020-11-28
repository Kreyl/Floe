/*
 * Effects.cpp
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#include "Effects.h"
#include "color.h"
#include "LEDs.h"

static uint32_t Start = 0, Delay = 180;
static Color_t RGBs[LED_CNT];

struct LedState_t {
    ColorHSV_t Hsv[LED_CNT];
    ColorHSV_t& operator[](int32_t Indx) { return Hsv[Indx]; }
    LedState_t& operator = (const LedState_t &Right) {
        Hsv[0] = Right.Hsv[0];
        Hsv[1] = Right.Hsv[1];
        Hsv[2] = Right.Hsv[2];
        Hsv[3] = Right.Hsv[3];
        Hsv[4] = Right.Hsv[4];
        Hsv[5] = Right.Hsv[5];
        Hsv[6] = Right.Hsv[6];
        return *this;
    }
    LedState_t() {
        Hsv[0] = hsvBlack;
        Hsv[1] = hsvBlack;
        Hsv[2] = hsvBlack;
        Hsv[3] = hsvBlack;
        Hsv[4] = hsvBlack;
        Hsv[5] = hsvBlack;
        Hsv[6] = hsvBlack;
    }
    LedState_t(ColorHSV_t c0, ColorHSV_t c1, ColorHSV_t c2, ColorHSV_t c3, ColorHSV_t c4, ColorHSV_t c5, ColorHSV_t c6) {
        Hsv[0] = c0;
        Hsv[1] = c1;
        Hsv[2] = c2;
        Hsv[3] = c3;
        Hsv[4] = c4;
        Hsv[5] = c5;
        Hsv[6] = c6;
    }

    // Returns delay
    uint32_t Adjust(LedState_t *Target, uint32_t Smooth) {
        uint32_t Delay = 0;
        for(uint32_t i=0; i<LED_CNT; i++) {
            uint32_t tmp = Hsv[i].AdjustAndGetDelay(Target->Hsv[i], Smooth);
            if(tmp > Delay) Delay = tmp;
        }
        return Delay;
    }

    void ToRGBs(Color_t *PDst) {
        for(uint32_t i=0; i<LED_CNT; i++) RGBs[i] = Hsv[i].ToRGB();
    }
};

static LedState_t CurrState { hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack };


#define MAX_CLR_CNT     7
class Effect_t {
private:
    uint32_t ClrCnt;
    ColorHSV_t ClrArr[MAX_CLR_CNT];

public:
    Effect_t(ColorHSV_t Clr0, ColorHSV_t Clr1) {
        ClrCnt = 2;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
        ConstructNewState();
    }
    Effect_t(ColorHSV_t Clr0, ColorHSV_t Clr1, ColorHSV_t Clr2, ColorHSV_t Clr3) {
        ClrCnt = 4;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
        ClrArr[2] = Clr2;
        ClrArr[3] = Clr3;
        ConstructNewState();
    }
    LedState_t State;
    void ConstructNewState() {
        for(uint32_t i=0; i<LED_CNT; i++) {
            uint32_t ClrIndx = Random::Generate(0, ClrCnt-1);
            State.Hsv[i] = ClrArr[ClrIndx];
        }
    }
};

Effect_t EffIdle{hsvBlack, hsvBlue, {240, 100, 60}, {240, 100, 30}};


Effect_t *CurrEff = &EffIdle;

namespace Effects {

void Init() { }

void Task() {
    if(Time.ElapsedSince(Start) < Delay) return;
    Start = Time.GetCurrent();
    // Get Target State
    LedState_t *TargetState = &CurrEff->State;
    // Adjust
    Delay = CurrState.Adjust(TargetState, 360);
//    Printf("%u: %u %u %u\r", Delay, CurrState[0].H, CurrState[0].S, CurrState[0].V);
    if(Delay == 0) {
        CurrEff->ConstructNewState();
    }
    // Show it
    CurrState.ToRGBs(RGBs);
    Leds::ShowPic(RGBs);
}

} // namespace
