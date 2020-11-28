/*
 * Effects.cpp
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#include "Effects.h"
#include "color.h"
#include "LEDs.h"

static Color_t RGBs[LED_CNT];

#define MAX_CLR_CNT     7
class Effect_t {
private:
    uint32_t ClrCnt;
    ColorHSV_t ClrArr[MAX_CLR_CNT];
public:
    uint32_t SmoothValue;
    Effect_t(uint32_t SmoothValue, ColorHSV_t Clr0, ColorHSV_t Clr1) : SmoothValue(SmoothValue) {
        ClrCnt = 2;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
    }
    Effect_t(uint32_t SmoothValue, ColorHSV_t Clr0, ColorHSV_t Clr1, ColorHSV_t Clr2, ColorHSV_t Clr3) : SmoothValue(SmoothValue) {
        ClrCnt = 4;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
        ClrArr[2] = Clr2;
        ClrArr[3] = Clr3;
    }

    ColorHSV_t GetNewClr() {
        uint32_t ClrIndx = Random::Generate(0, ClrCnt-1);
        return ClrArr[ClrIndx];
    }
};

Effect_t EffIdle{306, {240, 100, 100}, {240, 100, 60}, {240, 100, 30}, {240, 100, 4}};


Effect_t *CurrEff = &EffIdle;

class Pixel_t {
private:
    uint32_t tStart = 0, Delay = 0;
    ColorHSV_t CurrClr = hsvBlack, TargetClr = hsvBlack;
public:
    void Adjust() {
        if(Time.ElapsedSince(tStart) < Delay) return;
        tStart = Time.GetCurrent();
        Delay = CurrClr.AdjustAndGetDelay(TargetClr, CurrEff->SmoothValue);
        if(Delay == 0) TargetClr = CurrEff->GetNewClr();
    }
    Color_t ToRGB() { return CurrClr.ToRGB(); }
};

Pixel_t Pixels[LED_CNT];

void ToRGBs(Color_t *PDst) {
    for(uint32_t i=0; i<LED_CNT; i++) RGBs[i] = Pixels[i].ToRGB();
}

namespace Effects {

void Init() { }

void Task() {
    for(auto &Pix : Pixels) Pix.Adjust();
    // Show it
    ToRGBs(RGBs);
    Leds::ShowPic(RGBs);
}

} // namespace
