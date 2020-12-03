/*
 * Effects.cpp
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#include "Effects.h"
#include "color.h"
#include "LEDs.h"

#define BOTTOM_MIN_V    4
#define BOTTOM_MAX_V    18
#define TOP_MIN_V       45
#define TOP_MAX_V       100

static Color_t RGBs[LED_CNT];

#if 1 // ============================ Effect ===================================
#define MAX_CLR_CNT     7
class Effect_t {
private:
    uint32_t ClrCnt;
    ColorHSV_t ClrArr[MAX_CLR_CNT];
public:
    uint32_t SmoothValue;
    Effect_t(uint32_t SmoothValue, ColorHSV_t Clr0) : SmoothValue(SmoothValue) {
        ClrCnt = 1;
        ClrArr[0] = Clr0;
    }
    Effect_t(uint32_t SmoothValue, ColorHSV_t Clr0, ColorHSV_t Clr1) : SmoothValue(SmoothValue) {
        ClrCnt = 2;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
    }
    Effect_t(uint32_t SmoothValue, ColorHSV_t Clr0, ColorHSV_t Clr1, ColorHSV_t Clr2) : SmoothValue(SmoothValue) {
        ClrCnt = 3;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
        ClrArr[2] = Clr2;
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
#endif

//Effect_t EffIdle{306, {240, 100, 100}, {300, 100, 0}};
Effect_t EffIdle{306, {120, 100, 100}, {240, 100, 100}};
//Effect_t EffIdle{306, {240, 100, 100}};

Effect_t *CurrEff = &EffIdle;

class Pixel_t {
private:
    uint32_t tStart = 0, Delay = 0;
    ColorHSV_t CurrClr = hsvBlack, TargetClr = hsvBlack;
    uint32_t CurrV = BOTTOM_MIN_V;
public:
    void Update() {
//        if(Time.ElapsedSince(tStart) < Delay) return;
//        tStart = Time.GetCurrent();
//        Delay = CurrClr.AdjustAndGetDelay(TargetClr, CurrEff->SmoothValue);
//        if(Delay == 0) {
//            // if on top brt, go down
//            if(CurrClr.V >= TOP_MIN_V) {
////                TargetClr.V = Random::Generate(BOTTOM_MIN_V, BOTTOM_MAX_V);
//                TargetClr.V = BOTTOM_MIN_V;
//            }
//            // if on bottom brt, change color and go up
//            else {
//                TargetClr = CurrEff->GetNewClr();
//                TargetClr.V = Random::Generate(TOP_MIN_V, TOP_MAX_V);
//                CurrClr.H = TargetClr.H;
//            }
//        }
    }
    Color_t ToRGB() { return CurrClr.ToRGB(); }
};

Pixel_t Pixels[LED_CNT];

void ToRGBs(Color_t *PDst) {
    for(uint32_t i=0; i<LED_CNT; i++) RGBs[i] = Pixels[i].ToRGB();
}

namespace Effects {

void Task() {
    for(auto &Pix : Pixels) Pix.Update();
    // Show it
    ToRGBs(RGBs);
    Leds::ShowPic(RGBs);
}

} // namespace
