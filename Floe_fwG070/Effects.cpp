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

LedState_t Seq1[] = {
        { hsvRed, hsvGreen, hsvBlue, hsvYellow, hsvMagenta, hsvCyan, hsvWhite },
        { hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack },
        { hsvGreen, hsvGreen, hsvGreen, hsvGreen, hsvGreen, hsvGreen, hsvGreen },
        { hsvGreen, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack },
        { hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack, hsvBlack },
};

static LedState_t *CurrSeq = Seq1;
static uint32_t CurrStep = 0;

namespace Effects {

void Init() { }

void Task() {
    if(Time.ElapsedSince(Start) < Delay) return;
    Start = Time.GetCurrent();
    // Get Target State
    uint32_t TargetStep = CurrStep + 1;
    if(TargetStep >= countof(Seq1)) TargetStep = 0; // XXX
    LedState_t *TargetState = &CurrSeq[TargetStep];
    // Adjust
    Delay = CurrState.Adjust(TargetState, 360);
//    Printf("%u: %u %u %u\r", Delay, CurrState[0].H, CurrState[0].S, CurrState[0].V);
    if(Delay == 0) {
        CurrStep = TargetStep;
        Delay = 99;
    }
    // Show it
    CurrState.ToRGBs(RGBs);
    Leds::ShowPic(RGBs);
}

} // namespace
