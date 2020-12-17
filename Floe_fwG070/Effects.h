/*
 * Effects.h
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#pragma once

#include "color.h"

#define MAX_CLR_CNT      2
class Effect_t {
public:
    uint32_t ClrCnt = 0;
    ColorHSV_t ClrArr[MAX_CLR_CNT];
    uint32_t SmoothValue = 0;

    void Set(uint32_t ASmoothValue, ColorHSV_t Clr0) {
        SmoothValue = ASmoothValue;
        ClrCnt = 1;
        ClrArr[0] = Clr0;
    }
    void Set(uint32_t ASmoothValue, ColorHSV_t Clr0, ColorHSV_t Clr1) {
        SmoothValue = ASmoothValue;
        ClrCnt = 2;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
    }

    Effect_t() {}
    Effect_t(uint32_t SmoothValue, ColorHSV_t Clr0) :
        SmoothValue(SmoothValue) {
        ClrCnt = 1;
        ClrArr[0] = Clr0;
    }
    Effect_t(uint32_t SmoothValue, ColorHSV_t Clr0, ColorHSV_t Clr1) : SmoothValue(SmoothValue) {
        ClrCnt = 2;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
    }

    ColorHSV_t GetNewClr() {
        uint32_t ClrIndx = Random::Generate(0, ClrCnt-1);
        return ClrArr[ClrIndx];
    }
} __attribute__((packed));

namespace Effects {
    void Init();
    void Set(Effect_t &Eff);
    void Blink(ColorHSV_t Clr);
};
