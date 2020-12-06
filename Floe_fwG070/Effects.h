/*
 * Effects.h
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#pragma once

#include "color.h"

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

namespace Effects {
    void Init();
    void Set(Effect_t &Eff);
};
