/*
 * Effects.h
 *
 *  Created on: 28 нояб. 2020 г.
 *      Author: layst
 */

#pragma once

#include "color.h"

#define MAX_CLR_CNT     4
class Effect_t {
public:
    uint32_t ClrCnt = 0;
    ColorHSV_t ClrArr[MAX_CLR_CNT];
    uint32_t SmoothValue = 0;
    uint32_t ShowDuration_s = 2;

    void Set(uint32_t ASmoothValue, ColorHSV_t Clr0, uint32_t AShowDuration) {
        SmoothValue = ASmoothValue;
        ClrCnt = 1;
        ClrArr[0] = Clr0;
        ShowDuration_s = AShowDuration;
    }
    void Set(uint32_t ASmoothValue, ColorHSV_t Clr0, ColorHSV_t Clr1, uint32_t AShowDuration) {
        SmoothValue = ASmoothValue;
        ClrCnt = 2;
        ClrArr[0] = Clr0;
        ClrArr[1] = Clr1;
        ShowDuration_s = AShowDuration;
    }

    Effect_t() {}
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
} __attribute__((packed));

namespace Effects {
    void Init();
    void Set(Effect_t &Eff);
};
