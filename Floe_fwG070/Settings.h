/*
 * Settings.h
 *
 *  Created on: 17 апр. 2020 г.
 *      Author: layst
 */

#pragma once

#include "SaveToFlash.h"
#include "Effects.h"
#include "kl_crc.h"

union Settings_t {
    uint64_t __Align;
    struct {
        uint32_t crc16 = 0;
        Effect_t EffIdle;
        Effect_t EffKnock;
        Effect_t EffWave;
        Effect_t EffPress;
    };

    void Load() {
        Flash::Read(FLASH_SETTINGS_ADDR, this, sizeof(Settings_t));
        // Check it
        Crc::InitHw();
        if(crc16 != Crc::CalculateCRC16HW((uint8_t*)this, sizeof(Settings_t))) {
            EffIdle .Set(306, hsvWhite, 2); // Duration is not used here
            EffKnock.Set(306, hsvGreen, 4);
            EffWave .Set(306, hsvBlue, 4);
            EffPress.Set(630, hsvRed, 4);
        }
    }
    uint8_t Save() {
        return Flash::Write(FLASH_SETTINGS_ADDR, this, sizeof(Settings_t));
    }
} __attribute__((packed));

extern Settings_t Settings;
