/*
 * Settings.h
 *
 *  Created on: 17 апр. 2020 г.
 *      Author: layst
 */

#pragma once

#include "SaveToFlash.h"

#define DELAY_MULTIPLIER    4096

enum Mode_t : uint8_t { modeIdle, modeGradient, modeClock, modeSwitchBrt, modeSteadyTest, modeAnimation };

struct Settings_t {
    uint64_t __ReservedForBootSettings; // Do not use this, Bootloader keeps here it's settings
    Mode_t Mode;
    uint32_t CntMax;
    void Load() {
        Flash::Read(FLASH_SETTINGS_ADDR, this, sizeof(Settings_t));
        // Check it
        if(Mode == 0xFF) {
            Mode = modeSwitchBrt;
            CntMax = 720000;
        }
    }
    uint8_t Save() {
        return Flash::Write(FLASH_SETTINGS_ADDR, this, sizeof(Settings_t));
    }
} __attribute__((packed));

extern Settings_t Settings;
