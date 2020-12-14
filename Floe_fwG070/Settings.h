/*
 * Settings.h
 *
 *  Created on: 17 апр. 2020 г.
 *      Author: layst
 */

#pragma once

#include "SaveToFlash.h"
#include "kl_crc.h"

struct Settings_t {
    uint32_t crc16 = 0;
    uint32_t TypeID = 0;

    void Load() {
        Flash::Read(FLASH_SETTINGS_ADDR, this, sizeof(Settings_t));
        // Check it
        Crc::InitHw();
        uint32_t crcLoaded = crc16;
        crc16 = 0; // Reset it to not influence on calculated crc
        uint32_t crcCalculated = Crc::CalculateCRC16HW((uint8_t*)this, sizeof(Settings_t));
        Printf("%X %X\r", crcLoaded, crcCalculated);
        if(crcLoaded == crcCalculated) {
            Printf("Settings loaded\r");
        }
        else {
            TypeID = 0;
            Printf("Settings not set\r");
        }
    }
    uint8_t Save() {
        crc16 = 0; // Reset it to not influence on calculated crc
        uint32_t crcCalculated = Crc::CalculateCRC16HW((uint8_t*)this, sizeof(Settings_t));
        crc16 = crcCalculated;
        Printf("crc: %X\r", crc16);
        return Flash::Write(FLASH_SETTINGS_ADDR, this, sizeof(Settings_t));
    }
} __attribute__((packed, aligned(8)));

extern Settings_t Settings;
