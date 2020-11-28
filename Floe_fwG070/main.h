/*
 * main.h
 *
 *  Created on: 25 июн. 2020 г.
 *      Author: layst
 */

#pragma once

#include <inttypes.h>

struct AniFrame_t {
    uint8_t x0, y0, xsz, ysz;
    uint8_t Data[];
    AniFrame_t(uint8_t ax0, uint8_t ay0, uint8_t axsz, uint8_t aysz) :
        x0(ax0), y0(ay0), xsz(axsz), ysz(aysz) {}
} __attribute__((__packed__));
