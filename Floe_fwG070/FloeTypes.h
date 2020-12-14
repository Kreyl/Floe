/*
 * EffTypes.h
 *
 *  Created on: 6 дек. 2020 г.
 *      Author: layst
 */

#pragma once

#define TYPE_CNT            12

// Timings
#define DURATION_OF_WAVE_S    4
#define DURATION_OF_KNOCK_S   4
#define DURATION_OF_PRESS_S   18

// ==== Always ====
Effect_t EffPwrOn{306, hsvWhite};
Effect_t EffIdle {306, hsvBlue};
Effect_t EffGood {630, hsvGreen};
Effect_t EffBad  {180, hsvRed};

struct FloeEffs_t {
    Effect_t Wave, Knock, Press;
    const char *Description;
    FloeEffs_t(Effect_t AWave, Effect_t AKnock, Effect_t APress, const char* ADescription) :
        Wave(AWave), Knock(AKnock), Press(APress), Description(ADescription) { }
};

FloeEffs_t FloeTypes[] = {
        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood, "Random Good"}, // 0
        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood, "Random Bad" }, // 1
        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood, "Good 1" }, // 2
        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood, "Bad 1" }, // 3
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 4
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 5
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 6
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 7
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 8
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 9
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 10
//        { {306, {120, 100, 100}, {240, 100, 100}},  {90, {120, 100, 100}, {240, 100, 100}}, EffGood }, // 11
};
