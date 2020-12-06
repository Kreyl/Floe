/*
 * EffTypes.h
 *
 *  Created on: 6 дек. 2020 г.
 *      Author: layst
 */

#pragma once

//#define IS_GOOD     TRUE
#define TYPE            1

// Timings
#define DURATION_OF_WAVE    4500
#define DURATION_OF_KNOCK   4500
#define DURATION_OF_PRESS   18000

// ==== Always ====
Effect_t EffIdle{306, {240, 100, 100}};

#if IS_GOOD
Effect_t EffOnPress{630, {120, 100, 100}};

#if TYPE == 1
Effect_t EffWave {306, {120, 100, 100}, {240, 100, 100}};
Effect_t EffKnock{90, {120, 100, 100}, {240, 100, 100}};
#elif TYPE == 2

#endif

#else
Effect_t EffOnPress {180, {0, 100, 100}};

#if TYPE == 1
Effect_t EffWave {306, {120, 100, 100}, {240, 100, 100}};
Effect_t EffKnock{90, {120, 100, 100}, {240, 100, 100}};
#elif TYPE == 2

#endif

#endif







