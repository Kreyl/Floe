/*
 * EffTypes.h
 *
 *  Created on: 6 дек. 2020 г.
 *      Author: layst
 */

#ifndef FLOETYPES_H__
#define FLOETYPES_H__

#define TYPE_CNT            12

// Timings
#define DURATION_OF_WAVE_S  11
#define DURATION_OF_KNOCK_S 11
#define DURATION_OF_PRESS_S 18

#define SMOOTH_SLOW         540
#define SMOOTH_MID          90

// ==== Always ====
Effect_t EffPwrOn{306, hsvWhite};
Effect_t EffIdle {630, hsvBlue};
Effect_t EffGood {630, hsvGreen};
Effect_t EffBad  {180, hsvRed};

struct FloeEffs_t {
    Effect_t Knock, Wave, Press;
    const char *Description;
    FloeEffs_t(Effect_t AKnock, Effect_t AWave, Effect_t APress, const char* ADescription) :
        Knock(AKnock), Wave(AWave), Press(APress), Description(ADescription) { }
};

FloeEffs_t FloeTypes[] = {
        { {SMOOTH_MID,  hsvWhite               },  {SMOOTH_SLOW, hsvWhite              }, EffGood, "Random Good"}, // 0
        { {SMOOTH_MID,  hsvWhite               },  {SMOOTH_SLOW, hsvWhite              }, EffBad,  "Random Bad" }, // 1
        { {SMOOTH_MID,  hsvCyan                },  {SMOOTH_SLOW, hsvCyan,    hsvGreen  }, EffGood, "Good 1"     }, // 2
        { {SMOOTH_SLOW, hsvMagenta             },  {SMOOTH_MID,  hsvMagenta, hsvBlue   }, EffBad,  "Bad 1"      }, // 3

        { {SMOOTH_MID,  hsvCyan,    hsvBlue    },  {SMOOTH_MID,  hsvCyan,    hsvGreen  }, EffGood, "Good 2"     }, // 4
        { {SMOOTH_SLOW, hsvCyan                },  {SMOOTH_SLOW, hsvMagenta            }, EffGood, "Good 3"     }, // 5
        { {SMOOTH_SLOW, hsvCyan,    hsvBlue    },  {SMOOTH_MID,  hsvCyan,    hsvMagenta}, EffGood, "Good 4"     }, // 6
        { {SMOOTH_SLOW, hsvMagenta, hsvBlue    },  {SMOOTH_SLOW, hsvCyan               }, EffGood, "Good 5"     }, // 7

        { {SMOOTH_SLOW, hsvMagenta, hsvGreen   },  {SMOOTH_SLOW, hsvCyan,    hsvMagenta}, EffBad,  "Bad 2"      }, // 8
        { {SMOOTH_MID,  hsvMagenta             },  {SMOOTH_SLOW, hsvCyan,    hsvBlue   }, EffBad,  "Bad 3"      }, // 9
        { {SMOOTH_MID,  hsvMagenta, hsvBlue    },  {SMOOTH_MID,  hsvCyan               }, EffBad,  "Bad 4"      }, // 10
        { {SMOOTH_MID,  hsvMagenta, hsvGreen   },  {SMOOTH_MID,  hsvCyan,    hsvBlue   }, EffBad,  "Bad 5"      }, // 11
};

#endif //FLOETYPES_H__