/*
 * EvtMsgIDs.h
 *
 *  Created on: 21 ���. 2017 �.
 *      Author: Kreyl
 */

#pragma once

enum EvtMsgId_t {
    evtIdNone = 0, // Always

    // Pretending to eternity
    evtIdShellCmd,
    evtIdEverySecond,
    evtIdAdcRslt,

    // Not eternal
    evtIdStateEnd,
    evtIdWave0,
    evtIdWave1,
    evtIdKnock,
    evtIdPress,

    evtIdTimeToSave,
    evtIdTimeToFlare,
};
