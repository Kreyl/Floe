/*
 * EvtMsgIDs.h
 *
 *  Created on: 21 ���. 2017 �.
 *      Author: Kreyl
 */

#ifndef EVTMSGIDS_H__
#define EVTMSGIDS_H__

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

#endif //EVTMSGIDS_H__