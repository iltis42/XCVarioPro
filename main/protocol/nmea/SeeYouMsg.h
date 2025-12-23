/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "protocol/NMEA.h"

class SeeYouMsg final : public NmeaPlugin
{
public:
    using VAL_ITEM_IDX = enum {MC_VAL, BAL_VAL, BUGS_VAL, QNH_VAL, ELEVATION_VAL};
    SeeYouMsg(NmeaPrtcl &nr) : NmeaPlugin(nr, SEEYOU_P, false) {};
    virtual ~SeeYouMsg() = default;
    const ParserEntry* getPT() const override { return _pt; }

    // Declare send routines in NmeaPrtcl class !

private:
    // Received messages
    static dl_action_t gen_query(NmeaPlugin *plg);
    static const ParserEntry _pt[];
};
