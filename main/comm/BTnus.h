/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "InterfaceCtrl.h"

#include <cstdint>

class BlueTooth;

class BTnus final : public InterfaceCtrl
{
public:
    BTnus();
    virtual ~BTnus();
    bool start();
    void stop();
    inline bool selfTest() const { return isRunning(); }
	bool isRunning() const { return _server_running; }
	bool isConnected() const { return my_conn_id != 0xFFFF; }

    // Ctrl
    InterfaceId getId() const override { return BT_LE; }
    const char *getStringId() const override { return "BTle"; }
    void ConfigureIntf(int cfg) override {}
    int Send(const char *msg, int &len, int port = 0) override;

private:
	BlueTooth& core; // shared BT recourses

    // Receiving data
    friend class BTnus_EVENT_HANDLER;
    uint16_t my_conn_id = 0xFFFF;
    uint16_t service_handle = 0;
    uint16_t rx_char_handle = 0;
    uint16_t tx_char_handle = 0;
    uint16_t tx_cccd_handle = 0;
    uint16_t peer_mtu;
    bool nus_notify_enabled = false;

    bool _server_running = false;
};

extern BTnus *BLUEnus;
