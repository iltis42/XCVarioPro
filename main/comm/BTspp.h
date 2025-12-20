/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "InterfaceCtrl.h"

#include <esp_spp_api.h>

class BlueTooth;

class BTspp final : public InterfaceCtrl
{
public:
	BTspp();
	virtual ~BTspp();

	bool start();
	void stop();
	inline bool selfTest() const { return isRunning(); }
	bool isRunning() const { return _server_running; }
	bool isConnected() const { return _client_handle != 0; }

	// Ctrl
	InterfaceId getId() const override { return BT_SPP; }
	const char *getStringId() const override { return "BTspp"; }
	void ConfigureIntf(int cfg) override {}
	int Send(const char *msg, int &len, int port = 0) override;

private:
	BlueTooth& core; // shared BT recourses

	// Receiving data
	friend class BTspp_EVENT_HANDLER;
	uint32_t _client_handle = 0;
	bool _server_running = false;
};

extern BTspp *BLUEspp;

