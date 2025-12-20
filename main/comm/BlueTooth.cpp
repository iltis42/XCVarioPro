/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "BlueTooth.h"
#include "setup/SetupCommon.h"
#include "logdefnone.h"

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>


BlueTooth& BlueTooth::instance()
{
    static BlueTooth inst;
    return inst;
}

void BlueTooth::acquire()
{
    if (++refCount == 1) {
        init();
    }
}

void BlueTooth::release()
{
    if (--refCount == 0) {
        deinit();
    }
}

void BlueTooth::init()
{
    ESP_LOGI(FNAME,"BT init()" );

    // Initialize BlueTooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(FNAME, "BlueTooth controller initialize failed");
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(FNAME, "BlueTooth controller enable failed");
        return;
    }

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(FNAME, "Bluedroid stack initialize failed");
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(FNAME, "Bluedroid stack enable failed");
        return;
    }

    // Set the BlueTooth device name
    ret = esp_bt_gap_set_device_name(SetupCommon::getID());
    if (ret != ESP_OK)
    {
        ESP_LOGE(FNAME, "Failed to set device name: %s", esp_err_to_name(ret));
    }
}


void BlueTooth::deinit()
{
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}