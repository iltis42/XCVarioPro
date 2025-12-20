/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <atomic>

class BlueTooth {
public:
    static BlueTooth& instance();
    void acquire();
    void release();

private:
    BlueTooth() = default;
    void init();
    void deinit();

    std::atomic<int> refCount{0};
};
