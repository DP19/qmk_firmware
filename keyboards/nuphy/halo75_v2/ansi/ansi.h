// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "quantum.h"

enum custom_keycodes {
    RF_DFU = QK_KB_0,
    LNK_USB,
    LNK_RF,
    LNK_BLE1,
    LNK_BLE2,
    LNK_BLE3,

    MAC_TASK,
    MAC_SEARCH,
    MAC_VOICE,
    MAC_CONSOLE,
    MAC_DND,
    MAC_PRT,
    MAC_PRTA,

    DEV_RESET,
    SLEEP_MODE,
    BAT_SHOW,
    BAT_NUM,

    SIDE_VAI,
    SIDE_VAD,
    SIDE_MOD_A,
    SIDE_MOD_B,
    SIDE_HUI,
    SIDE_SPI,
    SIDE_SPD,
};

#define SYS_PRT                 G(S(KC_3))
#define MAC_PRTA                G(S(KC_4))
#define WIN_PRTA                G(S(KC_S))
#define MAC_SEARCH              G(KC_SPC)
#define MAC_LOCK                G(C(KC_Q))
