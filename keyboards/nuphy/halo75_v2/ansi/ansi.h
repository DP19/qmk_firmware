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

    MAC_VOICE,
    MAC_DND,
    WIN_LOCK,

    DEV_RESET,
    SLEEP_MODE,
    BAT_SHOW,
    BAT_NUM,
    RGB_TEST,

    SIDE_VAI,
    SIDE_VAD,
    SIDE_MOD_A,
    SIDE_MOD_B,
    SIDE_HUI,
    SIDE_SPI,
    SIDE_SPD,

    DEB_I,
    DEB_D,
    DEB_SHOW,
    L_SLP_I,
    L_SLP_D,
    L_SLP_SHOW,
    D_SLP_I,
    D_SLP_D,
    D_SLP_SHOW,
    PWR_SHOW,
};

#define SYS_PRT                 G(S(KC_3))
#define MAC_PRTA                G(S(KC_4))
#define WIN_PRTA                G(S(KC_S))
#define MAC_SEARCH              G(KC_SPC)
#define MAC_LOCK                G(C(KC_Q))
