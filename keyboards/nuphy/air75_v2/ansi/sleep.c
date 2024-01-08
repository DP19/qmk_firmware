/*
Copyright 2023 @ Nuphy <https://nuphy.com/>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "user_kb.h"
#include "ansi.h"
#include "hal_usb.h"
#include "usb_main.h"
#include "mcu_pwr.h"

extern user_config_t   user_config;
extern DEV_INFO_STRUCT dev_info;
extern uint16_t        rf_linking_time;
extern uint16_t        rf_link_timeout;
extern uint16_t        no_act_time;
extern bool            f_goto_sleep;
extern bool            f_wakeup_prepare;

uint8_t uart_send_cmd(uint8_t cmd, uint8_t ack_cnt, uint8_t delayms);

void deep_sleep_handle(void) {
    enter_deep_sleep(); // puts the board in WFI mode and pauses the MCU
    exit_deep_sleep();  // This gets called when there is an interrupt (wake) event.

    // Clear the keyboard for new QMK life cycle to handle.
    clear_weak_mods();
    clear_mods();
    clear_keyboard();

    /* If RF is not connected anymore you would lose the first keystroke.
       This is expected behavior as the connection is not there.
       Very quick taps on the first key (the one that wakes the board) has a
       very high chance of getting stuck for some reason in the regular QMK lifecycle.
       So... just press and release normally if you don't want problems. Or spam some keys.
    */
}

/**
 * @brief  Sleep Handle.
 */
void sleep_handle(void) {
    static uint32_t delay_step_timer     = 0;
    static uint8_t  usb_suspend_debounce = 0;
    static uint32_t rf_disconnect_time   = 0;

    /* 50ms interval */
    if (timer_elapsed32(delay_step_timer) < 50) return;
    delay_step_timer = timer_read32();

    // sleep process;
    if (f_goto_sleep) {
        f_goto_sleep    = 0;
        bool deep_sleep = 0;
        if (user_config.sleep_enable) {
            deep_sleep = 1;
            // light sleep if charging? Charging event might keep waking MCU. To be confirmed...
            if (dev_info.rf_charge & 0x01) {
                deep_sleep = 0;
            }
            // or if it's in USB mode but USB state is suspended
            // TODO: How to detect if USB is unplugged? I only use RF so not a big deal I guess...
            else if (dev_info.link_mode == LINK_USB && USB_DRIVER.state == USB_SUSPENDED) {
                deep_sleep = 0;
            }
        }

        if (deep_sleep) {
            deep_sleep_handle();
            return; // don't need to do anything else
        } else {
            enter_light_sleep();
            f_wakeup_prepare = 1;
        }
    }

    // wakeup check
    // we only arrive here on light sleep.
    if (f_wakeup_prepare) {
        if (no_act_time < 10) { // activity wake up
            f_wakeup_prepare = 0;
            exit_light_sleep();
        }
        // No longer charging? Go deep sleep.
        // TODO: don't really know true charge bit logic. I'm just guessing here.
        else if ((dev_info.rf_charge & 0x01) == 0) {
            f_wakeup_prepare = 0;
            deep_sleep_handle();
            return;
        }
    }

    // sleep check, won't reach here on deep sleep.
    if (f_goto_sleep || f_wakeup_prepare) return;
    if (dev_info.link_mode == LINK_USB) {
        if (USB_DRIVER.state == USB_SUSPENDED) {
            usb_suspend_debounce++;
            if (usb_suspend_debounce >= 20) {
                f_goto_sleep = 1;
            }
        } else {
            usb_suspend_debounce = 0;
        }
    } else if (dev_info.rf_state == RF_CONNECT) {
        rf_disconnect_time = 0;
        if (no_act_time >= SLEEP_TIME_DELAY) {
            f_goto_sleep = 1;
        }
    } else if (rf_linking_time >= LINK_TIMEOUT) {
        rf_linking_time = 0;
        f_goto_sleep    = 1;
    } else if (dev_info.rf_state == RF_DISCONNECT) {
        rf_disconnect_time++;
        if (rf_disconnect_time > 5 * 20) {
            rf_disconnect_time = 0;
            f_goto_sleep       = 1;
        }
    }
}
