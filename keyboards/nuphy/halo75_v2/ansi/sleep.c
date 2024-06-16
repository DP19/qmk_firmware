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
extern uint16_t        no_act_time;
extern bool            f_goto_sleep;
extern bool            f_force_deep;
extern bool            f_wakeup_prepare;

void set_left_rgb(uint8_t r, uint8_t g, uint8_t b);

void signal_sleep(uint8_t r, uint8_t g, uint8_t b) {
    // Visual cue for sleep/wake on side LED.
    pwr_side_led_on();
    wait_ms(50); // give some time to ensure LED powers on.
    set_left_rgb(r, g, b);
    // side_rgb_refresh();
    wait_ms(300);
}

void deep_sleep_handle(void) {
    // flash red when deep sleep is about to happen
    signal_sleep(0x99, 0x00, 0x00);

    // Sync again before sleeping
    dev_sts_sync();

    enter_deep_sleep(); // puts the board in WFI mode and pauses the MCU
    exit_deep_sleep();  // This gets called when there is an interrupt (wake) event.
    f_goto_sleep = 0;

    // flash white on wake up
    signal_sleep(0x99, 0x99, 0x99);
    /* If RF is not connected anymore you would lose the first keystroke.
       This is expected behavior as the connection is not there.
    */
    no_act_time = 0; // required to not cause an immediate sleep on first wake
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

    // sleep process
    if (f_goto_sleep) {
        bool deep_sleep = 0;
        f_goto_sleep    = 0;

        if (user_config.sleep_enable) {
            deep_sleep = 1;
            if (f_force_deep) {
            } else if (no_act_time < SLEEP_TIME_DELAY) {
                deep_sleep = 0;
            } else if (dev_info.link_mode == LINK_USB && USB_DRIVER.state == USB_SUSPENDED) {
                deep_sleep = 0;
            }

            // if (dev_info.rf_state == RF_CONNECT)
            //     uart_send_cmd(CMD_SET_CONFIG, 5, 5);
            // else
            //     uart_send_cmd(CMD_SLEEP, 5, 5);

            // // power off led
            // gpio_write_pin_low(DC_BOOST_PIN);
            // gpio_write_pin_low(RGB_DRIVER_SDB1);
            // gpio_write_pin_low(RGB_DRIVER_SDB2);
            //
        }

        if (deep_sleep) {
            f_force_deep = 0;
            deep_sleep_handle();
            return;
        } else {
            enter_light_sleep();
            f_wakeup_prepare = 1;
        }
    }

    // wakeup check
    if (f_wakeup_prepare) {
        if (no_act_time < 10) {
            f_wakeup_prepare = 0;
            f_goto_sleep     = 0;
            exit_light_sleep();

            // gpio_write_pin_high(DC_BOOST_PIN);
            //         gpio_write_pin_high(RGB_DRIVER_SDB1);
            //         gpio_write_pin_high(RGB_DRIVER_SDB2);

            //         uart_send_cmd(CMD_HAND, 0, 1);

            //         if (dev_info.link_mode == LINK_USB) {
            //             usb_lld_wakeup_host(&USB_DRIVER);
            //             restart_usb_driver(&USB_DRIVER);
            //         }
        }
    }

    if (f_wakeup_prepare) return;

    if (dev_info.link_mode == LINK_USB) {
        if (USB_DRIVER.state == USB_SUSPENDED) {
            usb_suspend_debounce++;
            if (usb_suspend_debounce >= 20 && (dev_info.rf_charge != 0x03)) {
                f_goto_sleep = 1;
                f_force_deep = 1;
            }
        } else {
            usb_suspend_debounce = 0;
        }
    } else if (dev_info.rf_state == RF_CONNECT) {
        rf_disconnect_time = 0;
        if (no_act_time >= LIGHT_SLEEP_DELAY) {
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
            f_force_deep       = 1;
        }
    }
}
