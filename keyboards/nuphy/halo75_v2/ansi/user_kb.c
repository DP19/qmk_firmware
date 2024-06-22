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
#include "config.h"
#include "usb_main.h"
#include "mcu_pwr.h"
#include "rf_driver.h"

user_config_t user_config;
DEV_INFO_STRUCT dev_info = {
    .rf_battery = 100,
    .link_mode  = LINK_USB,
    .rf_state   = RF_IDLE,
};
bool f_bat_hold         = 0;
bool f_sys_show         = 0;
bool f_sleep_show       = 0;
bool f_send_channel     = 0;
bool f_dial_sw_init_ok  = 0;
bool f_rf_sw_press      = 0;
bool f_dev_reset_press  = 0;
bool f_rf_dfu_press     = 0;
bool f_rgb_test_press   = 0;
bool f_bat_num_show     = 0;

uint8_t       rf_blink_cnt          = 0;
uint8_t       rf_sw_temp            = 0;
uint8_t       host_mode;
uint16_t      rf_linking_time       = 0;
uint16_t      rf_link_show_time     = 0;
uint16_t      no_act_time           = 0;
uint16_t      dev_reset_press_delay = 0;
uint16_t      rf_dfu_press_delay    = 0;
uint16_t      rf_sw_press_delay     = 0;
uint16_t      rgb_test_press_delay  = 0;
uint16_t      rgb_led_last_act      = 0;
host_driver_t *m_host_driver        = 0;

extern bool               f_rf_new_adv_ok;
extern report_keyboard_t *keyboard_report;
#ifdef NKRO_ENABLE
    extern report_nkro_t *nkro_report;
    extern uint8_t bitkb_report_buf[32];
#endif // NKRO_ENABLE
extern uint8_t bytekb_report_buf[8];

extern bool f_goto_sleep;

/**
 * @brief  gpio initial.
 */
void gpio_init(void) {
    gpio_set_pin_output(DC_BOOST_PIN);
    gpio_write_pin_high(DC_BOOST_PIN);

    // Initializes the RGB Driver SDB pin
    gpio_set_pin_output(RGB_DRIVER_SDB1);
    gpio_write_pin_high(RGB_DRIVER_SDB1);
    gpio_set_pin_output(RGB_DRIVER_SDB2);
    gpio_write_pin_high(RGB_DRIVER_SDB2);

    // RF wake up pin configuration
    gpio_set_pin_output(NRF_WAKEUP_PIN);
    gpio_write_pin_high(NRF_WAKEUP_PIN);

    // RFboot Control pin
    gpio_set_pin_input_high(NRF_TEST_PIN);

    // RF reset pin configuration
    gpio_set_pin_output(NRF_RESET_PIN);
    gpio_write_pin_low(NRF_RESET_PIN);
    wait_ms(50);
    gpio_write_pin_high(NRF_RESET_PIN);

    // Switch detection pin
    gpio_set_pin_input_high(DEV_MODE_PIN);
    gpio_set_pin_input_high(SYS_MODE_PIN);
}

/**
 * @brief  long press key process.
 */
void long_press_key(void)
{
    static uint32_t long_press_timer = 0;

    if (timer_elapsed32(long_press_timer) < 100) return;
    long_press_timer = timer_read32();

    if (f_rf_sw_press) {
        rf_sw_press_delay++;
        if (rf_sw_press_delay >= RF_LONG_PRESS_DELAY) {
            f_rf_sw_press        = 0;
            dev_info.link_mode   = rf_sw_temp;
            dev_info.rf_channel  = rf_sw_temp;
            dev_info.ble_channel = rf_sw_temp;

            uint8_t timeout = 5;
            while (timeout--) {
                uart_send_cmd(CMD_NEW_ADV, 0, 1);
                wait_ms(20);
                uart_receive_pro();
                if (f_rf_new_adv_ok) break;
            }
        }
    } else {
        rf_sw_press_delay = 0;
    }

    if (f_dev_reset_press) {
        dev_reset_press_delay++;
        if (dev_reset_press_delay >= DEV_RESET_PRESS_DELAY)  {
            f_dev_reset_press = 0;

            if (dev_info.link_mode != LINK_USB) {
                if (dev_info.link_mode != LINK_RF_24) {
                    dev_info.link_mode      = LINK_BT_1;
                    dev_info.ble_channel    = LINK_BT_1;
                    dev_info.rf_channel     = LINK_BT_1;
                }
            } else {
                dev_info.ble_channel = LINK_BT_1;
                dev_info.rf_channel  = LINK_BT_1;
            }

            uart_send_cmd(CMD_SET_LINK, 10, 10);
            wait_ms(500);
            uart_send_cmd(CMD_CLR_DEVICE, 10, 10);

            void device_reset_show(void);
            void device_reset_init(void);

            eeconfig_init();
            device_reset_show();
            device_reset_init();


            if (dev_info.sys_sw_state == SYS_SW_MAC) {
                default_layer_set(1 << 0);  // MAC
                #ifdef NKRO_ENABLE
                    keymap_config.nkro = 0;
                #endif // NKRO_ENABLE
            } else {
                default_layer_set(1 << 2);  // WIN
                #ifdef NKRO_ENABLE
                    keymap_config.nkro = 1;
                #endif // NKRO_ENABLE
            }
        }
    } else {
        dev_reset_press_delay = 0;
    }

    if (f_rf_dfu_press) {
        rf_dfu_press_delay++;
        if (rf_dfu_press_delay >= RF_DFU_PRESS_DELAY) {
            f_rf_dfu_press = 0;
            uart_send_cmd(CMD_RF_DFU, 10, 20);
        }
    } else {
        rf_dfu_press_delay = 0;
    }

    if (f_rgb_test_press) {
        rgb_test_press_delay++;
        if (rgb_test_press_delay >= RGB_TEST_PRESS_DELAY) {
            f_rgb_test_press = 0;
            rgb_test_show();
        }
    } else {
        rgb_test_press_delay = 0;
    }
}

/**
 * @brief  Release all keys, clear keyboard report.
 */
void break_all_key(void) {
    uint8_t report_buf[NKRO_REPORT_BITS + 1];
    bool nkro_temp = keymap_config.nkro;

    clear_weak_mods();
    clear_mods();
    clear_keyboard();

    #ifdef NKRO_ENABLE
        keymap_config.nkro = 1;
        memset(nkro_report, 0, sizeof(report_nkro_t));
        host_nkro_send(nkro_report);
        wait_ms(10);
    #endif // NRKO_ENABLE

    keymap_config.nkro = 0;
    memset(keyboard_report, 0, sizeof(report_keyboard_t));
    host_keyboard_send(keyboard_report);
    wait_ms(10);

    keymap_config.nkro = nkro_temp;

    if (dev_info.link_mode != LINK_USB) {
        #ifdef NKRO_ENABLE
            memset(report_buf, 0, 16);
            uart_send_report(CMD_RPT_BIT_KB, report_buf, 16);
            wait_ms(10);
        #endif // NKRO_ENABLE
        uart_send_report(CMD_RPT_BYTE_KB, report_buf, 8);
        wait_ms(10);
    }

    #ifdef NKRO_ENABLE
        memset(bitkb_report_buf, 0, sizeof(bitkb_report_buf));
    #endif // NKRO_ENABLE
    memset(bytekb_report_buf, 0, sizeof(bytekb_report_buf));
}

/**
 * @brief  switch device link mode.
 * @param mode : link mode
 */
static void switch_dev_link(uint8_t mode) {
    if (mode > LINK_USB) return;
    break_all_key();

    dev_info.link_mode = mode;
    dev_info.rf_state = RF_IDLE;
    f_send_channel    = 1;

    if (mode == LINK_USB) {
        host_mode = HOST_USB_TYPE;
        host_set_driver(m_host_driver);
        rf_link_show_time = 0;
    }
    else {
        host_mode = HOST_RF_TYPE;
        host_set_driver(&rf_host_driver);
    }
}

/**
 * @brief  scan dial switch.
 */
void dial_sw_scan(void)
{
    uint8_t dial_scan               = 0;
    static uint8_t dial_save        = 0xf0;
    static uint8_t debounce         = 0;
    static uint32_t dial_scan_timer = 0;
    static bool flag_power_on       = true;

    if (!flag_power_on) {
        if (timer_elapsed32(dial_scan_timer) < 20) return;
    }
    dial_scan_timer = timer_read32();

    gpio_set_pin_input_high(DEV_MODE_PIN);
    gpio_set_pin_input_high(SYS_MODE_PIN);

    if (gpio_read_pin(DEV_MODE_PIN)) dial_scan |= 0X01;
    if (gpio_read_pin(SYS_MODE_PIN)) dial_scan |= 0X02;

    if (dial_save != dial_scan) {
        break_all_key();
        no_act_time         = 0;
        rf_linking_time     = 0;

        dial_save         = dial_scan;
        debounce            = 25;
        f_dial_sw_init_ok   = 0;
        return;
    } else if (debounce) {
        debounce--;
        return;
    }

    if (dial_scan & 0x01) {
        if (dev_info.link_mode != LINK_USB) {
            switch_dev_link(LINK_USB);
        }
    } else {
        if (dev_info.link_mode != dev_info.rf_channel) {
            switch_dev_link(dev_info.rf_channel);
        }
    }

    if (dial_scan & 0x02) {
        if (dev_info.sys_sw_state != SYS_SW_WIN) {
            f_sys_show = 1;
            default_layer_set(1 << 2);
            dev_info.sys_sw_state = SYS_SW_WIN;
            #ifdef NKRO_ENABLE
                keymap_config.nkro = 1;
            #endif // NKRO_ENABLE
            break_all_key();
        }
    } else {
        if (dev_info.sys_sw_state != SYS_SW_MAC) {
            f_sys_show = 1;
            default_layer_set(1 << 0);
            dev_info.sys_sw_state = SYS_SW_MAC;
            keymap_config.nkro    = 0;
            break_all_key();
        }
    }

    if (f_dial_sw_init_ok == 0) {
        f_dial_sw_init_ok = 1;
        flag_power_on     = false;

        if (dev_info.link_mode != LINK_USB) {
            host_set_driver(&rf_host_driver);
        }
    }
}

/**
 * @brief  power on scan dial switch.
 */
 //dial_sw_fast_scan
void dial_sw_fast_scan(void) {

    uint8_t dial_scan_dev = 0;
    uint8_t dial_scan_sys = 0;
    uint8_t dial_check_dev = 0;
    uint8_t dial_check_sys = 0;
    uint8_t debounce = 0;

    gpio_set_pin_input_high(DEV_MODE_PIN);
    gpio_set_pin_input_high(SYS_MODE_PIN);

    for(debounce=0; debounce<10; debounce++) {
        dial_scan_dev = 0;
        dial_scan_sys = 0;
        if (readPin(DEV_MODE_PIN)) {
            dial_scan_dev = 0x01;
        } else {
            dial_scan_dev = 0;
        }

        if (readPin(SYS_MODE_PIN)) {
            dial_scan_sys = 0x01;
        } else {
            dial_scan_sys = 0;
        }

        if ((dial_scan_dev != dial_check_dev)||(dial_scan_sys != dial_check_sys)) {
            dial_check_dev = dial_scan_dev;
            dial_check_sys = dial_scan_sys;
            debounce = 0;
        }
        wait_ms(1);
    }

    if (dial_scan_dev) {
        if (dev_info.link_mode != LINK_USB) {
            switch_dev_link(LINK_USB);
        }
    } else {
        if (dev_info.link_mode != dev_info.rf_channel) {
            switch_dev_link(dev_info.rf_channel);
        }
    }
    // Win or Mac
    if (dial_scan_sys) {
        if (dev_info.sys_sw_state != SYS_SW_WIN) {
            break_all_key();
            default_layer_set(1 << 2);
            dev_info.sys_sw_state = SYS_SW_WIN;
            #ifdef NKRO_ENABLE
                keymap_config.nkro = 1;
            #endif // NKRO_ENABLE
        }
    } else {
        if (dev_info.sys_sw_state != SYS_SW_MAC) {
            break_all_key();
            default_layer_set(1 << 0);
            dev_info.sys_sw_state = SYS_SW_MAC;
            keymap_config.nkro    = 0;
        }
    }
}



/**
    @brief  timer process.
 */
void timer_pro(void)
{
    static uint32_t interval_timer = 0;
    static bool f_first            = true;

    if (f_first) {
        if (host_get_driver()) {
            f_first        = false;
            interval_timer = timer_read32();
            m_host_driver  = host_get_driver();
        }
    }

    // step 10ms
    if (timer_elapsed32(interval_timer) < 10) return;
    interval_timer = timer_read32();

    if (rf_link_show_time < RF_LINK_SHOW_TIME) rf_link_show_time++;

    if (no_act_time < 0xffff) no_act_time++;

    if (rf_linking_time < 0xffff) rf_linking_time++;

    if (rgb_led_last_act < 0xffff) rgb_led_last_act++;

}

/**
 * @brief Wrapper for rgb_matrix_set_color for sleep.c logic usage.
 */
void user_set_rgb_color(int index, uint8_t red, uint8_t green, uint8_t blue) {
    if (red || green || blue) {
        rgb_led_last_act = 0;
        pwr_rgb_led_on(); // turn on LEDs
    }
    rgb_matrix_set_color(index, red, green, blue);
}

/**
 * @brief Handle LED power
 * @note Turn off LEDs if not used to save some power. This is ported
 *       from older Nuphy leaks.
 */
void led_power_handle(void) {
    // sleep handler is setting values we shouldn't check
    if (f_goto_sleep) return;

    static uint32_t interval = 0;

    if (timer_elapsed32(interval) < 500) // only check once in a while, less flickering for unhandled cases
        return;

    interval = timer_read32();

    if (rgb_led_last_act > 100) { // 10ms intervals
        if (rgb_matrix_is_enabled() && rgb_matrix_get_val() != 0) {
            pwr_rgb_led_on();
        } else { // brightness is 0 or RGB off.
            pwr_rgb_led_off();
        }
    }
}

void eeconfig_init_kb_datablock(void) {
    // eeprom has been reset by user or flash
    user_config.side_mode_a  = DEFAULT_SIDE_MODE_A;  /*SIDE_OFF*/
    user_config.side_mode_b  = DEFAULT_SIDE_MODE_B;
    user_config.side_light   = DEFAULT_SIDE_LIGHT; /*LIGHT_OFF*/
    user_config.side_speed   = DEFAULT_SIDE_SPEED;
    user_config.side_rgb     = DEFULAT_SIDE_RGB;
    user_config.side_colour  = DEFAULT_SIDE_COLOUR;
    user_config.sleep_enable = true;
    eeconfig_update_kb_datablock(&user_config);
}
