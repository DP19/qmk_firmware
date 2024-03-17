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
#include "side_table.h"
#include "ws2812.h"
#include "mcu_pwr.h"

#define SIDE_BRIGHT_MAX 4
#define SIDE_SPEED_MAX 4
#define SIDE_COLOUR_MAX 8

#define SIDE_LINE 6
#define SIDE_LED_NUM 12

#define RF_LED_LINK_PERIOD 500
#define RF_LED_PAIR_PERIOD 250

/* side rgb mode */
enum {
    SIDE_WAVE = 0,
    SIDE_MIX,
    SIDE_STATIC,
    SIDE_BREATH,
    SIDE_OFF,
};

bool      flush_side_leds = false;
uint8_t   side_play_point = 0;
uint8_t   r_temp, g_temp, b_temp;
rgb_led_t side_leds[SIDE_LED_NUM] = {0};

const uint8_t side_speed_table[5][5] = {
    [SIDE_WAVE]   = {10, 14, 20, 28, 38}, //
    [SIDE_MIX]    = {10, 14, 20, 28, 38}, //
    [SIDE_STATIC] = {50, 50, 50, 50, 50}, //
    [SIDE_BREATH] = {10, 14, 20, 28, 38}, //
    [SIDE_OFF]    = {50, 50, 50, 50, 50}, //
};

const uint8_t side_light_table[6] = {
    0,   //
    22,  //
    34,  //
    55,  //
    79,  //
    106, //
};

const uint8_t side_led_index_tab[SIDE_LINE][2] = {
    {5, 6},  //
    {4, 7},  //
    {3, 8},  //
    {2, 9},  //
    {1, 10}, //
    {0, 11}, //
};

extern DEV_INFO_STRUCT dev_info;
extern user_config_t   user_config;
extern uint16_t        rf_link_show_time;
extern bool            f_bat_hold;
extern bool            f_sys_show;
extern bool            f_sleep_show;
extern bool            f_goto_sleep;

void side_ws2812_setleds(rgb_led_t *ledarray, uint16_t leds);
void rgb_matrix_update_pwm_buffers(void);

bool is_side_rgb_off(void) {
    for (int i = 0; i < SIDE_LED_NUM; i++) {
        if ((side_leds[i].r != 0) || (side_leds[i].g != 0) || (side_leds[i].b != 0)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief  side leds set color vaule.
 * @param  i: index of side_leds[].
 * @param  ...
 */
void side_rgb_set_color(int i, uint8_t r, uint8_t g, uint8_t b) {
    r >>= 2, g >>= 2, b >>= 2; // this is necessary apparently for this driver.
    if (side_leds[i].r != r || side_leds[i].g != g || side_leds[i].b != b) {
        flush_side_leds = true;
    }
    side_leds[i].r = r;
    side_leds[i].g = g;
    side_leds[i].b = b;
}

/**
 * @brief Set all side LED colour
 */
void side_rgb_set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i = 0; i < 12; i++) {
        side_rgb_set_color(i, r, g, b);
    }
}

/**
 * @brief  refresh side leds.
 */
void side_rgb_refresh(void) {
    if (!is_side_rgb_off()) {
        side_led_last_act = 0;
        pwr_side_led_on(); // power on side LED before refresh
    }
    if (!flush_side_leds) return;
    side_ws2812_setleds(side_leds, SIDE_LED_NUM);
    flush_side_leds = false;
}

/**
 * @brief  Adjusting the brightness of side lights.
 * @param  dir: 0 - decrease, 1 - increase.
 * @note  save to eeprom.
 */
void side_light_control(uint8_t dir) {
    if (dir) {
        if (user_config.side_light > SIDE_BRIGHT_MAX) {
            return;
        } else
            user_config.side_light++;
    } else {
        if (user_config.side_light == 0) {
            return;
        } else
            user_config.side_light--;
    }
    eeconfig_update_kb_datablock(&user_config);
}

/**
 * @brief  Adjusting the speed of side lights.
 * @param  dir: 0 - decrease, 1 - increase.
 * @note  save to eeprom.
 */
void side_speed_control(uint8_t dir) {
    if (dir) {
        if (user_config.side_speed == 0) {
            return;
        }
        user_config.side_speed--;
    } else {
        if (user_config.side_speed == SIDE_SPEED_MAX) {
            return;
        }
        user_config.side_speed++;
    }
    eeconfig_update_kb_datablock(&user_config);
}

/**
 * @brief  Switch to the next color of side lights.
 * @note  save to eeprom.
 */
void side_colour_control(void) {
    if (user_config.side_mode != SIDE_WAVE) {
        if (user_config.side_rgb) {
            user_config.side_rgb    = 0;
            user_config.side_colour = 0;
        }
    }
    if (user_config.side_rgb) {
        user_config.side_rgb    = 0;
        user_config.side_colour = 0;
    } else {
        user_config.side_colour++;
        if (user_config.side_colour >= SIDE_COLOUR_MAX) {
            user_config.side_rgb    = 1;
            user_config.side_colour = 0;
        }
    }
    eeconfig_update_kb_datablock(&user_config);
}

/**
 * @brief  Change the color mode of side lights.
 * @note  save to eeprom.
 */
void side_mode_control(void) {
    user_config.side_mode++;
    if (user_config.side_mode > SIDE_OFF) {
        user_config.side_mode = 0;
    }
    side_play_point = 0;
    eeconfig_update_kb_datablock(&user_config);
}

/**
 * @brief  set left side leds.
 * @param  ...
 */
void set_left_rgb(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < 6; i++)
        side_rgb_set_color(i, r >> 2, g >> 2, b >> 2);
}

/**
 * @brief  set right side leds.
 * @param  ...
 */
void set_right_rgb(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 6; i < 12; i++)
        side_rgb_set_color(i, r >> 2, g >> 2, b >> 2);
}

/**
 * @brief Determine to trend up or down the breath tab table.
 */
bool breath_tab_trend(bool trend, uint8_t playpoint) {
    if (playpoint == 0) {
        return 1;
    }
    if (playpoint == BREATHE_TAB_LEN - 1) {
        return 0;
    }
    return trend;
}

/**
 * @brief  set left side leds.
 */
void sys_sw_led_show(void) {
    static uint32_t sys_show_timer = 0;
    static bool     sys_show_flag  = false;

    if (f_sys_show) {
        f_sys_show     = false;
        sys_show_timer = timer_read32();
        sys_show_flag  = true;
    }

    if (sys_show_flag) {
        if (dev_info.sys_sw_state == SYS_SW_MAC) {
            r_temp = 0x80;
            g_temp = 0x80;
            b_temp = 0x80;
        } else {
            r_temp = 0x00;
            g_temp = 0x00;
            b_temp = 0x80;
        }
        if ((timer_elapsed32(sys_show_timer) / 500) % 2 == 0) {
            set_right_rgb(r_temp, g_temp, b_temp);
        } else {
            set_right_rgb(0x00, 0x00, 0x00);
        }
        if (timer_elapsed32(sys_show_timer) >= 3000) {
            sys_show_flag = false;
        }
    }
}

/**
 * @brief  sleep_sw_led_show.
 */
void sleep_sw_led_show(void) {
    static uint32_t sleep_show_timer = 0;
    static bool     sleep_show_flag  = false;

    if (f_sleep_show) {
        f_sleep_show     = false;
        sleep_show_timer = timer_read32();
        sleep_show_flag  = true;
    }

    if (sleep_show_flag) {
        if (user_config.sleep_enable) {
            r_temp = 0x00;
            g_temp = 0x80;
            b_temp = 0x00;
        } else {
            r_temp = 0x80;
            g_temp = 0x00;
            b_temp = 0x00;
        }
        if ((timer_elapsed32(sleep_show_timer) / 500) % 2 == 0) {
            set_right_rgb(r_temp, g_temp, b_temp);
        } else {
            set_right_rgb(0x00, 0x00, 0x00);
        }
        if (timer_elapsed32(sleep_show_timer) >= 3000) {
            sleep_show_flag = false;
        }
    }
}

/**
 * @brief  sys_led_show.
 */
void sys_led_show(void) {
    if (dev_info.link_mode == LINK_USB) {
        if (host_keyboard_led_state().caps_lock) {
            set_left_rgb(0X00, 0x80, 0x80);
        }
    }

    else {
        if (dev_info.rf_led & 0x02) {
            set_left_rgb(0X00, 0x80, 0x80);
        }
    }
}

/**
 * @brief  light_point_playing.
 * @param trend:
 * @param step:
 * @param len:
 * @param point:
 */
static void light_point_playing(uint8_t trend, uint8_t step, uint8_t len, uint8_t *point) {
    if (trend) {
        *point += step;
        if (*point >= len) *point -= len;
    } else {
        *point -= step;
        if (*point >= len) *point = len - (255 - *point) - 1;
    }
}

/**
 * @brief  count_rgb_light.
 * @param light_temp:
 */
static void count_rgb_light(uint8_t light_temp) {
    uint16_t temp;

    temp   = (light_temp)*r_temp + r_temp;
    r_temp = temp >> 8;

    temp   = (light_temp)*g_temp + g_temp;
    g_temp = temp >> 8;

    temp   = (light_temp)*b_temp + b_temp;
    b_temp = temp >> 8;
}

/**
 * @brief  side_wave_mode_show.
 */
static void side_wave_mode_show(void) {
    uint8_t play_index;

    if (user_config.side_rgb)
        light_point_playing(0, 3, FLOW_COLOUR_TAB_LEN, &side_play_point);
    else
        light_point_playing(0, 2, WAVE_TAB_LEN, &side_play_point);

    play_index = side_play_point;
    for (int i = 0; i < SIDE_LINE; i++) {
        if (user_config.side_rgb) {
            r_temp = flow_rainbow_colour_tab[play_index][0];
            g_temp = flow_rainbow_colour_tab[play_index][1];
            b_temp = flow_rainbow_colour_tab[play_index][2];

            light_point_playing(1, 24, FLOW_COLOUR_TAB_LEN, &play_index);
        } else {
            r_temp = colour_lib[user_config.side_colour][0];
            g_temp = colour_lib[user_config.side_colour][1];
            b_temp = colour_lib[user_config.side_colour][2];

            light_point_playing(1, 12, WAVE_TAB_LEN, &play_index);
            count_rgb_light(wave_data_tab[play_index]);
        }

        count_rgb_light(side_light_table[user_config.side_light]);

        for (int j = 0; j < 2; j++) {
            side_rgb_set_color(side_led_index_tab[i][j], r_temp >> 2, g_temp >> 2, b_temp >> 2);
        }
    }
}

/**
 * @brief  side_spectrum_mode_show.
 */
static void side_spectrum_mode_show(void) {
    light_point_playing(1, 1, FLOW_COLOUR_TAB_LEN, &side_play_point);

    r_temp = flow_rainbow_colour_tab[side_play_point][0];
    g_temp = flow_rainbow_colour_tab[side_play_point][1];
    b_temp = flow_rainbow_colour_tab[side_play_point][2];

    count_rgb_light(side_light_table[user_config.side_light]);

    side_rgb_set_color_all(r_temp, g_temp, b_temp);
}

/**
 * @brief  side_breathe_mode_show.
 */
static void side_breathe_mode_show(void) {
    static uint8_t play_point = 0;
    static bool    trend      = 1;
    light_point_playing(trend, 1, BREATHE_TAB_LEN, &play_point);
    trend = breath_tab_trend(trend, play_point);

    r_temp = colour_lib[user_config.side_colour][0];
    g_temp = colour_lib[user_config.side_colour][1];
    b_temp = colour_lib[user_config.side_colour][2];

    count_rgb_light(breathe_data_tab[play_point]);
    count_rgb_light(side_light_table[user_config.side_light]);

    side_rgb_set_color_all(r_temp, g_temp, b_temp);
}

/**
 * @brief  side_static_mode_show.
 */
static void side_static_mode_show(void) {
    if (side_play_point >= SIDE_COLOUR_MAX) side_play_point = 0;

    r_temp = colour_lib[user_config.side_colour][0];
    g_temp = colour_lib[user_config.side_colour][1];
    b_temp = colour_lib[user_config.side_colour][2];

    count_rgb_light(side_light_table[user_config.side_light]);

    side_rgb_set_color_all(r_temp, g_temp, b_temp);
}

/**
 * @brief  side_off_mode_show.
 */
static void side_off_mode_show(void) {
    r_temp = 0x00;
    g_temp = 0x00;
    b_temp = 0x00;

    side_rgb_set_color_all(r_temp, g_temp, b_temp);
}

/**
 * @brief  rf_led_show.
 */
void rf_led_show(void) {
    static uint32_t rf_blink_timer = 0;
    uint16_t        rf_blink_priod = 0;

    if (rf_blink_cnt || (rf_link_show_time < RF_LINK_SHOW_TIME)) {
        if (dev_info.link_mode == LINK_RF_24) {
            r_temp = 0x00;
            g_temp = 0x80;
            b_temp = 0x00;
        } else if (dev_info.link_mode == LINK_USB) {
            r_temp = 0x80;
            g_temp = 0x80;
            b_temp = 0x00;
        } else {
            r_temp = 0x00;
            g_temp = 0x00;
            b_temp = 0x80;
        }
    } else {
        rf_blink_timer = timer_read32();
        return;
    }

    if (rf_blink_cnt) {
        if (dev_info.rf_state == RF_PAIRING)
            rf_blink_priod = RF_LED_PAIR_PERIOD;
        else
            rf_blink_priod = RF_LED_LINK_PERIOD;

        if (timer_elapsed32(rf_blink_timer) > (rf_blink_priod >> 1)) {
            r_temp = 0x00;
            g_temp = 0x00;
            b_temp = 0x00;
        }

        if (timer_elapsed32(rf_blink_timer) >= rf_blink_priod) {
            rf_blink_cnt--;
            rf_blink_timer = timer_read32();
        }
    }

    set_left_rgb(r_temp, g_temp, b_temp);
}

/**
 * @brief  bat_num_led.
 */
void bat_num_led(uint8_t bat_percent) {
    if (bat_percent >= 100) {
        bat_percent = 100;
    }

    uint8_t bat_pct_tens = bat_percent / 10;
    uint8_t bat_pct_ones = bat_percent % 10;

    uint8_t r, g, b;

    // set color
    if (bat_percent <= 15) {
        r = 0xff;
        g = 0x00;
        b = 0x00;
    } else if (bat_percent <= 50) {
        r = 0xff;
        g = 0x40;
        b = 0x00;
    } else if (bat_percent <= 80) {
        r = 0xff;
        g = 0xff;
        b = 0x00;
    } else {
        r = 0x00;
        g = 0xff;
        b = 0x00;
    }

    for (uint8_t i = 1; i <= bat_pct_tens; i++) {
        user_set_rgb_color(i, r, g, b);
    }

    for (uint8_t i = 0; i < bat_pct_ones; i++) {
        user_set_rgb_color(29 - i, r, g, b);
    }
}

void num_led_show(void) {
    static uint8_t num_bat_temp = 0;
    num_bat_temp                = dev_info.rf_baterry;
    bat_num_led(num_bat_temp);
}

void bat_led_close(void) {
    for (int i = 20; i <= 29; i++) {
        user_set_rgb_color(i, 0, 0, 0);
    }
}

/**
 * @brief  bat_percent_led.
 */
void bat_percent_led(uint8_t bat_percent) {
    uint8_t bat_end_led = 0;
    uint8_t bat_r, bat_g, bat_b;

    if (bat_percent <= 15) {
        bat_end_led = 0;
        bat_r = 0x80, bat_g = 0, bat_b = 0;
    } else if (bat_percent <= 20) {
        bat_end_led = 1;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;
    } else if (bat_percent <= 40) {
        bat_end_led = 2;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;
    } else if (bat_percent <= 60) {
        bat_end_led = 3;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;
    } else if (bat_percent <= 80) {
        bat_end_led = 4;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;
    } else if (bat_percent <= 95) {
        bat_end_led = 5;
        bat_r = 0x80, bat_g = 0x40, bat_b = 0;
    } else {
        bat_end_led = 5;
        bat_r = 0, bat_g = 0x80, bat_b = 0;
    }

    uint8_t i = 0;
    for (; i <= bat_end_led; i++)
        side_rgb_set_color(11 - i, bat_r >> 2, bat_g >> 2, bat_b >> 2);

    for (; i < 6; i++)
        side_rgb_set_color(11 - i, 0, 0, 0);
}

/**
 * @brief  bat_led_show.
 */
void bat_led_show(void) {
    static bool bat_show_flag   = 1;
    static bool bat_show_breath = 0;
    static bool f_init          = 1;
    static bool trend           = 1;

    static uint8_t  play_point     = 0;
    static uint32_t bat_play_timer = 0;
    static uint32_t bat_show_time  = 0;

    static uint32_t bat_sts_debounce = 0;
    static uint8_t  charge_state     = 0;

    if (f_init) {
        f_init        = 0;
        bat_show_time = timer_read32();
        charge_state  = dev_info.rf_charge;
    }

    if (charge_state != dev_info.rf_charge) {
        if (timer_elapsed32(bat_sts_debounce) > 1000) {
            if (((charge_state & 0x01) == 0) && ((dev_info.rf_charge & 0x01) != 0)) {
                bat_show_flag   = true;
                bat_show_breath = true;
                bat_show_time   = timer_read32();
            }
            charge_state = dev_info.rf_charge;
        }
    } else {
        bat_sts_debounce = timer_read32();
        if (timer_elapsed32(bat_show_time) > 5000) {
            bat_show_flag   = false;
            bat_show_breath = false;
        }
        if (charge_state == 0x03) { // charging, not full?
            bat_show_breath = true;
        } else if (charge_state & 0x01) { // charging
            dev_info.rf_baterry = 100;
        }
    }

    if (dev_info.rf_baterry < 15) {
        bat_show_flag = true;
        bat_show_time = timer_read32();
    }

    if (f_bat_hold || bat_show_flag) {
        if (bat_show_breath) {
            if (timer_elapsed32(bat_play_timer) > 10) {
                bat_play_timer = timer_read32();
                light_point_playing(trend, 1, BREATHE_TAB_LEN, &play_point);
                trend = breath_tab_trend(trend, play_point);
            }
            r_temp = 0x80;
            g_temp = 0x40;
            b_temp = 0x00;
            count_rgb_light(breathe_data_tab[play_point]);
            set_right_rgb(r_temp, g_temp, b_temp);
        } else {
            bat_percent_led(dev_info.rf_baterry);
        }
    }
}

/**
 * @brief  device_reset_show.
 */
void device_reset_show(void) {
    gpio_write_pin_high(DC_BOOST_PIN);
    gpio_set_pin_output(DRIVER_LED_CS_PIN);
    gpio_write_pin_low(DRIVER_LED_CS_PIN);
    pwr_side_led_on();

    for (int blink_cnt = 0; blink_cnt < 3; blink_cnt++) {
        rgb_matrix_set_color_all(0x10, 0x10, 0x10);
        set_left_rgb(0x40, 0x40, 0x40);
        set_right_rgb(0x40, 0x40, 0x40);
        rgb_matrix_update_pwm_buffers();
        side_rgb_refresh();
        wait_ms(200);

        rgb_matrix_set_color_all(0x00, 0x00, 0x00);
        set_left_rgb(0x00, 0x00, 0x00);
        set_right_rgb(0x00, 0x00, 0x00);
        rgb_matrix_update_pwm_buffers();
        side_rgb_refresh();
        wait_ms(200);
    }
}

/**
 * @brief  device_reset_init.
 */
void device_reset_init(void) {
    f_bat_hold = false;

    rgb_matrix_enable();
    rgb_matrix_mode(RGB_MATRIX_DEFAULT_MODE);
    rgb_matrix_set_speed(255);
    rgb_matrix_sethsv(255, 255, RGB_MATRIX_MAXIMUM_BRIGHTNESS);
}

/**
 *      RGB test
 */
void rgb_test_show(void) {
    // open power control
    gpio_write_pin_high(DC_BOOST_PIN);
    gpio_set_pin_output(DRIVER_LED_CS_PIN);
    gpio_write_pin_low(DRIVER_LED_CS_PIN);
    pwr_side_led_on();

    // set test color
    rgb_matrix_set_color_all(0xFF, 0x00, 0x00);
    rgb_matrix_update_pwm_buffers();
    set_left_rgb(0xff, 0x00, 0x00);
    set_right_rgb(0xff, 0x00, 0x00);
    side_rgb_refresh();
    wait_ms(500);

    rgb_matrix_set_color_all(0x00, 0xFF, 0x00);
    rgb_matrix_update_pwm_buffers();
    set_left_rgb(0x00, 0xFF, 0x00);
    set_right_rgb(0x00, 0xFF, 0x00);
    side_rgb_refresh();
    wait_ms(500);

    rgb_matrix_set_color_all(0x00, 0x00, 0xFF);
    rgb_matrix_update_pwm_buffers();
    set_left_rgb(0x00, 0x00, 0xFF);
    set_right_rgb(0x00, 0x00, 0xFF);
    side_rgb_refresh();
    wait_ms(500);

    rgb_matrix_set_color_all(0x80, 0x80, 0x80);
    rgb_matrix_update_pwm_buffers();
    set_left_rgb(0x80, 0x80, 0x80);
    set_right_rgb(0x80, 0x80, 0x80);
    side_rgb_refresh();
    wait_ms(500);

    rgb_matrix_set_color_all(0x80, 0x80, 0x00);
    rgb_matrix_update_pwm_buffers();
    set_left_rgb(0x80, 0x80, 0x00);
    set_right_rgb(0x80, 0x80, 0x00);
    side_rgb_refresh();
    wait_ms(500);

    rgb_matrix_set_color_all(0x80, 0x00, 0x80);
    rgb_matrix_update_pwm_buffers();
    set_left_rgb(0x80, 0x00, 0x80);
    set_right_rgb(0x80, 0x00, 0x80);
    side_rgb_refresh();
    wait_ms(500);

    rgb_matrix_set_color_all(0x00, 0x80, 0x80);
    rgb_matrix_update_pwm_buffers();
    set_left_rgb(0x00, 0x80, 0x80);
    set_right_rgb(0x00, 0x80, 0x80);
    side_rgb_refresh();
    wait_ms(500);
}

/**
 * @brief  side_led_show.
 */
void side_led_show(void) {
    static uint32_t side_refresh_time = 0;
    static uint32_t side_update_time  = 0;
    static bool     do_refresh        = false;

    uint8_t update_interval = side_speed_table[user_config.side_mode][user_config.side_speed];
    if (timer_elapsed32(side_update_time) >= update_interval) {
        side_update_time = timer_read32();
        do_refresh       = true;
        switch (user_config.side_mode) {
            case SIDE_WAVE:
                side_wave_mode_show();
                break;
            case SIDE_MIX:
                side_spectrum_mode_show();
                break;
            case SIDE_BREATH:
                side_breathe_mode_show();
                break;
            case SIDE_STATIC:
                side_static_mode_show();
                break;
            case SIDE_OFF:
                side_off_mode_show();
                break;
        }
    }

    bat_led_show();
    sleep_sw_led_show();
    sys_sw_led_show();

    sys_led_show();
    rf_led_show();

    // This only refreshes if LEDs change anyways. Fixes side LED not refreshing synchronously.
    if ((do_refresh || timer_elapsed32(side_refresh_time) >= 10) && !f_goto_sleep) {
        side_refresh_time = timer_read32();
        do_refresh        = false;
        side_rgb_refresh();
    }
}
