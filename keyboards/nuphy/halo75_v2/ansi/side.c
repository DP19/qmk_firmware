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
#include "ansi.h"
#include "side.h"
#include "user_kb.h"
#include "mcu_pwr.h"
//------------------------------------------------
enum {
    SIDE_WAVE = 0,
    SIDE_MIX,
    SIDE_NEW,
    SIDE_BREATH,
    SIDE_STATIC,
};

enum {
    SIDE_MODE_1 = 0,
    SIDE_MODE_2,
    SIDE_MODE_3,
    SIDE_MODE_4,
    SIDE_MODE_5,
    SIDE_MODE_6,
    SIDE_MODE_7,
};

#define LIGHT_COLOUR_MAX 8
#define SIDE_COLOUR_MAX 8
#define LIGHT_SPEED_MAX 4

const uint8_t side_speed_table[5][5] = {
    [SIDE_WAVE] = {10, 20, 25, 30, 45}, [SIDE_MIX] = {25, 30, 40, 50, 60}, [SIDE_NEW] = {30, 50, 60, 70, 100}, [SIDE_BREATH] = {25, 30, 40, 50, 60}, [SIDE_STATIC] = {10, 20, 25, 30, 45},
};

#define SIDE_BLINK_LIGHT 128
const uint8_t side_light_table[5] = {
    0, 64, 128, 192, 255,
};

const uint8_t side_led_index_tab[45] = {
    SIDE_INDEX + 10, SIDE_INDEX + 11, SIDE_INDEX + 12, SIDE_INDEX + 13, SIDE_INDEX + 14, SIDE_INDEX + 15, SIDE_INDEX + 16, SIDE_INDEX + 17, SIDE_INDEX + 18, SIDE_INDEX + 19, SIDE_INDEX + 20, SIDE_INDEX + 21, SIDE_INDEX + 22, SIDE_INDEX + 23, SIDE_INDEX + 24, SIDE_INDEX + 25, SIDE_INDEX + 26, SIDE_INDEX + 27,

    SIDE_INDEX + 0,  SIDE_INDEX + 1,  SIDE_INDEX + 2,  SIDE_INDEX + 3,  SIDE_INDEX + 4,

    SIDE_INDEX + 28, SIDE_INDEX + 29, SIDE_INDEX + 30, SIDE_INDEX + 31, SIDE_INDEX + 32, SIDE_INDEX + 33, SIDE_INDEX + 34, SIDE_INDEX + 35, SIDE_INDEX + 36, SIDE_INDEX + 37, SIDE_INDEX + 38, SIDE_INDEX + 39, SIDE_INDEX + 40, SIDE_INDEX + 41, SIDE_INDEX + 42, SIDE_INDEX + 43, SIDE_INDEX + 44,

    SIDE_INDEX + 9,  SIDE_INDEX + 8,  SIDE_INDEX + 7,  SIDE_INDEX + 6,  SIDE_INDEX + 5,
};

bool     f_charging      = 1;
bool     bat_show_flag   = 1;
bool     side_off        = 0;
uint16_t side_play_cnt   = 0;
uint32_t side_play_timer = 0;
uint8_t  side_line       = 45;
uint8_t  side_play_point = 0;
uint32_t bat_show_time   = 0;

uint8_t light_colour_max;
uint8_t r_temp, g_temp, b_temp;

extern DEV_INFO_STRUCT dev_info;
extern user_config_t   user_config;
extern uint16_t        rf_link_show_time;
extern bool            f_bat_hold;
extern bool            f_sys_show;
extern bool            f_sleep_show;
extern bool            f_power_togg_show;
extern uint16_t        rgb_led_last_act;

#define IS31FL3733_PWM_REGISTER_COUNT 192
#define IS31FL3733_LED_CONTROL_REGISTER_COUNT 24
typedef struct is31fl3733_driver_t {
    uint8_t pwm_buffer[IS31FL3733_PWM_REGISTER_COUNT];
    bool    pwm_buffer_dirty;
    uint8_t led_control_buffer[IS31FL3733_LED_CONTROL_REGISTER_COUNT];
    bool    led_control_buffer_dirty;
} PACKED                   is31fl3733_driver_t;
extern is31fl3733_driver_t driver_buffers[2];
bool                       is_side_rgb_off(void) {
    is31fl3733_led_t led;
    for (int i = SIDE_INDEX; i < SIDE_INDEX + 10; i++) {
        memcpy_P(&led, (&g_is31_leds[i]), sizeof(led));
        if ((driver_buffers[led.driver].pwm_buffer[led.r] != 0) || (driver_buffers[led.driver].pwm_buffer[led.g] != 0) || (driver_buffers[led.driver].pwm_buffer[led.b] != 0)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief  Adjusting the brightness of side lights.
 * @param  dir: 0 - decrease, 1 - increase.
 * @note  save to eeprom.
 */
void light_level_control(uint8_t brighten) {
    if (brighten) {
        if (user_config.side_light == 4) {
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
void light_speed_contol(uint8_t fast) {
    if ((user_config.side_speed) > LIGHT_SPEED_MAX) (user_config.side_speed) = LIGHT_SPEED_MAX / 2;

    if (fast) {
        if ((user_config.side_speed)) user_config.side_speed--;
    } else {
        if ((user_config.side_speed) < LIGHT_SPEED_MAX) user_config.side_speed++;
    }
    eeconfig_update_kb_datablock(&user_config);
}

/**
 * @brief  Switch to the next color of side lights.
 * @param  dir: 0 - prev, 1 - next.
 * @note  save to eeprom.
 */
void side_colour_control(void) {
    if (user_config.side_mode_a == SIDE_NEW) {
        light_colour_max = 3;
    } else {
        light_colour_max = LIGHT_COLOUR_MAX;
    }

    if ((user_config.side_mode_a != SIDE_WAVE) && (user_config.side_mode_a != SIDE_BREATH)) {
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
        if (user_config.side_colour >= light_colour_max) {
            user_config.side_rgb    = 1;
            user_config.side_colour = 0;
        }
    }
    eeconfig_update_kb_datablock(&user_config);
}

/**
 * @brief  Change the color mode of side lights.
 * @param  dir: 0 - prev, 1 - next.
 * @note  save to eeprom.
 */
uint8_t side_old_color = 0;
void    side_mode_a_control(void) {
    user_config.side_mode_a++;
    if (user_config.side_mode_a > SIDE_STATIC) {
        user_config.side_mode_a = 0;
    }

    if (user_config.side_mode_a == SIDE_NEW) {
        side_old_color          = user_config.side_colour;
        user_config.side_colour = 0;
    } else if (user_config.side_mode_a == SIDE_BREATH) {
        user_config.side_colour = side_old_color;
    }

    side_play_point = 0;
    eeconfig_update_kb_datablock(&user_config);
}

void side_mode_b_control(void) {
    user_config.side_mode_b++;
    if (user_config.side_mode_b > SIDE_MODE_7) {
        user_config.side_mode_b = SIDE_MODE_1;
    }
    side_play_point = 0;
    eeconfig_update_kb_datablock(&user_config);
}

void user_set_side_rgb_color(int index, uint8_t red, uint8_t green, uint8_t blue) {
    if (side_off) {
        side_off = 0;
    }
    rgb_led_last_act = 0;
    rgb_matrix_set_color(index, red, green, blue);
}

/**
 * @brief  set left side leds.
 * @param  ...
 */
void set_left_rgb(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < 5; i++)
        user_set_side_rgb_color(SIDE_INDEX + i, r, g, b);
}

void set_logo_rgb(uint8_t r, uint8_t g, uint8_t b) {
    rgb_matrix_set_color(LOGO_INDEX, r, g, b);
}

void set_all_side_off(void) {
    if (side_off) {
        return;
    } else {
        for (int i = 0; i < 45; i++)
            user_set_side_rgb_color(SIDE_INDEX + i, 0, 0, 0);
        side_off = 1;
    }
}

/**
 * @brief  mac or win system indicate
 */
void sys_sw_led_show(void) {
    static uint32_t sys_show_timer = 0;
    static bool     sys_show_flag  = false;

    if (f_sys_show) {
        f_sys_show     = false;
        sys_show_timer = timer_read32(); // store time of last refresh
        sys_show_flag  = true;
    }

    if (sys_show_flag) {
        if (dev_info.sys_sw_state == SYS_SW_MAC) {
            r_temp = colour_lib[7][0];
            g_temp = colour_lib[7][1];
            b_temp = colour_lib[7][2];
        } else {
            r_temp = colour_lib[5][0];
            g_temp = colour_lib[5][1];
            b_temp = colour_lib[5][2];
        }
        if ((timer_elapsed32(sys_show_timer) / 500) % 2 == 0) {
            set_left_rgb(r_temp, g_temp, b_temp);
        } else {
            set_left_rgb(0x00, 0x00, 0x00);
        }
        if (timer_elapsed32(sys_show_timer) >= (3000 - 50)) {
            sys_show_flag = false;
        }
    }
}

/**
 * @brief  sleep enable or disable indicate
 Highjack for power ani toggle
 */
void sw_led_show(void) {
    static uint32_t show_timer      = 0;
    static bool     sleep_show_flag = false;
    static bool     power_togg_flag = false;

    if (f_sleep_show) {
        f_sleep_show    = false;
        show_timer      = timer_read32(); // store time of last refresh
        sleep_show_flag = true;
    }

    if (f_power_togg_show) {
        f_power_togg_show = false;
        show_timer        = timer_read32(); // store time of last refresh
        power_togg_flag   = true;
    }

    if (sleep_show_flag) {
        if (user_config.sleep_enable) {
            r_temp = 0x00;
            g_temp = SIDE_BLINK_LIGHT;
            b_temp = 0x00;
        } else {
            r_temp = 0xff;
            g_temp = 0x00;
            b_temp = 0x00;
        }
    }

    if (power_togg_flag) {
        if (user_config.power_on_show) {
            r_temp = 0x00;
            g_temp = SIDE_BLINK_LIGHT;
            b_temp = 0x00;
        } else {
            r_temp = 0xff;
            g_temp = 0x00;
            b_temp = 0x00;
        }
    }

    if (sleep_show_flag || power_togg_flag) {
        if ((timer_elapsed32(show_timer) / 500) % 2 == 0) {
            set_left_rgb(r_temp, g_temp, b_temp);
        } else {
            set_left_rgb(0x00, 0x00, 0x00);
        }
        if (timer_elapsed32(show_timer) >= (3000 - 50)) {
            sleep_show_flag = false;
            power_togg_flag = false;
        }
    }
}

/**
 * @brief  host system led indicate.
 */
void sys_led_show(void) {
    if (host_keyboard_led_state().caps_lock) {
        set_left_rgb(colour_lib[4][0], colour_lib[4][1], colour_lib[4][2]);
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
 * @brief  auxiliary_rgb_light.
 */
uint8_t f_side_flag      = 0x1f;
uint8_t key_pwm_tab[45]  = {0x00};
uint8_t power_play_index = 0;
uint8_t f_power_show     = 1;
uint8_t is_side_rgb_on(uint8_t index) {
    if ((((index >= 0) && (index <= 10)) || ((index >= 37) && (index <= 39))) && (f_side_flag & 0x01))
        return true;
    else if ((((index >= 11) && (index <= 17)) || ((index >= 23) && (index <= 29)) || ((index >= 32) && (index <= 36))) && (f_side_flag & 0x02))
        return true;
    else if (((index >= 40) && (index <= 44)) && (f_side_flag & 0x04))
        return true;
    else if (((index >= 18) && (index <= 22)) && (f_side_flag & 0x08))
        return true;
    else if (((index >= 30) && (index <= 31)) && (f_side_flag & 0x10))
        return true;
    else
        return false;
}

static void side_power_mode_show(void) {
    if (user_config.power_on_show) {
        if (side_play_cnt <= side_speed_table[0][user_config.side_speed])
            return;
        else
            side_play_cnt -= side_speed_table[0][user_config.side_speed];
        if (side_play_cnt > 20) side_play_cnt = 0;

        if (power_play_index <= 45) {
            key_pwm_tab[power_play_index] = 0xff;
            power_play_index++;
        }

        uint8_t i;

        for (i = 0; i < 45; i++) {
            r_temp = colour_lib[user_config.side_colour][0];
            g_temp = colour_lib[user_config.side_colour][1];
            b_temp = colour_lib[user_config.side_colour][2];

            count_rgb_light(key_pwm_tab[i]);
            count_rgb_light(side_light_table[2]);
            user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
        }

        for (i = 0; i < 45; i++) {
            if (key_pwm_tab[i] & 0x80)
                key_pwm_tab[i] -= 8;
            else if (key_pwm_tab[i] & 0x40)
                key_pwm_tab[i] -= 6;
            else if (key_pwm_tab[i] & 0x20)
                key_pwm_tab[i] -= 4;
            else if (key_pwm_tab[i] & 0x10)
                key_pwm_tab[i] -= 3;
            else if (key_pwm_tab[i] & 0x08)
                key_pwm_tab[i] -= 2;
            else if (key_pwm_tab[i])
                key_pwm_tab[i]--;
        }

        if (key_pwm_tab[44] == 1) {
            f_power_show      = 0;
            rf_link_show_time = 0;
            bat_show_flag     = true;
            f_charging        = true;
            bat_show_time     = timer_read32();
        }
    } else {
        f_power_show      = 0;
        rf_link_show_time = 0;
        bat_show_flag     = true;
        f_charging        = true;
        bat_show_time     = timer_read32();
    }
}

/**
 * @brief  side_wave_mode_show.
 */
static void side_wave_mode_show(void) {
    uint8_t play_index;
    uint8_t play_index_1;

    if (side_play_cnt <= side_speed_table[user_config.side_mode_a][user_config.side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[user_config.side_mode_a][user_config.side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    if (user_config.side_rgb)
        light_point_playing(0, 1, FLOW_COLOUR_TAB_LEN, &side_play_point);
    else
        light_point_playing(0, 1, WAVE_TAB_LEN, &side_play_point);

    play_index = side_play_point;
    if (side_line == 0 || user_config.side_light == 0) {
        set_all_side_off();
        return;
    }
    for (int i = 0; i <= side_line - 5; i++) {
        if (user_config.side_rgb) {
            r_temp = flow_rainbow_colour_tab[play_index][0];
            g_temp = flow_rainbow_colour_tab[play_index][1];
            b_temp = flow_rainbow_colour_tab[play_index][2];

            light_point_playing(1, 5, FLOW_COLOUR_TAB_LEN, &play_index);

        } else {
            r_temp = colour_lib[user_config.side_colour][0];
            g_temp = colour_lib[user_config.side_colour][1];
            b_temp = colour_lib[user_config.side_colour][2];

            light_point_playing(1, 5, WAVE_TAB_LEN, &play_index);
            count_rgb_light(wave_data_tab[play_index]);
        }

        count_rgb_light(side_light_table[user_config.side_light]);

        play_index_1 = play_index;

        if (i == 40) {
            if (f_side_flag == 0x1f) {
                for (; i < 45; i++) {
                    if (user_config.side_rgb) {
                        r_temp = flow_rainbow_colour_tab[play_index_1][0] * 0.4;
                        g_temp = flow_rainbow_colour_tab[play_index_1][1] * 0.4;
                        b_temp = flow_rainbow_colour_tab[play_index_1][2] * 0.4;
                    } else {
                        r_temp = colour_lib[user_config.side_colour][0] * 0.4;
                        g_temp = colour_lib[user_config.side_colour][1] * 0.4;
                        b_temp = colour_lib[user_config.side_colour][2] * 0.4;
                        count_rgb_light(wave_data_tab[play_index_1]);
                    }
                    count_rgb_light(side_light_table[user_config.side_light]);
                    user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
                }
                return;
            } else {
                for (; i < 45; i++) {
                    user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
                }
                return;
            }
        }
        if (is_side_rgb_on(i))
            user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
        else
            user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
    }
}

static void side_new_mode_show(void) {
    uint8_t play_index;

    if (side_play_cnt <= side_speed_table[user_config.side_mode_a][user_config.side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[user_config.side_mode_a][user_config.side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    light_point_playing(0, 1, (side_line - 5), &side_play_point);
    play_index = side_play_point;
    if (side_line == 0 || user_config.side_light == 0) {
        set_all_side_off();
        return;
    }
    for (int i = 0; i <= (side_line - 5); i++) {
        if (play_index < (side_line - 5) / 2) {
            r_temp = dual_colour_lib[user_config.side_colour][0];
            g_temp = dual_colour_lib[user_config.side_colour][1];
            b_temp = dual_colour_lib[user_config.side_colour][2];
        } else {
            r_temp = dual_colour_lib[user_config.side_colour][3];
            g_temp = dual_colour_lib[user_config.side_colour][4];
            b_temp = dual_colour_lib[user_config.side_colour][5];
        }

        light_point_playing(1, 1, (side_line - 5), &play_index);

        count_rgb_light(side_light_table[user_config.side_light]);

        if (i == 40) {
            if (f_side_flag == 0x1f) {
                r_temp = r_temp * 0.3;
                g_temp = g_temp * 0.3;
                b_temp = b_temp * 0.3;

                for (; i < 45; i++) {
                    user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
                }
                return;
            } else {
                for (; i < 45; i++) {
                    user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
                }
                return;
            }
        }
        if (is_side_rgb_on(i))
            user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
        else
            user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
    }
}

static void side_spectrum_mode_show(void) {
    if (side_play_cnt <= side_speed_table[user_config.side_mode_a][user_config.side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[user_config.side_mode_a][user_config.side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    if (side_line == 0 || user_config.side_light == 0) {
        set_all_side_off();
        return;
    }

    light_point_playing(1, 1, FLOW_COLOUR_TAB_LEN, &side_play_point);

    r_temp = flow_rainbow_colour_tab[side_play_point][0];
    g_temp = flow_rainbow_colour_tab[side_play_point][1];
    b_temp = flow_rainbow_colour_tab[side_play_point][2];

    count_rgb_light(side_light_table[user_config.side_light]);

    for (int i = 0; i <= 40; i++) {
        if (i == 40) {
            if (f_side_flag == 0x1f) {
                r_temp = r_temp * 0.3;
                g_temp = g_temp * 0.3;
                b_temp = b_temp * 0.3;
                for (; i < 45; i++)
                    user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
                return;
            } else {
                for (; i < 45; i++)
                    user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
                return;
            }
        }
        if (is_side_rgb_on(i))
            user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
        else
            user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
    }
}

static void side_breathe_mode_show(void) {
    static uint8_t play_point = 0;
    static uint8_t colour     = 0;

    if (side_play_cnt <= side_speed_table[user_config.side_mode_a][user_config.side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[user_config.side_mode_a][user_config.side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    if (side_line == 0 || user_config.side_light == 0) {
        set_all_side_off();
        return;
    }

    light_point_playing(0, 1, BREATHE_TAB_LEN, &play_point);

    if (user_config.side_rgb) {
        if (play_point == 0) {
            if (++colour >= LIGHT_COLOUR_MAX) colour = 0;
        }
        r_temp = colour_lib[colour][0];
        g_temp = colour_lib[colour][1];
        b_temp = colour_lib[colour][2];

    } else {
        r_temp = colour_lib[user_config.side_colour][0];
        g_temp = colour_lib[user_config.side_colour][1];
        b_temp = colour_lib[user_config.side_colour][2];
    }
    count_rgb_light(breathe_data_tab[play_point]);
    count_rgb_light(side_light_table[user_config.side_light]);

    for (int i = 0; i <= 40; i++) {
        if (i == 40) {
            if (f_side_flag == 0x1f) {
                r_temp = r_temp * 0.3;
                g_temp = g_temp * 0.3;
                b_temp = b_temp * 0.3;
                for (; i < 45; i++)
                    user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
                return;
            } else {
                for (; i < 45; i++)
                    user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
                return;
            }
        }
        if (is_side_rgb_on(i))
            user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
        else
            user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
    }
}

/**
 * @brief  side_static_mode_show.
 */
static void side_static_mode_show(void) {
    if (side_play_cnt <= side_speed_table[user_config.side_mode_a][user_config.side_speed])
        return;
    else
        side_play_cnt -= side_speed_table[user_config.side_mode_a][user_config.side_speed];
    if (side_play_cnt > 20) side_play_cnt = 0;

    if (side_line == 0 || user_config.side_light == 0) {
        set_all_side_off();
        return;
    }

    if (side_play_point >= SIDE_COLOUR_MAX) side_play_point = 0;

    for (int i = 0; i < side_line; i++) {
        r_temp = colour_lib[user_config.side_colour][0];
        g_temp = colour_lib[user_config.side_colour][1];
        b_temp = colour_lib[user_config.side_colour][2];

        if ((side_led_index_tab[i] <= SIDE_INDEX + 9) && (side_led_index_tab[i] >= SIDE_INDEX)) {
            r_temp = colour_lib_1[user_config.side_colour][0] * 0.7;
            g_temp = colour_lib_1[user_config.side_colour][1] * 0.7;
            b_temp = colour_lib_1[user_config.side_colour][2] * 0.7;
        }

        count_rgb_light(side_light_table[user_config.side_light]);

        if (is_side_rgb_on(i))
            user_set_side_rgb_color(side_led_index_tab[i], r_temp, g_temp, b_temp);
        else
            user_set_side_rgb_color(side_led_index_tab[i], 0, 0, 0);
    }
}

/**
 * @brief  bat_chargeing_breathe.
 */
void bat_charging_breathe(void) {
    static uint32_t interval_timer = 0;
    static uint8_t  play_point     = 0;

    if (timer_elapsed32(interval_timer) > 30) {
        interval_timer = timer_read32();
        light_point_playing(0, 2, BREATHE_TAB_LEN, &play_point);
    }

    r_temp = 0x80;
    g_temp = 0x40;
    b_temp = 0x00;
    count_rgb_light(breathe_data_tab[play_point]);
    set_left_rgb(r_temp, g_temp, b_temp);
}

/**
 * @brief  bat_chargeing_design.
 */
void bat_charging_design(uint8_t init, uint8_t r, uint8_t g, uint8_t b) {
    static uint32_t interval_timer = 0;
    static uint16_t show_mask      = 0x00;
    static bool     f_move_trend   = 0;
    uint16_t        bit_mask       = 1;
    uint8_t         i;

    if (timer_elapsed32(interval_timer) > 100) {
        interval_timer = timer_read32();

        if (f_move_trend) {
            show_mask >>= 1;
            if (show_mask == 0x1f >> (side_line - init)) f_move_trend = 0;
        } else {
            show_mask <<= 1;
            show_mask |= 1;
            if (show_mask == 0x7f) f_move_trend = 1;
        }
    }

    for (i = 0; i < side_line; i++) {
        if (show_mask & bit_mask) {
            user_set_side_rgb_color(i, r, g, b);
        } else {
            user_set_side_rgb_color(i, 0x00, 0x00, 0x00);
        }
        bit_mask <<= 1;
    }
}

/**
 * @brief  rf state indicate
 */
#define RF_LED_LINK_PERIOD 300
#define RF_LED_PAIR_PERIOD 200
void rf_led_show(void) {
    static uint32_t rf_blink_timer = 0;
    uint16_t        rf_blink_priod = 0;
    static bool     flag_power_on  = 1;

    if (dev_info.link_mode == LINK_RF_24) {
        r_temp = colour_lib[3][0];
        g_temp = colour_lib[3][1];
        b_temp = colour_lib[3][2];
    } else if (dev_info.link_mode == LINK_USB) {
        r_temp = colour_lib[2][0];
        g_temp = colour_lib[2][1];
        b_temp = colour_lib[2][2];
        if (flag_power_on && (rf_link_show_time < RF_LINK_SHOW_TIME)) return;
    } else {
        r_temp = colour_lib[5][0];
        g_temp = colour_lib[5][1];
        b_temp = colour_lib[5][2];
    }

    flag_power_on = 0;

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
    } else if (rf_link_show_time < RF_LINK_SHOW_TIME) {
        // showing led during period
    } else {
        rf_blink_timer = timer_read32();
        return;
    }

    set_left_rgb(r_temp, g_temp, b_temp);
}

uint8_t low_bat_blink_cnt = 6;
#define LOW_BAT_BLINK_PRIOD 500
void low_bat_show(void) {
    static uint32_t interval_timer = 0;

    r_temp = 0x80, g_temp = 0, b_temp = 0;

    if (low_bat_blink_cnt) {
        if (timer_elapsed32(interval_timer) > (LOW_BAT_BLINK_PRIOD >> 1)) {
            r_temp = 0x00;
            g_temp = 0x00;
            b_temp = 0x00;
        }

        if (timer_elapsed32(interval_timer) >= LOW_BAT_BLINK_PRIOD) {
            interval_timer = timer_read32();
            low_bat_blink_cnt--;
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
        user_set_rgb_color(16 + i, r, g, b);
    }
}

void num_led_show(void) {
    static uint8_t num_bat_temp = 0;
    num_bat_temp                = dev_info.rf_battery;
    bat_num_led(num_bat_temp);
}

uint8_t bat_pwm_buf[6 * 3] = {0};
uint8_t bat_end_led        = 0;
uint8_t bat_r, bat_g, bat_b;

/**
 * @brief  Battery level indicator
 */
void bat_percent_led(uint8_t bat_percent) {
    uint8_t i;

    if (bat_percent <= 20) {
        bat_end_led = 1;
        bat_r       = colour_lib[0][0];
        bat_g       = colour_lib[0][1];
        bat_b       = colour_lib[0][2];
    } else if (bat_percent <= 50) {
        bat_end_led = 2;
        bat_r       = colour_lib[1][0];
        bat_g       = colour_lib[1][1];
        bat_b       = colour_lib[1][2];
    } else if (bat_percent <= 80) {
        bat_end_led = 4;
        bat_r       = colour_lib[2][0];
        bat_g       = colour_lib[2][1];
        bat_b       = colour_lib[2][2];
    } else {
        bat_end_led = 5;
        bat_r       = colour_lib[3][0];
        bat_g       = colour_lib[3][1];
        bat_b       = colour_lib[3][2];
    }
    if (f_charging) {
        low_bat_blink_cnt = 6;
#if (CHARGING_SHIFT)
        bat_charging_design(bat_end_led, bat_r >> 2, bat_g >> 2, bat_b >> 2);
#else
        bat_charging_breathe();
#endif
    } else if (bat_percent < 10) {
        low_bat_show();
    } else {
        bat_end_led       = 4;
        low_bat_blink_cnt = 6;
        for (i = 0; i <= bat_end_led; i++)
            user_set_side_rgb_color(SIDE_INDEX + i, bat_r, bat_g, bat_b);
    }
}

bool low_bat_flag = 0;
/**
 * @brief  battery state indicate
 */
void bat_led_show(void) {
    static bool     bat_show_flag    = true;
    static uint32_t bat_show_time    = 0;
    static uint32_t bat_sts_debounce = 0;
    static uint32_t bat_per_debounce = 0;
    static uint8_t  charge_state     = 0;
    static uint8_t  bat_percent      = 0;
    static bool     f_init           = 1;

    if (dev_info.link_mode != LINK_USB) {
        if (rf_link_show_time < RF_LINK_SHOW_TIME) return;

        if (dev_info.rf_state != RF_CONNECT) return;
    }

    if (f_init) {
        f_init        = 0;
        bat_show_time = timer_read32();
        charge_state  = dev_info.rf_charge;
        bat_percent   = dev_info.rf_battery;
    }

    if (charge_state != dev_info.rf_charge) {
        if (timer_elapsed32(bat_sts_debounce) > 1000) {
            if (((charge_state & 0x01) == 0) && ((dev_info.rf_charge & 0x01) != 0)) {
                bat_show_flag = true;
                f_charging    = true;
                bat_show_time = timer_read32();
            }
            charge_state = dev_info.rf_charge;
        }
    } else {
        bat_sts_debounce = timer_read32();
        if (f_charging) {
            if (timer_elapsed32(bat_show_time) > 10000) {
                bat_show_flag = false;
                f_charging    = false;
            }
        } else {
            if (timer_elapsed32(bat_show_time) > 2000) {
                bat_show_flag = false;
            }
        }
        if (charge_state == 0x03) {
            f_charging = true;
        } else if (!(charge_state & 0x01)) {
            f_charging = 0;
        }
    }

    if (bat_percent != dev_info.rf_battery) {
        if (timer_elapsed32(bat_per_debounce) > 1000) {
            bat_percent = dev_info.rf_battery;
        }
    } else {
        bat_per_debounce = timer_read32();

        if ((bat_percent < 10) && (!(charge_state & 0x01))) {
            bat_show_flag = true;
            bat_show_time = timer_read32();
            low_bat_flag  = 1;
            if (rgb_matrix_config.hsv.v > RGB_MATRIX_VAL_STEP) {
                rgb_matrix_config.hsv.v = RGB_MATRIX_VAL_STEP;
            }

            if (user_config.side_light > 1) {
                user_config.side_light = 1;
                eeconfig_update_kb_datablock(&user_config);
            }
        } else
            low_bat_flag = 0;
    }
    if (f_bat_hold || bat_show_flag) {
        bat_percent_led(bat_percent);
    }
}

/**
 * @brief  device_reset_show.
 */
void device_reset_show(void) {
    writePinHigh(DC_BOOST_PIN);
    writePinHigh(RGB_DRIVER_SDB1);
    writePinHigh(RGB_DRIVER_SDB2);
    for (int blink_cnt = 0; blink_cnt < 3; blink_cnt++) {
        rgb_matrix_set_color_all(0xFF, 0xFF, 0xFF);
        rgb_matrix_update_pwm_buffers();
        wait_ms(200);

        rgb_matrix_set_color_all(0x00, 0x00, 0x00);
        rgb_matrix_update_pwm_buffers();
        wait_ms(200);
    }
}

void device_reset_init(void) {
    side_play_cnt   = 0;
    side_play_timer = timer_read32();

    f_bat_hold = false;

    rgb_matrix_enable();
    rgb_matrix_mode(RGB_MATRIX_DEFAULT_MODE);
    rgb_matrix_set_speed(255 - RGB_MATRIX_SPD_STEP * 2);
    rgb_matrix_sethsv(RGB_DEFAULT_COLOUR, 255, RGB_MATRIX_MAXIMUM_BRIGHTNESS - RGB_MATRIX_VAL_STEP * 2);
}

/**
 * @brief  rgb test
 */
void rgb_test_show(void) {
    writePinHigh(DC_BOOST_PIN);
    writePinHigh(RGB_DRIVER_SDB1);
    writePinHigh(RGB_DRIVER_SDB2);
    rgb_matrix_set_color_all(0xFF, 0x00, 0x00);
    rgb_matrix_update_pwm_buffers();
    wait_ms(1000);
    rgb_matrix_set_color_all(0x00, 0xFF, 0x00);
    rgb_matrix_update_pwm_buffers();
    wait_ms(1000);
    rgb_matrix_set_color_all(0x00, 0x00, 0xFF);
    rgb_matrix_update_pwm_buffers();
    wait_ms(1000);
}

/**
 * @brief  side_led_show.
 */
void side_led_show(void) {
    static bool flag_power_on = 1;
    extern bool f_dial_sw_init_ok;

    side_play_cnt += timer_elapsed32(side_play_timer);
    side_play_timer = timer_read32();

    if (flag_power_on) {
        if (!f_dial_sw_init_ok) return;
        flag_power_on = 0;
    }

    if (f_power_show) {
        side_power_mode_show();
        return;
    }

    switch (user_config.side_mode_b) {
        case SIDE_MODE_1:
            side_line   = 0;
            f_side_flag = 0;
            break;

        case SIDE_MODE_2:
            side_line   = 45;
            f_side_flag = 0x08;
            break;

        case SIDE_MODE_3: // side left and right
            side_line   = 45;
            f_side_flag = 0x18;
            break;

        case SIDE_MODE_4:
            side_line   = 45;
            f_side_flag = 0x1f;
            break;

        case SIDE_MODE_5:
            side_line   = 45;
            f_side_flag = 0x01;
            break;

        case SIDE_MODE_6:
            side_line   = 45;
            f_side_flag = 0x09;
            break;

        case SIDE_MODE_7:
            side_line   = 45;
            f_side_flag = 0x19;
            break;
        default:
            break;
    }

    if (f_goto_sleep) return;

    switch (user_config.side_mode_a) {
        case SIDE_WAVE:
            side_wave_mode_show();
            break;
        case SIDE_NEW:
            side_new_mode_show();
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
    }

    bat_led_show();
    sw_led_show();
    sys_sw_led_show();
    sys_led_show();
    rf_led_show();
}
