/*
Copyright 2024 DP19

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
#include "via_kb.h"

//https://www.caniusevia.com/docs/custom_ui#command-handlers
void via_config_set_value(uint8_t *data) {
    // data = [ value_id, value_data ]

    uint8_t *value_id   = &(data[0]);
    uint8_t *value_data = &(data[1]);

    switch (*value_id) {
        case id_side_mode_a:
            user_config.side_mode_a = *value_data;
            break;
        case id_side_mode_b:
            user_config.side_mode_b = *value_data;
            break;
        case id_side_light_brightness:
            user_config.side_light = *value_data;
            break;
        case id_side_light_speed:
            user_config.side_speed = *value_data;
            break;
        case id_side_light_color:
            user_config.side_colour = *value_data;
            break;
        case id_debounce:
            user_config.debounce = *value_data;
            break;
        case id_light_sleep_delay:
            user_config.light_sleep_delay = *value_data + 1; // Options are indexed at 0
            break;
        case id_deep_sleep_delay:
            user_config.deep_sleep_delay = *value_data + 1; // Options are indexed at 0
            break;
        case id_power_on_animation:
            user_config.power_on_show = *value_data;
            break;
        case id_sleep_toggle:
            user_config.sleep_enable = *value_data;
            break;
    }
}

//https://www.caniusevia.com/docs/custom_ui#command-handlers
void via_config_get_value(uint8_t *data) {
    uint8_t *value_id   = &(data[0]);
    uint8_t *value_data = &(data[1]);
    switch (*value_id) {
        case id_side_mode_a:
            *value_data = user_config.side_mode_a;
            break;
        case id_side_mode_b:
            *value_data = user_config.side_mode_b;
            break;
        case id_side_light_brightness:
            *value_data = user_config.side_light;
            break;
        case id_side_light_speed:
            *value_data = user_config.side_speed;
            break;
        case id_side_light_color:
            *value_data = user_config.side_colour;
            break;
        case id_debounce:
            *value_data = user_config.debounce;
            break;
        case id_light_sleep_delay:
            *value_data = user_config.light_sleep_delay - 1;
            break;
        case id_deep_sleep_delay:
            *value_data = user_config.deep_sleep_delay - 1;
            break;
        case id_power_on_animation:
            *value_data = user_config.power_on_show;
            break;
        case id_sleep_toggle:
            *value_data = user_config.sleep_enable;
            break;
    }
}


// https://www.caniusevia.com/docs/custom_ui#command-handlers
void via_custom_value_command_kb(uint8_t *data, uint8_t length) {
    // data = [ command_id, channel_id, value_id, value_data ]
    uint8_t *command_id = &(data[0]);
    uint8_t *channel_id = &(data[1]);

    uint8_t *value_id_and_data = &(data[2]);

    if (*channel_id == id_custom_channel) {
        switch (*command_id)

        {
            case id_custom_set_value: {
                via_config_set_value(value_id_and_data);
                break;
            }
            case id_custom_get_value: {
                via_config_get_value(value_id_and_data);
                break;
            }
            case id_custom_save: {
                eeconfig_update_kb_datablock(&user_config);
                break;
            }
            default: {
                // Unhandled message.
                *command_id = id_unhandled;
                break;
            }
        }
        return;
    }

    // Return the unhandled state
    *command_id = id_unhandled;

    // DO NOT call raw_hid_send(data,length) here, let caller do this
}
