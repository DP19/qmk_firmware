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


extern user_config_t user_config;

enum via_indicator_value {
    id_side_mode             = 0,
    id_side_light_brightness = 1,
    id_side_light_speed      = 2,
    id_side_light_color      = 3,
    id_logo_mode             = 4,
    id_logo_light_brightness = 5,
    id_logo_light_speed      = 6,
    id_logo_light_color      = 7,
    // Custom
    id_debounce              = 10,
    id_light_sleep_delay     = 11,
    id_deep_sleep_delay      = 12,
    id_sleep_toggle          = 13,
};

void via_custom_value_command_kb(uint8_t *data, uint8_t length);
