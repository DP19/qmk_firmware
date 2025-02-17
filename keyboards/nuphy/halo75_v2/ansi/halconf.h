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
#pragma once

#undef HAL_USE_SERIAL
#define HAL_USE_SERIAL TRUE
// force enable timer usage for wait_us
#undef HAL_USE_GPT
#define HAL_USE_GPT TRUE

#undef HAL_USE_I2C
#define HAL_USE_I2C TRUE

#undef HAL_USE_DMA
#define HAL_USE_DMA TRUE

#include_next <halconf.h>
