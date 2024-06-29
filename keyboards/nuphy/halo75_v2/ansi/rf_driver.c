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

#include "host_driver.h"
#include "host.h"
#include "user_kb.h"
#include "rf_queue.h"

/* Variable declaration */
extern DEV_INFO_STRUCT dev_info;
extern report_buffer_t report_buff_a;
extern report_buffer_t report_buff_b;
extern rf_queue_t      rf_queue;

/* Host driver */
static uint8_t rf_keyboard_leds(void);
static void    rf_send_keyboard(report_keyboard_t *report);
static void    rf_send_nkro(report_nkro_t *report);
static void    rf_send_mouse(report_mouse_t *report);
static void    rf_send_extra(report_extra_t *report);

const host_driver_t rf_host_driver = {rf_keyboard_leds, rf_send_keyboard, rf_send_nkro, rf_send_mouse, rf_send_extra};

/* defined in rf.c */
void clear_report_buffer(void);
void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size);

/**
 * @brief Send or queue the RF report.
 *
 */
static void send_or_queue(report_buffer_t *report) {
    if (dev_info.rf_state == RF_CONNECT && rf_queue.is_empty()) {
        uart_send_report(report->cmd, report->buffer, report->length);
        report->repeat++;
    } else {
        rf_queue.enqueue(report);
    }
}

static report_buffer_t make_report_buffer(uint8_t cmd, uint8_t *buff, uint8_t len) {
    report_buffer_t report = {.cmd = cmd, .length = len};
    memcpy(report.buffer, buff, len);
    return report;
}

static uint8_t rf_keyboard_leds(void) {
    return dev_info.rf_led;
}

static void rf_send_keyboard(report_keyboard_t *report) {
    clear_report_buffer();
    report->reserved    = 0;
    report_buffer_t rpt = make_report_buffer(CMD_RPT_BYTE_KB, &report->mods, 8);
    send_or_queue(&rpt);
    report_buff_a = rpt;
}

static void rf_send_nkro(report_nkro_t *report) {
#ifdef NKRO_ENABLE
    clear_report_buffer();
    uart_auto_nkey_send(&nkro_report->mods, 16); // only need 1 byte mod + 15 byte keys
#endif                                           // NKRO_ENABLE
}

static void rf_send_mouse(report_mouse_t *report) {
#ifdef MOUSEKEY_ENABLE
    clear_report_buffer();
    report_buffer_t rpt = make_report_buffer(CMD_RPT_MS, &report->buttons, 5);
    send_or_queue(&rpt);
#endif // MOUSEKEY_ENABLE
}

static void rf_send_extra_helper(uint8_t cmd, report_extra_t *report) {
    clear_report_buffer();
    report_buffer_t rpt = make_report_buffer(cmd, (uint8_t *)(&report->usage), 2);
    send_or_queue(&rpt);
}

static void rf_send_extra(report_extra_t *report) {
    if (report->report_id == REPORT_ID_CONSUMER) {
        rf_send_extra_helper(CMD_RPT_CONSUME, report);
    } else if (report->report_id == REPORT_ID_SYSTEM) {
        rf_send_extra_helper(CMD_RPT_SYS, report);
    }
}
