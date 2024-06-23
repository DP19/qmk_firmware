VIA_ENABLE = yes
UART_DRIVER_REQUIRED = yes

SRC += user_kb.c
SRC += rf.c
SRC += side.c
SRC += via_kb.c
SRC += mcu_pwr.c sleep.c debounce.c rf_driver.c

OPT ?= 2
CUSTOM_MATRIX = lite
SRC += matrix.c
