SRC += user_kb.c
SRC += rf.c
SRC += side.c

SRC += mcu_pwr.c sleep.c debounce.c rf_driver.c

UART_DRIVER_REQUIRED = yes

OPT ?= 2
CUSTOM_MATRIX = lite
SRC += matrix.c
