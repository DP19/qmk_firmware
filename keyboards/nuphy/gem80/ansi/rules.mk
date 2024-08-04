UART_DRIVER_REQUIRED = yes

SRC += user_kb.c
SRC += rf.c
SRC += side.c side_driver.c side_logo.c
SRC += via_kb.c
SRC += mcu_pwr.c sleep.c debounce.c rf_driver.c rf_queue.c

OPT ?= 2
CUSTOM_MATRIX = lite
SRC += matrix.c
