#ifndef MAD_CIRCUITS_DEVICE_ESC_CFG_H
#define MAD_CIRCUITS_DEVICE_ESC_CFG_H

#include "../../../inc/driver/driver_asclin/driver_asclin_asc.h"
#include "../../../inc/driver/driver_gtm/driver_gtm_atom_pwm.h"

#define DEVICE_ESC_UART_TXFIFO_SIZE 512u
#define DEVICE_ESC_UART_RXFIFO_SIZE 64u
#define DEVICE_ESC_UART_FIFO_OVERHEAD (sizeof(Ifx_Fifo) + 8u)
#define DEVICE_ESC_UART_TXFIFO_BUFFER_BYTE (DEVICE_ESC_UART_TXFIFO_SIZE + DEVICE_ESC_UART_FIFO_OVERHEAD)
#define DEVICE_ESC_UART_RXFIFO_BUFFER_BYTE (DEVICE_ESC_UART_RXFIFO_SIZE + DEVICE_ESC_UART_FIFO_OVERHEAD)

typedef enum
{
    DEVICE_ESC_1 = 0,
    DEVICE_ESC_2 = 1,
    DEVICE_ESC_3 = 2,
    DEVICE_ESC_COUNT = 3,
}device_esc_id_t;

typedef struct
{
    device_esc_id_t esc_id;
    uint32 min_duty_cycle;
    uint32 max_duty_cycle;
    driver_gtm_atom_pwm_cfg_t atom_pwm_cfg;
}device_esc_cfg_t;

typedef struct
{
    device_esc_id_t esc_id;
    driver_gtm_atom_pwm_runtime_t atom_pwm_runtime;
    void (*device_esc_callback) (void);
}device_esc_runtime_t;

typedef struct
{
    driver_asclin_asc_cfg_t asclin_asc_cfg;
} device_esc_uart_cfg_t;

typedef struct
{
    driver_asclin_asc_runtime_t asclin_asc_runtime;
    uint8 txFifo_Buffer[DEVICE_ESC_UART_TXFIFO_BUFFER_BYTE];
    uint8 rxFifo_Buffer[DEVICE_ESC_UART_RXFIFO_BUFFER_BYTE];
} device_esc_uart_runtime_t;

device_esc_cfg_t* device_esc_cfg_table_get(void);
device_esc_runtime_t* device_esc_runtime_table_get(void);
device_esc_uart_cfg_t* device_esc_uart_cfg_get(void);
device_esc_uart_runtime_t* device_esc_uart_runtime_get(void);

#endif
