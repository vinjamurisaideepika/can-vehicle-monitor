/**
 * @file    can_driver.h
 * @brief   Interrupt-driven bxCAN driver (CAN1, PA11 RX / PA12 TX).
 *
 * - Received frames are copied by the RX FIFO0 interrupt into a software
 *   queue; the main loop pops them with can_drv_receive().
 * - Error interrupts (warning, passive, bus-off, protocol errors) are
 *   counted and exposed as flags for the node state machine.
 */
#ifndef APP_CAN_DRIVER_H
#define APP_CAN_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint32_t timestamp_ms;
} can_frame_t;

typedef enum {
    CAN_BUS_ERROR_ACTIVE = 0, /* normal                         */
    CAN_BUS_ERROR_WARNING,    /* TEC or REC >= 96               */
    CAN_BUS_ERROR_PASSIVE,    /* TEC or REC >= 128              */
    CAN_BUS_OFF               /* TEC > 255: node left the bus   */
} can_bus_state_t;

typedef struct {
    uint32_t tx_ok;
    uint32_t tx_fail;      /* no free mailbox / HAL error   */
    uint32_t rx_ok;
    uint32_t rx_overflow;  /* software queue was full       */
    uint32_t err_irq;      /* error interrupts seen         */
    uint32_t bus_off_events;
    uint8_t  tec;          /* transmit error counter        */
    uint8_t  rec;          /* receive error counter         */
    uint8_t  last_error;   /* LEC: 0 none,1 stuff,2 form,3 ack,4 bit1,5 bit0,6 crc */
} can_stats_t;

HAL_StatusTypeDef can_drv_init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef can_drv_send(uint32_t std_id, const uint8_t *data, uint8_t dlc);
bool              can_drv_receive(can_frame_t *out);
can_bus_state_t   can_drv_bus_state(void);
void              can_drv_get_stats(can_stats_t *out);

/** Re-initialise and restart the controller after bus-off. */
HAL_StatusTypeDef can_drv_recover(void);

const char *can_drv_lec_name(uint8_t lec);

#endif /* APP_CAN_DRIVER_H */
