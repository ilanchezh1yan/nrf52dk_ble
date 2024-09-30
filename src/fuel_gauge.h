#ifndef FUEL_GAUGE_H
#define FUEL_GAUGE_H

#include <stdint.h>
#include "nrf_twi.h"
#include "nrf_drv_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

struct Registers {
      uint16_t RepCap;
      uint16_t FullCapRep;
      uint8_t Age;
      uint16_t Cycles;
      uint16_t TimerH;
      uint16_t Rcell;
}PACKED;

struct fuel_gauge {
    uint16_t header;
    uint8_t length;
    struct Registers FG;
    uint8_t CRC;
}PACKED;

#ifdef F_G_
struct Write_chunk {
      uint8_t address;
      uint16_t data;
}PACKED;

static uint8_t chksum8(const unsigned char *, size_t);
static uint16_t Master_read(uint8_t);
static void Master_write(struct Write_chunk);
static void FG_Reg_write(uint8_t, uint16_t);
#endif

void twi_init (void);
void battery_status(void);
void battery_profile(void *);
void configure_battery(void);
#endif