#pragma once
#include "hardware/i2c.h"
#include "hardware/pio.h"

extern uint32_t ERR_LED;
bool i2c_probe(i2c_inst_t *i2c_bus, uint8_t i2c_address);
void put_pixel(uint32_t pixel_grb);
void blink_pixel(uint32_t pixel_grb, uint32_t duration_ms);
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);
