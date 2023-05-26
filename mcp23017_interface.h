#pragma once
#include "hardware/i2c.h"

#ifdef __cplusplus
extern "C"{
#endif
struct Mcp23017;
typedef struct Mcp23017* Mcp23017Handle;

Mcp23017Handle create_mcp23017(i2c_inst_t* i2c_bus, uint8_t i2c_address);
void free_mcp23017(Mcp23017Handle);

int mcp23017_setup(Mcp23017Handle, bool, bool);
int mcp23017_set_io_direction(Mcp23017Handle, int direction);

int mcp23017_set_all_output_bits(Mcp23017Handle p, int all_bits);
void mcp23017_set_output_bit_for_pin(Mcp23017Handle p, int pin, bool set);
int mcp23017_flush_output(Mcp23017Handle p);

int mcp23017_update_and_get_input_values(Mcp23017Handle p);
bool mcp23017_get_last_input_pin_value(Mcp23017Handle p, int pin);
uint16_t mcp23017_get_last_input_pin_values(Mcp23017Handle p);
int mcp23017_set_pullup(Mcp23017Handle p, int direction);

#ifdef __cplusplus
}
#endif

