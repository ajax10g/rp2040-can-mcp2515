#include "mcp23017_interface.h"
#include "mcp23017.h"

extern "C"{
    Mcp23017Handle create_mcp23017(i2c_inst_t* i2c_bus, uint8_t i2c_address){ return new Mcp23017(i2c_bus, i2c_address); }
    void free_mcp23017(Mcp23017Handle p){ delete p; }
    int mcp23017_setup(Mcp23017Handle p, bool mirroring, bool polarity){ return p->setup(mirroring, polarity); }
    int mcp23017_set_io_direction(Mcp23017Handle p, int direction){ return p->set_io_direction(direction); }
    int mcp23017_set_all_output_bits(Mcp23017Handle p, int all_bits){ return p->set_all_output_bits(all_bits); }
    void mcp23017_set_output_bit_for_pin(Mcp23017Handle p, int pin, bool set){ return p->set_output_bit_for_pin(pin, set); }
    int mcp23017_flush_output(Mcp23017Handle p){ return p->flush_output(); }

    int mcp23017_update_and_get_input_values(Mcp23017Handle p){ return p->update_and_get_input_values(); }
    bool mcp23017_get_last_input_pin_value(Mcp23017Handle p, int pin){ return p->get_last_input_pin_value(pin); }
    uint16_t mcp23017_get_last_input_pin_values(Mcp23017Handle p){ return p->get_last_input_pin_values(); }
    int mcp23017_set_pullup(Mcp23017Handle p, int direction){ return p->set_pullup(direction); }
}
