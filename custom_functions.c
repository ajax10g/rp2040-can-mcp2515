#include "custom_functions.h"

bool i2c_probe(i2c_inst_t *i2c_bus, uint8_t i2c_address){
    int result;
    uint8_t rxdata;

    // Perform a 1-byte dummy read from the probe address. If a slave
    // acknowledges this address, the function returns the number of bytes
    // transferred. If the address byte is ignored, the function returns
    // -1.
    result = i2c_read_blocking(i2c_bus, i2c_address, &rxdata, 1, false);

    return(result>0);
}

/*static inline void put_pixel(uint32_t pixel_grb) {*/
void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

void blink_pixel(uint32_t pixel_grb, uint32_t duration_ms){
    put_pixel(pixel_grb);
    sleep_ms(duration_ms);
    put_pixel(urgb_u32(ERR_LED, 0, 0)); // OFF or Red if error
    sleep_ms(duration_ms);
}

uint32_t urgb_u32(uint8_t r/*Unused*/, uint8_t g, uint8_t b) {
    return
            /*((uint32_t) (r) << 8) |*/
            ((uint32_t) (ERR_LED) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}
