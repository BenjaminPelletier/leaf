#ifndef PageThermal_h
#define PageThermal_h

#include <Arduino.h>



// draw the pixels to the display
void thermalPage_draw(void);

// handle butotn presses relative to what's shown on the display
void thermalPage_button(uint8_t button, uint8_t state, uint8_t count);


#endif