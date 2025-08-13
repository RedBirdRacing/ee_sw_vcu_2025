#ifndef M328PMAP_H
#define M328PMAP_H

// This header maps symbolic port/pin names (e.g., PIN_PB2) to Arduino digital pin numbers
// for ATmega328P (Uno/Nano style).
// Reference: https://github.com/MCUdude/MiniCore#pinout and Arduino Uno pin mapping

// PORT B
#ifndef PIN_PB0
#define PIN_PB0 12
#endif
#ifndef PIN_PB1
#define PIN_PB1 13
#endif
#ifndef PIN_PB2
#define PIN_PB2 14
#endif
#ifndef PIN_PB3
#define PIN_PB3 15
#endif
#ifndef PIN_PB4
#define PIN_PB4 16
#endif
#ifndef PIN_PB5
#define PIN_PB5 17
#endif
#ifndef PIN_PB6
#define PIN_PB6 7
#endif
#ifndef PIN_PB7
#define PIN_PB7 8
#endif

// PORT C (Analog pins)
#ifndef PIN_PC0
#define PIN_PC0 23
#endif
#ifndef PIN_PC1
#define PIN_PC1 24
#endif
#ifndef PIN_PC2
#define PIN_PC2 25
#endif
#ifndef PIN_PC3
#define PIN_PC3 26
#endif
#ifndef PIN_PC4
#define PIN_PC4 27
#endif
#ifndef PIN_PC5
#define PIN_PC5 28
#endif
#ifndef PIN_PC6
#define PIN_PC6 29
#endif

// PORT D
#ifndef PIN_PD0
#define PIN_PD0 30
#endif
#ifndef PIN_PD1
#define PIN_PD1 31
#endif
#ifndef PIN_PD2
#define PIN_PD2 32
#endif
#ifndef PIN_PD3
#define PIN_PD3 1
#endif
#ifndef PIN_PD4
#define PIN_PD4 2
#endif
#ifndef PIN_PD5
#define PIN_PD5 9
#endif
#ifndef PIN_PD6
#define PIN_PD6 10
#endif
#ifndef PIN_PD7
#define PIN_PD7 11
#endif

#endif