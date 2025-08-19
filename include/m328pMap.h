#ifndef M328PMAP_H
#define M328PMAP_H

// This header maps symbolic port/pin names (e.g., PIN_PB2) to Arduino digital pin numbers
// for ATmega328P (Uno/Nano style).
// Reference: Arduino Uno pin mapping
// https://i.sstatic.net/CVRUw.png

// PORT B
#ifndef PIN_PB0
#define PIN_PB0 8
#endif
#ifndef PIN_PB1
#define PIN_PB1 9
#endif
#ifndef PIN_PB2
#define PIN_PB2 10
#endif
#ifndef PIN_PB3
#define PIN_PB3 11
#endif
#ifndef PIN_PB4
#define PIN_PB4 12
#endif
#ifndef PIN_PB5
#define PIN_PB5 13
#endif
/* don't use
#ifndef PIN_PB6
#define PIN_PB6 
#endif
#ifndef PIN_PB7
#define PIN_PB7 
#endif
*/

// PORT C (Analog pins)
#ifndef PIN_PC0
#define PIN_PC0 A0
#endif
#ifndef PIN_PC1
#define PIN_PC1 A1
#endif
#ifndef PIN_PC2
#define PIN_PC2 A2
#endif
#ifndef PIN_PC3
#define PIN_PC3 A3
#endif
#ifndef PIN_PC4
#define PIN_PC4 A4
#endif
#ifndef PIN_PC5
#define PIN_PC5 A5
#endif

// PORT D
#ifndef PIN_PD0
#define PIN_PD0 0
#endif
#ifndef PIN_PD1
#define PIN_PD1 1
#endif
#ifndef PIN_PD2
#define PIN_PD2 2
#endif
#ifndef PIN_PD3
#define PIN_PD3 3
#endif
#ifndef PIN_PD4
#define PIN_PD4 4
#endif
#ifndef PIN_PD5
#define PIN_PD5 5
#endif
#ifndef PIN_PD6
#define PIN_PD6 6
#endif
#ifndef PIN_PD7
#define PIN_PD7 7
#endif

#endif // M328PMAP_H