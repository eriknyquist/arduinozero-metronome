/*
 * Arduino Zero Stage Metronome
 * Erik K. Nyquist 2023
 *
 * Configuration parameters
 */
#ifndef METRONOME_CONFIG_H
#define METRONOME_CONFIG_H


// If 0, no logging messages will be sent to the UART
#define ENABLE_UART_LOGGING      (1u)

// If 0, CLI interface will not be available
#define ENABLE_UART_CLI          (1u)

// Time between value increments when holding a button down, in milliseconds
#define BUTTON_HOLD_CHANGE_MS    (500u)

// When holding down a button, the value increment will be doubled after this many #BUTTON_HOLD_CHANGE_MS periods
#define BUTTON_HOLD_DOUBLE_COUNT (4u)

// When holding down a button, the value increment will be doubled no more than this many times
#define BUTTON_HOLD_MAX_DOUBLES  (4u)

// Button debounce time in milliseconds
#define BUTTON_DEBOUNCE_MS       (100u)


// GPIO pin numbers for 20x4 character LCD (0, 1 and 9 are needed by I2S library).
// 13 is the builtin LED, which is used to flash in sync with the click sound)
#define LCD_RS_PIN               (A5)
#define LCD_EN_PIN               (A4)
#define LCD_D4_PIN               (A3)
#define LCD_D5_PIN               (A2)
#define LCD_D6_PIN               (A1)
#define LCD_D7_PIN               (A0)

// GPIO pin numbers for buttons
#define UP_BUTTON_PIN            (2)
#define DOWN_BUTTON_PIN          (3)
#define LEFT_BUTTON_PIN          (5)
#define RIGHT_BUTTON_PIN         (6)
#define SELECT_BUTTON_PIN        (7)
#define MODE_BUTTON_PIN          (8)
#define VOLUP_BUTTON_PIN         (10)
#define VOLDOWN_BUTTON_PIN       (11)
#define ADD_DEL_BUTTON_PIN       (12)

// GPIO pin for on/off switch
#define ON_OFF_SWITCH_PIN        (13)

#endif // METRONOME_CONFIG_H
