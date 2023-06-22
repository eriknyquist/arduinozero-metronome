/*
 * Advanced stage metronome for Arduino Zero
 *
 * Erik K. Nyquist 2023
 *
 * Hardware:
 *
 * - Arduino zero
 *
 * - 1 toggle switch for power
 *
 * - 7 push buttons for setting BPM/beat, and for creating/naming/deleting presets
 *
 * - 20x4 character LCD
 *
 * - HiLetgo PCM5102 I2S DAC (includes 3.5mm audio jack)
 *
 *
 * Features:
 *
 * - Available BPM values are 1 through 512. Dedicated buttons allow incrementing /
 *   decrementing the current BPM value.
 *
 * - Available beat count values are 1 through 16. Dedicated buttons allow
 *   incrementing / decrementing the current beat count value.
 *
 * - Up to 128 presets can be saved. A single preset consists of BPM, beat count,
 *   a name entered by the user. Saved presets are stored in MCU internal flash.
 *
 * - A "preset mode" enables setting the metronome BPM & beat count based on saved
 *   presets. Dedicated buttons allow switching to the next/previous preset, in the
 *   order that the presets were originally saved.
 *
 * - In "preset mode", if the metronome is running, switching to the next/previous
 *   preset will be seamless with respect to the current BPM & beat count; instead
 *   of changing immediately when the button is pressed, the preset will only be
 *   changed on the first beat of the next bar (i.e. the next beat #1).
 *
 * - Saved presets can be deleted & edited
 *
 * - Metronome beats are triggered by timer interrupts from TC4. Audio samples
 *   for metronome beats are streamed to the I2S DAC directly in the TC4 interrupt
 *   handler, using only the non-blocking versions of the I2S library functions.
 *   Everything else (UART reads/writes, character LCD writes, button state
 *   processing) is done in the main context inside the loop() function. The result
 *   of all this is that the timing of metronome beats is very reliable (or at
 *   least, as reliable as they can be, given the reliablility of the system core
 *   clock), since anything else we happen to be doing can always be interrupted
 *   to serve up another chunk of audio samples to the I2S DAC.
 *
 * - UART-based command line interface allows full command/control of the metronome
 *   user interface by interacting with the serial port (this is just extra-- all
 *   features, including creating/naming/editing/deleting presets, can be accessed via
 *   buttons & character LCD on the device).
 */

#include "metronome_beep_samples.h"

#include <stdarg.h>

#include <Arduino_CRC32.h>
#include <FlashStorage.h>
#include <ArduinoLowPower.h>


// Version number reported by CLI
#define METRONOME_SKETCH_VERSION "0.0.1"

// If 0, no log messages will be sent to the UART
#define ENABLE_UART_LOGGING     (1u)

// If 0, CLI interface will not be available
#define ENABLE_UART_CLI         (1u)

// Only used for debugging, useful if testing something unrelated to
// preset storage and want to avoid wasting flash write cycles
#define ENABLE_FLASH_WRITE      (1u)

// GPIO pin numbers for 20x4 character LCD (0, 1 and 9 are needed by I2S library).
// 13 is the builtin LED, which is used to show when we are streaming samples to
// the I2S DAC)
#define LCD_RS_PIN              (2)
#define LCD_EN_PIN              (3)
#define LCD_D4_PIN              (4)
#define LCD_D5_PIN              (5)
#define LCD_D6_PIN              (6)
#define LCD_D7_PIN              (7)

// GPIO pin numbers for buttons
#define UP_BUTTON_PIN           (A0)
#define DOWN_BUTTON_PIN         (A1)
#define LEFT_BUTTON_PIN         (A2)
#define RIGHT_BUTTON_PIN        (A3)
#define SELECT_BUTTON_PIN       (A4)
#define MODE_BUTTON_PIN         (A5)
#define ADD_DEL_BUTTON_PIN      (A5)


#define MAX_PRESET_NAME_LEN     (32u)   // Max. number of characters for a preset name string
#define MAX_PRESET_COUNT        (128u)  // Max. number of presets that can be saved in flash
#define BUTTON_DEBOUNCE_MS      (100u)  // Button debounce time, milliseconds
#define MIN_BPM                 (1u)    // Min. allowed BPM value
#define MAX_BPM                 (512u)  // Max. allowed BPM value
#define MIN_BEAT                (1u)    // Min. allowed beat value
#define MAX_BEAT                (16u)   // Max. allowed beat value

// Actual system core clock freq for Arduino Zero
// (taken from https://github.com/manitou48/crystals/blob/master/crystals.txt)
#define TRUE_CORE_CLOCK_HZ      (47972352UL)


// Checks if character is space or tab
#define IS_SPACE(c) ((c == ' ') || (c == '\t'))


// Enumerates all states the metronome can be in, button inputs
// mean different things for each of these states.
typedef enum
{
    STATE_METRONOME,                // Standard metronome screen
    STATE_PRESET_PLAYBACK,          // Preset playback screen
    STATE_PRESET_EDIT,              // Preset edit screen
    STATE_PRESET_EDIT_DELETE_MENU,  // Menu showing edit/delete options for current preset
    STATE_PRESET_DELETE_CHECK,      // Menu showing "are you sure? yes/no" for preset deletion
    STATE_PRESET_NAME_ENTRY,        // Preset name entry screen
    STATE_COUNT
} metronome_state_e;

// Enumerates all buttons
typedef enum
{
    BUTTON_UP = 0,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_SELECT,
    BUTTON_MODE,
    BUTTON_ADD_DELETE,
    BUTTON_COUNT
} button_e;

// Holds all data for a single metronome preset
typedef struct
{
    // Bits 0 through 9: BPM, 0-511 representing 1-512BPM
    // Bits 9 through 12: beat count, 0-15 representing 1-16 beats
    // Bits 13 through 15: reserved
    uint16_t settings;

    // Preset name, null-terminated
    char name[MAX_PRESET_NAME_LEN];
} metronome_preset_t;

// Holds all data for stored metronome presets
typedef struct
{
    // CRC of stored data
    uint32_t crc;

    // Number of presets currently populated
    uint16_t preset_count;

    // Array of stored preset data
    metronome_preset_t presets[MAX_PRESET_COUNT];
} metronome_presets_t;

// Enumerates all button debounce states
typedef enum
{
    DEBOUNCE_IDLE = 0,  // No change detected
    DEBOUNCE_START,     // Change detected, debounce not started
    DEBOUNCE_ACTIVE,    // Debounce in progress
    DEBOUNCE_FORCE      // Force simulated press from CLI command
} debounce_state_e;

// Holds all data required to track the state of a single button
typedef struct
{
    bool pressed;            // True if debounce determined button was pressed
    int pressed_state;      // GPIO value that represents a pressed button
    debounce_state_e state;  // Current debounce state
    unsigned long start_ms;  // Debounce start time, milliseconds
    void (*callback)(void);  // Interrupt handler
    int gpio_pin;            // GPIO pin number for button
} button_info_t;

#if ENABLE_UART_CLI
// Holds all data required to handle a CLI command
typedef struct
{
    const char *cmd_word;
    void (*cmd_handler)(char *cmd_args);
} cli_command_t;

// Forward declaration of CLI command handlers
static void _presets_cmd_handler(char *cmd_args);
static void _poweroff_cmd_handler(char *cmd_args);
static void _help_cmd_handler(char *cmd_args);
static void _up_cmd_handler(char *cmd_args);
static void _down_cmd_handler(char *cmd_args);
static void _left_cmd_handler(char *cmd_args);
static void _right_cmd_handler(char *cmd_args);
static void _select_cmd_handler(char *cmd_args);
static void _mode_cmd_handler(char *cmd_args);
static void _add_del_cmd_handler(char *cmd_args);


// Number of CLI commands we can handle (must be manually synced with _cli_commands)
#define CLI_COMMAND_COUNT (10u)

// Table mapping CLI command words to command handlers
static cli_command_t _cli_commands[CLI_COMMAND_COUNT] =
{
    {"help", _help_cmd_handler},
    {"presets", _presets_cmd_handler},
    {"off", _poweroff_cmd_handler},
    {"u", _up_cmd_handler},
    {"d", _down_cmd_handler},
    {"l", _left_cmd_handler},
    {"r", _right_cmd_handler},
    {"s", _select_cmd_handler},
    {"m", _mode_cmd_handler},
    {"a", _add_del_cmd_handler}
};

// Serial input buffer for CLI commands
static char _cli_buf[64u];
static unsigned int _cli_buf_pos = 0u;
#endif // ENABLE_UART_CLI


// RAM copy of saved preset data
static metronome_presets_t _presets;


// GPIO interrupt callbacks for buttons
void _up_button_callback(void) { _gpio_callback(BUTTON_UP); }
void _down_button_callback(void) { _gpio_callback(BUTTON_DOWN); }
void _left_button_callback(void) { _gpio_callback(BUTTON_LEFT); }
void _right_button_callback(void) { _gpio_callback(BUTTON_RIGHT); }
void _select_button_callback(void) { _gpio_callback(BUTTON_SELECT); }
void _mode_button_callback(void) { _gpio_callback(BUTTON_MODE); }
void _add_delete_button_callback(void) { _gpio_callback(BUTTON_ADD_DELETE); }


// State tracking/debouncing for all buttons
static volatile button_info_t _buttons[BUTTON_COUNT] =
{
    {false, LOW, DEBOUNCE_IDLE, 0u, _up_button_callback, UP_BUTTON_PIN},              // BUTTON_UP
    {false, LOW, DEBOUNCE_IDLE, 0u, _down_button_callback, DOWN_BUTTON_PIN},          // BUTTON_DOWN
    {false, LOW, DEBOUNCE_IDLE, 0u, _left_button_callback, LEFT_BUTTON_PIN},          // BUTTON_LEFT
    {false, LOW, DEBOUNCE_IDLE, 0u, _right_button_callback, RIGHT_BUTTON_PIN},        // BUTTON_RIGHT
    {false, LOW, DEBOUNCE_IDLE, 0u, _select_button_callback,  SELECT_BUTTON_PIN},     // BUTTON_SELECT
    {false, LOW, DEBOUNCE_IDLE, 0u, _mode_button_callback, MODE_BUTTON_PIN},          // BUTTON_MODE
    {false, LOW, DEBOUNCE_IDLE, 0u, _add_delete_button_callback, ADD_DEL_BUTTON_PIN}  // BUTTON_ADD_DELETE
};

// Runtime values for BPM, beat count, metronome mode, and preset index
static volatile uint16_t _current_bpm = 123u;
static volatile uint16_t _current_beat_count = 4u;
static volatile metronome_state_e _current_state = STATE_METRONOME;
static volatile uint16_t _current_preset_index = 0u;

// Stores the metronome BPM on metronome state exit, so we can
// restore it when we return to the metronome state
static volatile uint16_t _saved_metronome_bpm = _current_bpm;

// Tracks whether metronome is running (in metronome mode)
static volatile bool _metronome_running = false;

// Tracks current beat in bar
static volatile uint16_t _current_beat = 0u;


// Tracks requested preset index (we only load in a new preset before the first beat of the bar)
static volatile uint16_t _requested_preset_index = 0u;
static volatile bool _preset_change_requested = false;
static volatile bool _preset_change_complete = false;

// Holds the CRC value for presets loaded from flash on boot
static uint32_t _preset_crc_on_boot = 0u;

// Table of alphanumeric characters, used for preset name entry. '<' represents
// a backspace, '_' represents a space character, and '*' represents a 'done/save' button.
// All of those symbols may be drawn differently on the character LCD.
#define ALPHANUM_ROWS (3u)
#define ALPHANUM_COLS (20u)
static char _alphanum_table[ALPHANUM_ROWS][ALPHANUM_COLS] =
{
    {' ', ' ', 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', ' ', '1', '2', '3', '4', ' ', ' ', ' '},
    {' ', ' ', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', '_', ' ', '5', '6', '7', '8', ' ', ' ', ' '},
    {' ', ' ', ' ', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ' ', '<', ' ', ' ', '9', '0', ' ', ' ', '*', ' '},
};

// Tracks cursor position within aplhanum table, for preset name entry
static unsigned int _alphanum_col = 0u;
static unsigned int _alphanum_row = 0u;

// Buffer to hold preset name during preset name entry
static char _preset_name_buf[MAX_PRESET_NAME_LEN];
static unsigned int _preset_name_pos = 0u;


#if ENABLE_UART_LOGGING
#define LOG_INFO(fmt, ...) log("INFO", __func__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) log("ERROR", __func__, __LINE__, fmt, ##__VA_ARGS__)

// Formats and prints a debug message to the UART
static void log(const char *level, const char *func, int line, char *fmt, ...)
{
    char msg_buf[256u];
    float time_s = ((float) millis()) / 1000.0f;
    int printed = snprintf(msg_buf, sizeof(msg_buf), "[%.3f][%s:%d][%s] ",
                           time_s, func, line, level);

    if (sizeof(msg_buf) > (unsigned) printed)
    {
        va_list arg_ptr;
        va_start(arg_ptr, fmt);
        (void) vsnprintf(msg_buf + printed, sizeof(msg_buf) - printed, fmt, arg_ptr);
        va_end(arg_ptr);
    }

    Serial.println(msg_buf);
}
#else
#define LOG_INFO(fmt, ...) {}
#define LOG_ERROR(fmt, ...) {}
#endif // ENABLE_UART_LOGGING

// CRC generator, used to generate CRCs for preset data in flash
Arduino_CRC32 crc_generator;

// Flash storage object for preset saving
FlashStorage(preset_store, metronome_presets_t);


// Disable all interrupts, set all pins to inputs, and save presets to flash
static void _power_off(void)
{
    LOG_INFO("powering off");

    // Stop metronome timer, & disable TC4 interrupt request
    _stop_metronome();
    TC4->COUNT32.INTENSET.bit.MC0 = 0;
    while (_tc4_syncing());

    // Set builtin LED pin to floating input
    pinMode(13, INPUT);

    // Disable all GPIO interrupts and configure pins as floating inputs
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        pinMode(_buttons[i].gpio_pin, INPUT);
        detachInterrupt(digitalPinToInterrupt(_buttons[i].gpio_pin));
    }

#if ENABLE_FLASH_WRITE
    uint32_t new_crc = _calc_preset_crc();
    if (_preset_crc_on_boot != new_crc)
    {
        LOG_INFO("saving presets");
        _presets.crc = new_crc;
        preset_store.write(_presets);
    }
    else
    {
        LOG_INFO("no change to presets");
    }
#endif // ENABLE_FLASH_WRITE

#if ENABLE_UART_LOGGING || ENABLE_UART_CLI
    // Disable serial
    delay(500);
    Serial.end();
#endif // ENABLE_UART_LOGGING || ENABLE_UART_CLI

    // Go into low power mode forever
    LowPower.sleep();
}

// GPIO callback wrapper, initiates debounce for button pin if not already in progress
static void _gpio_callback(button_e button)
{
    if (DEBOUNCE_IDLE == _buttons[button].state)
    {
        // Disable interrupt for this pin
        detachInterrupt(digitalPinToInterrupt(_buttons[button].gpio_pin));

        // Tell debounce loop that pin state has changed
        _buttons[button].state = DEBOUNCE_START;
    }
}


#if ENABLE_UART_CLI
// 'help' CLI command handler
void _help_cmd_handler(char *cmd_args)
{
    Serial.println("-------- CLI command reference ---------");
    Serial.print("Version ");
    Serial.println(METRONOME_SKETCH_VERSION);
    Serial.println("help     - Show this printout");
    Serial.println("presets  - Show all saved presets");
    Serial.println("off      - Save presets to flash, power off device");
    Serial.println("u        - Inject UP button press");
    Serial.println("d        - Inject DOWN button press");
    Serial.println("l        - Inject LEFT button press");
    Serial.println("r        - Inject RIGHT button press");
    Serial.println("s        - Inject SELECT button press");
    Serial.println("m        - Inject MODE button press");
    Serial.println("a        - Inject ADD/DEL button press");
    Serial.println("----------------------------------------");
}

// 'dump presets' CLI command handler
static void _presets_cmd_handler(char *cmd_args)
{
    for (unsigned int i = 0u; i < _presets.preset_count; i++)
    {
        Serial.print("preset #");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(_presets.presets[i].name);
        Serial.print(" 0x");
        Serial.println(_presets.presets[i].settings, HEX);
    }
}

// 'power off' CLI command handler
static void _poweroff_cmd_handler(char *cmd_args)
{
    _power_off();
}

// Generic simulated button press CLI command handler
static void _button_cli_handler(button_e button)
{
    _buttons[button].pressed = true;
    _buttons[button].state = DEBOUNCE_FORCE;
}

// CLI command handlers for simulated button presses
static void _up_cmd_handler(char *cmd_args) { _button_cli_handler(BUTTON_UP); }
static void _down_cmd_handler(char *cmd_args) { _button_cli_handler(BUTTON_DOWN); }
static void _left_cmd_handler(char *cmd_args) { _button_cli_handler(BUTTON_LEFT); }
static void _right_cmd_handler(char *cmd_args) { _button_cli_handler(BUTTON_RIGHT); }
static void _select_cmd_handler(char *cmd_args) { _button_cli_handler(BUTTON_SELECT); }
static void _mode_cmd_handler(char *cmd_args) { _button_cli_handler(BUTTON_MODE); }
static void _add_del_cmd_handler(char *cmd_args) { _button_cli_handler(BUTTON_ADD_DELETE); }


// Read CLI commands from serial port, and run command handlers if required
static void _handle_cli_commands(void)
{
    bool line_received = false;

    while (Serial.available() > 0)
    {
        if (sizeof(_cli_buf) <= _cli_buf_pos)
        {
            LOG_ERROR("CLI overrun!");
            _cli_buf_pos = 0u;
            return;
        }

        char c = Serial.read();
        if ('\n' == c)
        {
            // Line complete, add null terminator
            _cli_buf[_cli_buf_pos] = '\0';
            line_received = true;
            break;
        }
        else
        {
            // Ignore leading spaces at beginning of line
            if (!IS_SPACE(c) || (_cli_buf_pos > 0u))
            {
                _cli_buf[_cli_buf_pos] = c;
                _cli_buf_pos += 1u;
            }
        }
    }

    if (!line_received)
    {
        // nothing to handle until a full line is received
        return;
    }

    unsigned int cmd_word_end = 0u;
    for (; cmd_word_end < sizeof(_cli_buf); cmd_word_end++)
    {
        if (IS_SPACE(_cli_buf[cmd_word_end]))
        {
            _cli_buf[cmd_word_end] = '\0';
            break;
        }
    }

    bool recognized_command = false;
    for (unsigned int i = 0u; i < CLI_COMMAND_COUNT; i++)
    {
        if (0 == strncmp(_cli_commands[i].cmd_word, _cli_buf, sizeof(_cli_buf)))
        {
            // Run handler
            recognized_command = true;
            _cli_commands[i].cmd_handler(_cli_buf + cmd_word_end + 1u);
            break;
        }
    }

    if (!recognized_command)
    {
        LOG_ERROR("unrecognized command '%s'", _cli_buf);
    }

    _cli_buf_pos = 0u;
}
#endif // ENABLE_UART_CLI

// Calculate CRC value of preset data
static uint32_t _calc_preset_crc(void)
{
    // Set CRC field to 0 for CRC calculation
    _presets.crc = 0u;
    // Calculate CRC
    return crc_generator.calc((uint8_t *) &_presets, sizeof(_presets));
}

// Save current BPM and beat count to a preset slot
static bool _save_preset(uint16_t *preset)
{
    if (MAX_PRESET_COUNT <= _presets.preset_count)
    {
        return false;
    }

    *preset = ((_current_bpm - 1u) & 0x1FFu) | (((_current_beat_count - 1u) & 0xFu) << 0x9u);
}

// Populate current BPM and beat count from a saved preset slot
static void _load_preset(uint16_t preset)
{
    _current_bpm = ((preset) & 0x1FFu) + 1u;
    _current_beat_count = (((preset) >> 9u) & 0xFu) + 1u;
}

// Delete a preset from the RAM copy, by 0-based preset index
static void _delete_preset(unsigned int index)
{
    if (0u == _presets.preset_count)
    {
        // Nothing to do
        return;
    }

    // If the last index, only need to decrement the preset count
    if (index < (_presets.preset_count - 1u))
    {
        // Calculate presets remaining after deleted preset
        unsigned int presets_remaining = (_presets.preset_count - 1u) - index;
        unsigned int bytes_remaining = sizeof(metronome_preset_t) * presets_remaining;

        // Shift remaining presets down to cover the deleted preset
        memmove(&_presets.presets[index], &_presets.presets[index + 1u], bytes_remaining);
    }

    _presets.preset_count -= 1u;

    // Check if current preset index is still valid
    if (_current_preset_index >= _presets.preset_count)
    {
        _current_preset_index = _presets.preset_count - 1u;
    }
}

// Draw current state to character LCD, based on current state
static void _update_char_lcd(void)
{
    if (STATE_METRONOME == _current_state)
    {
        // TODO
    }
    else if (STATE_PRESET_PLAYBACK == _current_state)
    {
        // TODO
    }
    else if (STATE_PRESET_NAME_ENTRY == _current_state)
    {
        // TODO
    }
    else
    {
        // Error
    }
}

// Start streaming sound for first beat of bar
static void _stream_beat_sound(void)
{
    // TODO: implement
}

// Start streaming sound for non-first beat of bar
static void _stream_subbeat_sound(void)
{
    // TODO: implement
}

// Called on TC4 interrupt, starts streaming the next beat to the I2S DAC
void _start_streaming_next_beat(void)
{
    unsigned long now = millis();

    if (1u == _current_beat)
    {
        // First beat of bar, check if preset change was requested
        if (_preset_change_requested)
        {
            _load_preset(_presets.presets[_requested_preset_index].settings);
            _tc4_set_period(_current_bpm);
            _current_preset_index = _requested_preset_index;
            _preset_change_requested = false;
            _preset_change_complete = true;
        }

        _stream_beat_sound();
    }
    else
    {
        _stream_subbeat_sound();
    }

    if (_current_beat_count == _current_beat)
    {
        _current_beat = 1u;
    }
    else
    {
        _current_beat += 1u;
    }
}

// Wait for TC4 to be not busy
bool _tc4_syncing(void)
{
    return TC4->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY;
}

// Reset (stop) TC4 timer
void _tc4_reset(void)
{
    TC4->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
    while (_tc4_syncing());
    while (TC4->COUNT32.CTRLA.bit.SWRST);
}

// Interrupt handler for TC4
void TC4_Handler(void)
{
    _start_streaming_next_beat();
    digitalWrite(13, !digitalRead(13));
    TC4->COUNT32.INTFLAG.bit.MC0 = 1; //Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}

// Start metronome with current settings (enable timer interrupt)
static void _start_metronome(void)
{
    // Start timer/counter, if runnning
    if ((TC4->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE) == 0u)
    {
        TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
        _metronome_running = true;
        LOG_INFO("starting metronome");
    }
}

// Stop metronome (disable timer interrupt)
static void _stop_metronome(void)
{
    // Start timer/counter, if runnning
    if ((TC4->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE) > 0u)
    {
        TC4->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
        _metronome_running = false;
        LOG_INFO("stopping metronome");
    }
}

// Change TC4 counter period, stops timer first if needed
void _tc4_set_period(unsigned int bpm)
{
    // Stop timer/counter, if runnning
    bool timer_was_running = false;
    if ((TC4->COUNT32.CTRLA.reg & TC_CTRLA_ENABLE) > 0u)
    {
        TC4->COUNT32.CTRLA.reg &= ~TC_CTRLA_ENABLE;
        timer_was_running = true;
    }

    // Set new period
    TC4->COUNT32.CC[0].reg = (TRUE_CORE_CLOCK_HZ * 60UL) / bpm;

    while (_tc4_syncing());

    if (timer_was_running)
    {
        // Re-start timer/counter
        TC4->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
        while (_tc4_syncing());
    }
}

// Display "no more room for presets" message and wait 2s
static void _max_preset_count_exceeded(void)
{
    LOG_ERROR("No more room for presets");
}

// Helper function to change state and log the transition
void _state_transition(metronome_state_e new_state)
{
    const char *statename = "";
    switch(new_state)
    {
        case STATE_METRONOME:
            statename = "Metronome";
            break;
        case STATE_PRESET_PLAYBACK:
            statename = "Preset playback";
            break;
        case STATE_PRESET_EDIT:
            statename = "Preset editing";
            break;
        case STATE_PRESET_DELETE_CHECK:
            statename = "Preset deletion check";
            break;
        case STATE_PRESET_EDIT_DELETE_MENU:
            statename = "Preset edit/delete menu";
            break;
        case STATE_PRESET_NAME_ENTRY:
            statename = "Preset name entry";
            break;
    }
    LOG_INFO("changing to state '%s'", statename);

    // Stop metronome timer/counter
    _stop_metronome();

    // Reset all button states
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        _buttons[i].pressed = false;
        _buttons[i].state = DEBOUNCE_IDLE;
    }

    // Update state value
    _current_state = new_state;
}

// Handle all buttons related to changing metronome BPM and beat count, and
// starting/stopping metronome. Used in the STATE_METRONOME and STATE_PRESET_EDIT states.
static bool _handle_metronome_settings_buttons(void)
{
   bool lcd_update_required = false;
    if (_buttons[BUTTON_UP].pressed)
    {
        // Increment BPM
        if (MAX_BPM > _current_bpm)
        {
            lcd_update_required = true;
            _current_bpm += 1u;
            _tc4_set_period(_current_bpm);
            LOG_INFO("%u BPM", _current_bpm);
        }

        _buttons[BUTTON_UP].pressed = false;
    }
    else if (_buttons[BUTTON_DOWN].pressed)
    {
        // Decrement BPM
        if (MIN_BPM < _current_bpm)
        {
            lcd_update_required = true;
            _current_bpm -= 1u;
            _tc4_set_period(_current_bpm);
            LOG_INFO("%u BPM", _current_bpm);
        }

        _buttons[BUTTON_DOWN].pressed = false;
    }
    else if (_buttons[BUTTON_LEFT].pressed)
    {
        // Decrement BPM
        if (MIN_BEAT == _current_beat_count)
        {
            _current_beat_count = MAX_BEAT;
        }
        else
        {
            _current_beat_count -= 1u;
        }

        LOG_INFO("%u beats", _current_beat_count);
        lcd_update_required = true;
        _buttons[BUTTON_LEFT].pressed = false;
    }
    else if (_buttons[BUTTON_RIGHT].pressed)
    {
        // Decrement BPM
        if (MAX_BEAT == _current_beat_count)
        {
            _current_beat_count = MIN_BEAT;
        }
        else
        {
            _current_beat_count += 1u;
        }

        LOG_INFO("%u beats", _current_beat_count);
        lcd_update_required = true;
        _buttons[BUTTON_RIGHT].pressed = false;
    }
    else if (_buttons[BUTTON_SELECT].pressed)
    {
        // Start/stop metronome
        if (!_metronome_running)
        {
            // Start timer counting for current BPM
            _tc4_set_period(_current_bpm);
            _start_metronome();
        }
        else
        {
            _stop_metronome();
        }

        _buttons[BUTTON_SELECT].pressed = false;
    }

    return lcd_update_required;
}

// Handle button inputs on edit saved preset screen
static bool _handle_preset_edit_inputs(void)
{
    bool lcd_update_required = false;
    if (_buttons[BUTTON_ADD_DELETE].pressed || _buttons[BUTTON_MODE].pressed)
    {
        // Save current settings back to preset slot
        _save_preset(&_presets.presets[_current_preset_index].settings);
        LOG_INFO("finished editing preset %s", _presets.presets[_current_preset_index].name);

        // Return to preset playback state
        _stop_metronome();
        _state_transition(STATE_PRESET_PLAYBACK);

        _buttons[BUTTON_ADD_DELETE].pressed = false;
        _buttons[BUTTON_MODE].pressed = false;
    }
    else
    {
        // handle buttons for changing beat & BPM, and for starting/stopping metronome
        lcd_update_required = _handle_metronome_settings_buttons();
    }

    return lcd_update_required;
}

// Handle button inputs on the edit/delete preset menu screen
static bool _handle_edit_delete_menu_inputs(void)
{
    bool lcd_update_required = false;

    static const unsigned int num_options = 3u;
    static const char *options[num_options] = {"Edit preset", "Delete preset", "Cancel"};
    static unsigned int selected = 0u;

    if (_buttons[BUTTON_UP].pressed)
    {
        if (selected > 0u)
        {
            selected -= 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_UP].pressed = false;
    }
    else if (_buttons[BUTTON_DOWN].pressed)
    {
        if (selected < (num_options - 1u))
        {
            selected += 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_DOWN].pressed = false;
    }
    else if (_buttons[BUTTON_SELECT].pressed)
    {
        if (2u == selected)
        {
            // Cancel, go back to preset playback
            _state_transition(STATE_PRESET_PLAYBACK);
        }
        else if (1u == selected)
        {
            // Delete. double check ("are you sure...")
            _state_transition(STATE_PRESET_DELETE_CHECK);
        }
        else if (0u == selected)
        {
            // Edit preset
            _state_transition(STATE_PRESET_EDIT);
        }

        lcd_update_required = true;
        _buttons[BUTTON_SELECT].pressed = false;
    }

    return lcd_update_required;
}

// Handle button inputs on the "are you sure?" screen for preset deletion
static bool _handle_preset_delete_inputs(void)
{
    bool lcd_update_required = false;

    static const unsigned int num_options = 2u;
    static const char *options[num_options] = {"Yes", "No"};
    static unsigned int selected = 0u;

    if (_buttons[BUTTON_UP].pressed)
    {
        if (selected > 0u)
        {
            selected -= 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_UP].pressed = false;
    }
    else if (_buttons[BUTTON_DOWN].pressed)
    {
        if (selected < (num_options - 1u))
        {
            selected += 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_DOWN].pressed = false;
    }
    else if (_buttons[BUTTON_SELECT].pressed)
    {
        if (1u == selected)
        {
            // Cancel deletion, go back to edit/delete menu
            _state_transition(STATE_PRESET_EDIT_DELETE_MENU);
        }
        else if (0u == selected)
        {
            // Delete preset, confirmed
            LOG_INFO("deleting preset #%u '%s'",
                     _current_preset_index,
                     _presets.presets[_current_preset_index].name);
            _delete_preset(_current_preset_index);

            // Return to preset playback state
            _state_transition(STATE_PRESET_PLAYBACK);
        }

        lcd_update_required = true;
    }

    return lcd_update_required;
}

// Handle button inputs on the metronome screen
static bool _handle_metronome_inputs(void)
{
    bool lcd_update_required = false;

    if (_buttons[BUTTON_MODE].pressed)
    {
        _saved_metronome_bpm = _current_bpm;

        // Switch to preset state (also clears button states)
        _state_transition(STATE_PRESET_PLAYBACK);
        lcd_update_required = true;

        // Load current preset data
        _load_preset(_presets.presets[_current_preset_index].settings);
        _tc4_set_period(_current_bpm);
        _preset_change_complete = true;
    }
    else if (_buttons[BUTTON_ADD_DELETE].pressed)
    {
        if (MAX_PRESET_COUNT <= _presets.preset_count)
        {
            // Display message on char LCD and block for 2s
            _max_preset_count_exceeded();
        }
        else
        {
            // Switch to preset name entry state (also clears button states)
            _alphanum_col = 2u;
            _alphanum_row = 0u;
            _state_transition(STATE_PRESET_NAME_ENTRY);
            lcd_update_required = true;
        }

        lcd_update_required = true;
    }
    else
    {
        // handle buttons for changing beat & BPM, and for starting/stopping metronome
        lcd_update_required = _handle_metronome_settings_buttons();
    }

    return lcd_update_required;
}

// Handle button inputs in "preset playback" mode
static bool _handle_preset_playback_inputs(void)
{
    bool lcd_update_required = false;

    if (_preset_change_complete)
    {
        LOG_INFO("loaded preset '%s'", _presets.presets[_current_preset_index].name);
        _preset_change_complete = false;
    }

    if (_buttons[BUTTON_UP].pressed)
    {
        unsigned int new_index = (_current_preset_index + 1u) % _presets.preset_count;
        if (_metronome_running)
        {
            // Increment requested preset index
            _requested_preset_index = new_index;
            _preset_change_requested = true;
        }
        else
        {
            // Load new preset immediately
            _current_preset_index = new_index;
            _load_preset(_presets.presets[_current_preset_index].settings);
            _preset_change_complete = true;
        }

        _buttons[BUTTON_UP].pressed = false;
    }
    else if (_buttons[BUTTON_DOWN].pressed)
    {
        unsigned int new_index = (0u == _current_preset_index) ? _presets.preset_count - 1u :
                                                                 _current_preset_index - 1u;
        if (_metronome_running)
        {
            // Increment requested preset index
            _requested_preset_index = new_index;
            _preset_change_requested = true;
        }
        else
        {
            // Load new preset immediately
            _current_preset_index = new_index;
            _load_preset(_presets.presets[_current_preset_index].settings);
            _preset_change_complete = true;
        }

        _buttons[BUTTON_DOWN].pressed = false;
    }
    else if (_buttons[BUTTON_SELECT].pressed)
    {
        // Start/stop metronome
        if (!_metronome_running)
        {
            // Start timer counting for current BPM
            _tc4_set_period(_current_bpm);
            _start_metronome();
        }
        else
        {
            _stop_metronome();
        }

        _buttons[BUTTON_SELECT].pressed = false;
    }
    else if (_buttons[BUTTON_MODE].pressed)
    {
        // Switch to metronome state (also clears button states)
        _state_transition(STATE_METRONOME);
        _current_bpm = _saved_metronome_bpm;
        lcd_update_required = true;
    }
    else if (_buttons[BUTTON_ADD_DELETE].pressed)
    {
        // Switch to edit/delete menu state
        _state_transition(STATE_PRESET_EDIT_DELETE_MENU);
        _stop_metronome();
        lcd_update_required = true;
    }

    if (_preset_change_requested)
    {
        // Update the screen right away on preset change
        lcd_update_required = true;

        // If metronome not running, we can update metronome settings right now
        if (!_metronome_running)
        {
            _load_preset(_presets.presets[_requested_preset_index].settings);
            _current_preset_index = _requested_preset_index;
            _preset_change_requested = false;
        }
    }

    return lcd_update_required;
}

// Increment alphanum table row index to next non-space character,
// return true if a non-space character was found
static bool _change_alphanum_row(bool increment)
{
    int32_t val_to_add = increment ? 1 : -1;

    unsigned int old_row = _alphanum_row;
    unsigned int old_col = _alphanum_col;

    if ((_alphanum_row == 0) && !increment)
    {
        return false;
    }
    if ((_alphanum_row == (ALPHANUM_ROWS - 1)) && increment)
    {
        return false;
    }

    do
    {
        _alphanum_row = (unsigned int) (((int) _alphanum_row) + val_to_add);
        if (' ' != _alphanum_table[_alphanum_row][_alphanum_col])
        {
            LOG_INFO("cursor (%c)", _alphanum_table[_alphanum_row][_alphanum_col]);
            return true;
        }
    } while ((_alphanum_row > 0) && (_alphanum_row < (ALPHANUM_ROWS - 1)));

    _alphanum_row = old_row;
    _alphanum_col = old_col;

    return false;
}

// Increment alphanum table column index to next non-space character,
// return true if a non-space character was found
static bool _change_alphanum_col(bool increment)
{
    int32_t val_to_add = increment ? 1 : -1;

    unsigned int old_row = _alphanum_row;
    unsigned int old_col = _alphanum_col;

    if ((_alphanum_col == 0) && !increment)
    {
        return false;
    }
    if ((_alphanum_col == (ALPHANUM_COLS - 1)) && increment)
    {
        return false;
    }

    do
    {
        _alphanum_col = (unsigned int) (((int) _alphanum_col) + val_to_add);
        if (' ' != _alphanum_table[_alphanum_row][_alphanum_col])
        {
            LOG_INFO("cursor (%c)", _alphanum_table[_alphanum_row][_alphanum_col]);
            return true;
        }
    } while ((_alphanum_col > 0) && (_alphanum_col < (ALPHANUM_COLS - 1)));

    if (increment)
    {
        // Go to save/end button by default if all the way to the right
        LOG_INFO("cursor (*)");
        _alphanum_row = ALPHANUM_ROWS - 1u;
        _alphanum_col = ALPHANUM_COLS - 2u;
        return true;
    }

    _alphanum_row = old_row;
    _alphanum_col = old_col;

    return false;
}

// Handle button inputs in "preset name entry" mode
static bool _handle_name_entry_inputs(void)
{
    bool lcd_update_required = false;
    if (_buttons[BUTTON_MODE].pressed || _buttons[BUTTON_ADD_DELETE].pressed)
    {
        // Cancel, switch back to metronome state (also clears button states)
        _state_transition(STATE_METRONOME);
        lcd_update_required = true;
    }
    else if (_buttons[BUTTON_SELECT].pressed)
    {
        char selected = _alphanum_table[_alphanum_row][_alphanum_col];
        if ('<' == selected)
        {
            // Delete last char
            if (0u < _preset_name_pos)
            {
                _preset_name_pos -= 1u;
                lcd_update_required = true;
                LOG_INFO("delete (%c)", _preset_name_buf[_preset_name_pos]);
            }
        }
        else if ('*' == selected)
        {
            // Done, save preset
            _preset_name_buf[_preset_name_pos] = '\0';
            metronome_preset_t *preset_slot = &_presets.presets[_presets.preset_count];
            memcpy(preset_slot->name, _preset_name_buf, _preset_name_pos + 1u);
            _save_preset(&preset_slot->settings);

            _presets.preset_count += 1u;
            _preset_name_pos = 0u;

            LOG_INFO("saved preset '%s' in slot %u/%u", preset_slot->name, _presets.preset_count, MAX_PRESET_COUNT);

            // Switch to metronome state (also clears button states)
            _state_transition(STATE_METRONOME);
        }
        else
        {
            if (MAX_PRESET_NAME_LEN > _preset_name_pos)
            {
                _preset_name_buf[_preset_name_pos] = selected;
                _preset_name_pos += 1u;
                lcd_update_required = true;
                LOG_INFO("selected char '%c'", selected);
            }
        }

        _buttons[BUTTON_SELECT].pressed = false;
    }
    else if (_buttons[BUTTON_UP].pressed)
    {
        lcd_update_required = _change_alphanum_row(false);
        _buttons[BUTTON_UP].pressed = false;
    }
    else if (_buttons[BUTTON_DOWN].pressed)
    {
        lcd_update_required = _change_alphanum_row(true);
        _buttons[BUTTON_DOWN].pressed = false;
    }
    else if (_buttons[BUTTON_LEFT].pressed)
    {
        lcd_update_required = _change_alphanum_col(false);
        _buttons[BUTTON_LEFT].pressed = false;
    }
    else if (_buttons[BUTTON_RIGHT].pressed)
    {
        lcd_update_required = _change_alphanum_col(true);
        _buttons[BUTTON_RIGHT].pressed = false;
    }

    return lcd_update_required;
}

// Calls the correct input handler function based on current mode.
// Returns true if screen update is required.
static bool _handle_inputs(void)
{
    unsigned int buttons_pressed = 0u;

    // Handle debouncing for buttons
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        switch(_buttons[i].state)
        {
            case DEBOUNCE_IDLE:
                // Nothing to do
                continue;
                break;
            case DEBOUNCE_FORCE:
                // Special case for CLI injected inputs, force pressed despite no GPIO state change
                _buttons[i].state = DEBOUNCE_IDLE;
                buttons_pressed += 1u;
                break;
            case DEBOUNCE_START:
                // Start debouncing this signal
                _buttons[i].start_ms = millis();
                _buttons[i].state = DEBOUNCE_ACTIVE;
                break;
            case DEBOUNCE_ACTIVE:
            {
                // Debounce in progress, check if complete
                unsigned long elapsed = millis() - _buttons[i].start_ms;
                if (BUTTON_DEBOUNCE_MS <= elapsed)
                {
                    if (digitalRead(_buttons[i].gpio_pin) == _buttons[i].pressed_state)
                    {
                        // Button is pressed after debounce
                        _buttons[i].pressed = true;
                        buttons_pressed += 1u;
                    }

                    // Re-enable interrupt for this pin
                    attachInterrupt(digitalPinToInterrupt(_buttons[i].gpio_pin),
                                    _buttons[i].callback,
                                    (_buttons[i].pressed_state) ? RISING : FALLING);

                    _buttons[i].state =  DEBOUNCE_IDLE;
                }
                break;
            }
        }
    }

    if (0u == buttons_pressed)
    {
        // If no buttons have been pressed, we can stop here
        return false;
    }

    // Handle any buttons that have been pressed
    bool lcd_update_required = false;
    switch (_current_state)
    {
        case STATE_METRONOME:
            lcd_update_required = _handle_metronome_inputs();
            break;
        case STATE_PRESET_EDIT_DELETE_MENU:
            lcd_update_required = _handle_edit_delete_menu_inputs();
            break;
        case STATE_PRESET_PLAYBACK:
            lcd_update_required = _handle_preset_playback_inputs();
            break;
        case STATE_PRESET_EDIT:
            lcd_update_required = _handle_preset_edit_inputs();
            break;
        case STATE_PRESET_DELETE_CHECK:
            lcd_update_required =  _handle_preset_delete_inputs();
            break;
        case STATE_PRESET_NAME_ENTRY:
            lcd_update_required = _handle_name_entry_inputs();
            break;
    }

    return lcd_update_required;
}

void setup()
{

    // Required for snprintf to support floats :(
    asm(".global _printf_float");

    // Use built-in LED to show when we are streaming samples to I2S device
    pinMode(13, OUTPUT);

    // Configure button input pins
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        pinMode(_buttons[i].gpio_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(_buttons[i].gpio_pin),
                                              _buttons[i].callback,
                                              (_buttons[i].pressed_state) ? RISING : FALLING);
    }

    // Configure TC4 as a 32-bit counter counting at 48MHz. TC4 is used to
    // generate interrupts for the metronome beat.
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY);
    _tc4_reset();
    TC4->COUNT32.CTRLA.reg |= (TC_CTRLA_MODE_COUNT32 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV1);

    // Configure IRQ for TC4
    NVIC_DisableIRQ(TC4_IRQn);
    NVIC_ClearPendingIRQ(TC4_IRQn);
    NVIC_SetPriority(TC4_IRQn, 0);
    NVIC_EnableIRQ(TC4_IRQn);

    // Enable TC4 interrupt request
    TC4->COUNT32.INTENSET.bit.MC0 = 1;
    while (_tc4_syncing());

    // Set timer period based on starting BPM
    _tc4_set_period(_current_bpm);

#if ENABLE_UART_LOGGING || ENABLE_UART_CLI
    Serial.begin(115200);
#endif // ENABLE_UART_LOGGING || ENABLE_UART_CLI

    LOG_INFO("Version "METRONOME_SKETCH_VERSION);
    LOG_INFO("preset store is %u bytes", sizeof(_presets));
    LOG_INFO("flash writes %s", (ENABLE_FLASH_WRITE) ? "enabled" : "disabled");

    // Read presets from flash and check CRC
    _presets = preset_store.read();
    uint32_t stored_crc = _presets.crc;
    uint32_t calc_crc = _calc_preset_crc();
    if (stored_crc != calc_crc)
    {
        LOG_INFO("Invalid CRC, expected 0x%x got 0x%x. Resetting presets.", calc_crc, stored_crc);
        memset(&_presets, 0, sizeof(_presets));
#if ENABLE_FLASH_WRITE
        _presets.crc = _calc_preset_crc();
        preset_store.write(_presets);
        _preset_crc_on_boot = _presets.crc;
#endif // ENABLE_FLASH_WRITE
    }
    else
    {
        _preset_crc_on_boot = stored_crc;
        LOG_INFO("%u presets stored", _presets.preset_count);
    }
}

void loop()
{
#if ENABLE_UART_CLI
    _handle_cli_commands();
#endif // ENABLE_UART_CLI

    if (_handle_inputs())
    {
        // _handle_inputs returns true if the char LCD needs to be re-drawn
        _update_char_lcd();
    }
}
