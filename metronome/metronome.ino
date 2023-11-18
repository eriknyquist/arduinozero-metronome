/*
 * Arduino Zero Stage Metronome
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
 *   handler and DMA interrupt handler, using only the non-blocking versions of the
 *   I2S library functions. Everything else (UART reads/writes, character LCD writes,
 *   button state processing) is done in the main context inside the loop() function.
 *   The result of all this is that the timing of metronome beats is very reliable
 *   (or at least, as reliable as they can be, given the accuracy of interrupts from
 *   TC4), since anything else we happen to be doing can always be interrupted
 *   to serve up another chunk of audio samples to the I2S DAC.
 *
 * - UART-based command line interface allows full command/control of the metronome
 *   user interface by interacting with the serial port (this is just extra-- all
 *   features, including creating/naming/editing/deleting presets, can be accessed via
 *   buttons & character LCD on the device).
 */

#include "metronome_beep_samples.h"
#include "metronome_config.h"
#include "ModifiedI2S.h"

#include <stdarg.h>

#include <LiquidCrystal.h>
#include <Arduino_CRC32.h>
#include <FlashStorage.h>
#include <ArduinoLowPower.h>


// Version number reported by CLI
#define METRONOME_SKETCH_VERSION "0.0.1"

// Only used for debugging, useful if testing something unrelated to
// preset storage and want to avoid wasting flash write cycles
#define ENABLE_FLASH_WRITE       (1u)


// I2S sample rate and sample width
#define SAMPLE_RATE              (44100)
#define SAMPLE_WIDTH             (16)


#define MAX_PRESET_NAME_LEN      (32u)   // Max. number of characters for a preset name string
#define MAX_PRESET_COUNT         (128u)  // Max. number of presets that can be saved in flash
#define MIN_BPM                  (1u)    // Min. allowed BPM value
#define MAX_BPM                  (512u)  // Max. allowed BPM value
#define MIN_BEAT                 (1u)    // Min. allowed beat value
#define MAX_BEAT                 (16u)   // Max. allowed beat value

// Actual system core clock freq for Arduino Zero
// (taken from https://github.com/manitou48/crystals/blob/master/crystals.txt)
#define TRUE_CORE_CLOCK_HZ       (47972352UL)


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
    BUTTON_VOLUP,
    BUTTON_VOLDOWN,
    BUTTON_ON_OFF_SWITCH,
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
    bool pressed;                    // True if button is currently pressed
    bool unhandled_press;            // True if the button is pressed and we have not handled it yet
    int pressed_state;               // GPIO value that represents a pressed button
    debounce_state_e state;          // Current debounce state
    unsigned int hold_repeat_count;  // Time button has been held, in increments of #BUTTON_HOLD_CHANGE_MS
    unsigned long start_ms;          // Debounce start time, milliseconds
    void (*callback)(void);          // Interrupt handler
    int gpio_pin;                    // GPIO pin number for button
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
static void _version_cmd_handler(char *cmd_args);
static void _addpreset_cmd_handler(char *cmd_args);
static void _help_cmd_handler(char *cmd_args);
static void _up_cmd_handler(char *cmd_args);
static void _down_cmd_handler(char *cmd_args);
static void _left_cmd_handler(char *cmd_args);
static void _right_cmd_handler(char *cmd_args);
static void _select_cmd_handler(char *cmd_args);
static void _mode_cmd_handler(char *cmd_args);
static void _add_del_cmd_handler(char *cmd_args);
static void _volup_cmd_handler(char *cmd_args);
static void _voldown_cmd_handler(char *cmd_args);
static void _poweroff_cmd_handler(char *cmd_args);


// Number of CLI commands we can handle (must be manually synced with _cli_commands)
#define CLI_COMMAND_COUNT (14u)

// Table mapping CLI command words to command handlers
static cli_command_t _cli_commands[CLI_COMMAND_COUNT] =
{
    {"help", _help_cmd_handler},
    {"version", _version_cmd_handler},
    {"presets", _presets_cmd_handler},
    {"addpreset", _addpreset_cmd_handler},
    {"off", _poweroff_cmd_handler},
    {"u", _up_cmd_handler},
    {"d", _down_cmd_handler},
    {"l", _left_cmd_handler},
    {"r", _right_cmd_handler},
    {"s", _select_cmd_handler},
    {"m", _mode_cmd_handler},
    {"a", _add_del_cmd_handler},
    {"+", _volup_cmd_handler},
    {"-", _voldown_cmd_handler}
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
void _volup_button_callback(void) { _gpio_callback(BUTTON_VOLUP); }
void _voldown_button_callback(void) { _gpio_callback(BUTTON_VOLDOWN); }
void _off_switch_callback(void) { _gpio_callback(BUTTON_ON_OFF_SWITCH); }


// State tracking/debouncing for all buttons
static volatile button_info_t _buttons[BUTTON_COUNT] =
{
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _up_button_callback, UP_BUTTON_PIN},              // BUTTON_UP
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _down_button_callback, DOWN_BUTTON_PIN},          // BUTTON_DOWN
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _left_button_callback, LEFT_BUTTON_PIN},          // BUTTON_LEFT
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _right_button_callback, RIGHT_BUTTON_PIN},        // BUTTON_RIGHT
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _select_button_callback,  SELECT_BUTTON_PIN},     // BUTTON_SELECT
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _mode_button_callback, MODE_BUTTON_PIN},          // BUTTON_MODE
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _add_delete_button_callback, ADD_DEL_BUTTON_PIN}, // BUTTON_ADD_DELETE
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _volup_button_callback, VOLUP_BUTTON_PIN},        // BUTTON_VOLUP
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _voldown_button_callback, VOLDOWN_BUTTON_PIN},    // BUTTON_VOLDOWN
    {false, false, LOW, DEBOUNCE_IDLE, 0u, 0u, _off_switch_callback, ON_OFF_SWITCH_PIN}          // BUTTON_ON_OFF_SWITCH
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
static volatile uint16_t _current_beat = MIN_BEAT;


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

// Number of samples we can send to the HiLetgo PCM5102 I2S DAC
// in one go without it blocking on us
#define SAMPLE_BUF_LEN       (256u)

// Pointer to the full list of stereo WAV samples for the beep sound currently
// being streamed to the HiLetgo PCM5102 I2S DAC (high beep or low beep)
static const int16_t *_beep_samples = high_beep_samples;

// Sample buffer passed to I2S.write(). We stream a beep sound
// to the HiLetgo PCM5102 I2S DAC in SAMPLE_BUF_LEN-sized chunks
// via this buffer.
static int16_t _sample_buf[SAMPLE_BUF_LEN];

// Current position (start of next chunk to stream) within _beep_samples
static volatile uint32_t _beep_samples_pos = 0u;

// Tracks current volume level as a percentage (0-100)
static unsigned int _volume = 100u;


#if ENABLE_UART_LOGGING
#define LOG_INFO(fmt, ...) log("INFO", __func__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) log("ERROR", __func__, __LINE__, fmt, ##__VA_ARGS__)

// Formats and prints a debug message to the UART
static void log(const char *level, const char *func, int line, const char *fmt, ...)
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

// LCD object for controlling the screen
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);


// Increment volume by 10%
static bool _increment_volume(void)
{
    bool changed = false;
    if (_volume <= 90u)
    {
        _volume += 10u;
        changed  = true;
    }

    return changed;
}

// Decrement volume by 10%
static bool _decrement_volume(void)
{
    bool changed = false;
    if (_volume >= 10u)
    {
        _volume -= 10u;
        changed = true;
    }

    return changed;
}

// Disable all interrupts, set all pins to inputs, and save presets to flash
static void _power_off(void)
{
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

    // Disable I2S
    ModifiedI2S.end();

    LOG_INFO("powering off");

#if ENABLE_UART_LOGGING || ENABLE_UART_CLI
    // Disable serial
    delay(100);
    Serial.end();
#endif // ENABLE_UART_LOGGING || ENABLE_UART_CLI

    // Go into low power mode forever
    LowPower.deepSleep();
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
static void _help_cmd_handler(char *cmd_args)
{
    (void) cmd_args;
    Serial.println("-------- CLI command reference ---------");
    Serial.print("Version ");
    Serial.println(METRONOME_SKETCH_VERSION);
    Serial.println("help      - Show this printout.");
    Serial.println("presets   - Show all saved presets.");
    Serial.println("addpreset - Create new preset. One line of output");
    Serial.println("            from the 'presets' command should be passed");
    Serial.println("            as parameter(s).");
    Serial.println("off       - Save presets to flash, power off device.");
    Serial.println("u         - Emulate UP button press.");
    Serial.println("d         - Emulate DOWN button press.");
    Serial.println("l         - Emulate LEFT button press.");
    Serial.println("r         - Emulate RIGHT button press.");
    Serial.println("s         - Emulate SELECT button press.");
    Serial.println("m         - Emulate MODE button press.");
    Serial.println("a         - Emulate ADD/DEL button press.");
    Serial.println("+         - Emulate 'volume up' button press");
    Serial.println("-         - Emulate 'volume down' button press");
    Serial.println("----------------------------------------");
}

// 'show software version' CLI command handler
static void _version_cmd_handler(char *cmd_args)
{
    (void) cmd_args;
    Serial.print("Arduino Zero Stage Metronome ");
    Serial.println(METRONOME_SKETCH_VERSION);
}

// 'dump presets' CLI command handler
static void _presets_cmd_handler(char *cmd_args)
{
    (void) cmd_args;
    Serial.print(_presets.preset_count);
    Serial.println(" preset(s) saved");

    if (0u == _presets.preset_count)
    {
        return;
    }

    for (unsigned int i = 0u; i < _presets.preset_count; i++)
    {
        Serial.print("preset #");
        Serial.print(i);
        Serial.print(": ");
        Serial.print("0x");
        Serial.print(_presets.presets[i].settings, HEX);
        Serial.print(" ");
        Serial.println(_presets.presets[i].name);
    }
}

// 'add new preset' CLI command handler
static void _addpreset_cmd_handler(char *cmd_args)
{
    bool valid_preset_found = false;
    bool asciizero_seen = false;

    if (MAX_PRESET_COUNT <= _presets.preset_count)
    {
        Serial.println("No more room for presets");
        return;
    }

    // Look for the '0x' at the start of a preset data string
    while (*cmd_args)
    {
        if (asciizero_seen)
        {
            if ((*cmd_args == 'x') || (*cmd_args == 'X'))
            {
                cmd_args++;
                break;
            }
            else
            {
                asciizero_seen = false;
            }
        }
        else
        {
            if (*cmd_args == '0')
            {
                asciizero_seen = true;
            }
        }

        cmd_args++;
    }

    // If we found '0x' before the end of the string...
    if (*cmd_args)
    {
        char numbuf[5];
        bool valid_hex_seen = false;
        // There should be no more than 4 hex chars, followed by a space
        for (int i = 0; i < 5; i++)
        {
            char c = cmd_args[i];
            if (((c <= 'F') && (c >= 'A')) ||
                ((c <= 'f') && (c >= 'a')) ||
                ((c <= '9') && (c >= '0')))
            {
                numbuf[i] = c;
            }
            else if ((c == ' ') && (i > 0))
            {
                numbuf[i] = '\0';
                valid_hex_seen = true;
                cmd_args += i + 1;
                break;
            }
            else
            {
                break;
            }
        }

        if (valid_hex_seen && *cmd_args)
        {
            char *endptr = NULL;
            unsigned long preset_data = strtoul(numbuf, &endptr, 16);
            if (endptr && (*endptr == '\0'))
            {
                metronome_preset_t *preset_slot = &_presets.presets[_presets.preset_count];
                for (int i = 0; (i < (int) sizeof(preset_slot->name)) && *cmd_args; i++, cmd_args++)
                {
                    if (*cmd_args == '\n')
                    {
                        preset_slot->name[i] = '\0';
                        break;
                    }

                    preset_slot->name[i] = *cmd_args;
                }

                if (!*cmd_args)
                {
                    // Ensure null-terminated if name was too long
                    preset_slot->name[sizeof(preset_slot->name) - 1] = '\0';
                }

                preset_slot->settings = (uint16_t) preset_data;
                _presets.preset_count += 1u;
                valid_preset_found = true;

                Serial.print("Added preset '");
                Serial.print(preset_slot->name);
                Serial.println("'");
            }
        }
    }

    if (!valid_preset_found)
    {
        Serial.println("Invalid preset data provided");
    }
}

// Generic simulated button press CLI command handler
static void _button_cli_handler(button_e button)
{
    _buttons[button].state = DEBOUNCE_FORCE;
}

// CLI command handlers for simulated button presses
static void _up_cmd_handler(char *cmd_args)       { (void) cmd_args; _button_cli_handler(BUTTON_UP); }
static void _down_cmd_handler(char *cmd_args)     { (void) cmd_args; _button_cli_handler(BUTTON_DOWN); }
static void _left_cmd_handler(char *cmd_args)     { (void) cmd_args; _button_cli_handler(BUTTON_LEFT); }
static void _right_cmd_handler(char *cmd_args)    { (void) cmd_args; _button_cli_handler(BUTTON_RIGHT); }
static void _select_cmd_handler(char *cmd_args)   { (void) cmd_args; _button_cli_handler(BUTTON_SELECT); }
static void _mode_cmd_handler(char *cmd_args)     { (void) cmd_args; _button_cli_handler(BUTTON_MODE); }
static void _add_del_cmd_handler(char *cmd_args)  { (void) cmd_args; _button_cli_handler(BUTTON_ADD_DELETE); }
static void _volup_cmd_handler(char *cmd_args)    { (void) cmd_args; _button_cli_handler(BUTTON_VOLUP); }
static void _voldown_cmd_handler(char *cmd_args)  { (void) cmd_args; _button_cli_handler(BUTTON_VOLDOWN); }
static void _poweroff_cmd_handler(char *cmd_args) { (void) cmd_args; _button_cli_handler(BUTTON_ON_OFF_SWITCH); }


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

    // Find the end of the command word
    unsigned int cmd_word_end = 0u;
    for (; cmd_word_end < sizeof(_cli_buf); cmd_word_end++)
    {
        if (IS_SPACE(_cli_buf[cmd_word_end]))
        {
            _cli_buf[cmd_word_end] = '\0';
            break;
        }
    }

    // Look for a command matching this command word in _cli_commands
    bool recognized_command = false;
    for (unsigned int i = 0u; i < CLI_COMMAND_COUNT; i++)
    {
        if (0 == strncmp(_cli_commands[i].cmd_word, _cli_buf, sizeof(_cli_buf)))
        {
            // Run handler, pass a pointer to the characters after the command word
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

// Pack current BPM and beat count into a preset slot
static void _save_preset(uint16_t *preset, uint16_t *bpm, uint16_t *beat_count)
{
    *preset = ((*bpm - 1u) & 0x1FFu) | (((*beat_count - 1u) & 0xFu) << 0x9u);
}

// Populate current BPM and beat count from a saved preset slot
static void _load_preset(uint16_t preset_data, uint16_t *bpm, uint16_t *beat_count)
{
    *bpm = ((preset_data) & 0x1FFu) + 1u;
    *beat_count = (((preset_data) >> 9u) & 0xFu) + 1u;
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

// Sends SAMPLE_BUF_LEN samples of 0 to the I2S DAC
static void _send_onebuf_silence(void)
{
    static uint16_t silence[SAMPLE_BUF_LEN] = {0u};
    ModifiedI2S.write(silence, sizeof(silence));
}

// I2S transfer complete function, provides the next chunk of audio samples for I2S DAC
static void _i2s_complete_handler(void)
{
    static bool final_silence_sent = false;

    if (_beep_samples_pos >= BEEP_SAMPLE_COUNT)
    {
        // All samples for this beep sound have been sent
        if (!final_silence_sent)
        {
            // Send one buffer of silence at the end
            _send_onebuf_silence();
            final_silence_sent = true;
        }

        if (0 == ModifiedI2S.remainingBytesToTransmit())
        {
            // done with I2S transfers for this beep sound
            ModifiedI2S.disable();
            final_silence_sent = false;
        }

        return;
    }

    uint32_t samples_remaining = BEEP_SAMPLE_COUNT - _beep_samples_pos;
    uint32_t samples_to_send = min(samples_remaining, SAMPLE_BUF_LEN);

    // Copy samples to sample buf, modifying for current volume as we go
    float fvol = ((float) _volume) / 100.0f;
    for (uint32_t i = 0u; i < samples_to_send; i++)
    {
        _sample_buf[i] = (uint16_t) (((float) _beep_samples[_beep_samples_pos + i]) * fvol);
    }

    (void) ModifiedI2S.write(_sample_buf, sizeof(uint16_t) * samples_to_send);
    _beep_samples_pos += samples_to_send;
}

// Start streaming sound for first beat of bar
static void _stream_beat_sound(void)
{
    _beep_samples_pos = 0u;
    _beep_samples = high_beep_samples;
    ModifiedI2S.enable();

    // Fill the double-buffer with SAMPLE_BUF_LEN * 2 samples
    _send_onebuf_silence(); // Start with one buffer of silence
    _i2s_complete_handler();
}

// Start streaming sound for non-first beat of bar
static void _stream_subbeat_sound(void)
{
    _beep_samples_pos = 0u;
    _beep_samples = low_beep_samples;
    ModifiedI2S.enable();

    // Fill the double-buffer with SAMPLE_BUF_LEN * 2 samples
    _send_onebuf_silence(); // Start with one buffer of silence
    _i2s_complete_handler();
}

// Called on TC4 interrupt, starts streaming the next beat to the I2S DAC
static void _start_streaming_next_beat(void)
{
    if ((MIN_BEAT == _current_beat) && _preset_change_requested)
    {
        // First beat of bar, check if preset change was requested
        _load_preset(_presets.presets[_requested_preset_index].settings,
                     (uint16_t *) &_current_bpm,
                     (uint16_t *) &_current_beat_count);
        _current_preset_index = _requested_preset_index;
        _preset_change_requested = false;
        _preset_change_complete = true;
    }

    _tc4_set_period(_current_bpm);

    if (MIN_BEAT == _current_beat)
    {
        _stream_beat_sound();
    }
    else
    {
        _stream_subbeat_sound();
    }

    if (_current_beat_count <= _current_beat)
    {
        _current_beat = MIN_BEAT;
    }
    else
    {
        _current_beat += 1u;
    }
}

// Wait for TC4 to be not busy
static bool _tc4_syncing(void)
{
    return TC4->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY;
}

// Reset (stop) TC4 timer
static void _tc4_reset(void)
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
    // Reset beat count
    _current_beat = MIN_BEAT;

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
static void _tc4_set_period(unsigned int bpm)
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
static void _state_transition(metronome_state_e new_state)
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
        default:
            statename = "Unrecognized state";
            break;
    }
    LOG_INFO("changing to state '%s'", statename);

    // Stop metronome timer/counter
    _stop_metronome();

    // Reset all button states
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        _buttons[i].unhandled_press = false;
        _buttons[i].state = DEBOUNCE_IDLE;
    }

    // Update state value
    _current_state = new_state;
}

// Handle timing & value increment calculations for a button that is being held down
static bool _handle_held_button(button_e button, int *update_value)
{
    bool ret = false;
    unsigned long now = millis();
    unsigned long elapsed = now - _buttons[button].start_ms;

    if (BUTTON_HOLD_CHANGE_MS <= elapsed)
    {
        unsigned int double_count = _buttons[button].hold_repeat_count / BUTTON_HOLD_DOUBLE_COUNT;
        ret = true;

        if (double_count < BUTTON_HOLD_MAX_DOUBLES)
        {
            if ((_buttons[button].hold_repeat_count % BUTTON_HOLD_DOUBLE_COUNT) == 0)
            {
                *update_value *= 2;
            }
        }

        _buttons[button].start_ms = now;
        _buttons[button].hold_repeat_count += 1u;
    }

    return ret;
}

// Handle all buttons related to changing metronome BPM and beat count, and
// starting/stopping metronome. Used in the STATE_METRONOME and STATE_PRESET_EDIT states.
static bool _handle_metronome_settings_buttons(void)
{
    static int bpm_update_value = 0;

    bool lcd_update_required = false;

    if (_buttons[BUTTON_UP].unhandled_press)
    {
        // Increment BPM
        if (MAX_BPM > _current_bpm)
        {
            lcd_update_required = true;
            _current_bpm += 1u;
            LOG_INFO("%u BPM", _current_bpm);
        }

        bpm_update_value = 2;
        _buttons[BUTTON_UP].hold_repeat_count = 0;
        _buttons[BUTTON_UP].unhandled_press = false;
    }
    else
    {
        if (_buttons[BUTTON_UP].pressed && (MAX_BPM > _current_bpm))
        {
            if (_handle_held_button(BUTTON_UP, &bpm_update_value))
            {
                if ((_current_bpm + (uint16_t) bpm_update_value) > MAX_BPM)
                {
                    _current_bpm = MAX_BPM;
                }
                else
                {
                    _current_bpm += (uint16_t) bpm_update_value;
                }

                LOG_INFO("%u BPM", _current_bpm);
                lcd_update_required = true;
            }
        }
    }

    if (_buttons[BUTTON_DOWN].unhandled_press)
    {
        // Decrement BPM
        if (MIN_BPM < _current_bpm)
        {
            lcd_update_required = true;
            _current_bpm -= 1u;
            LOG_INFO("%u BPM", _current_bpm);
        }

        bpm_update_value = 2;
        _buttons[BUTTON_DOWN].hold_repeat_count = 0;
        _buttons[BUTTON_DOWN].unhandled_press = false;
    }
    else
    {
        if (_buttons[BUTTON_DOWN].pressed && (MIN_BPM < _current_bpm))
        {
            if (_handle_held_button(BUTTON_DOWN, &bpm_update_value))
            {
                if (_current_bpm < (uint16_t) bpm_update_value)
                {
                    _current_bpm = MIN_BPM;
                }
                else
                {
                    _current_bpm -= (uint16_t) bpm_update_value;
                }

                LOG_INFO("%u BPM", _current_bpm);
                lcd_update_required = true;
            }
        }
    }

    if (_buttons[BUTTON_LEFT].unhandled_press)
    {
        // Decrement beat count
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
        _buttons[BUTTON_LEFT].unhandled_press = false;
    }
    else if (_buttons[BUTTON_RIGHT].unhandled_press)
    {
        // Decrement beat count
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
        _buttons[BUTTON_RIGHT].unhandled_press = false;
    }
    else if (_buttons[BUTTON_SELECT].unhandled_press)
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

        _buttons[BUTTON_SELECT].unhandled_press = false;
    }

    return lcd_update_required;
}

// Handle button inputs on edit saved preset screen
static bool _handle_preset_edit_inputs(void)
{
    bool lcd_update_required = false;
    if (_buttons[BUTTON_ADD_DELETE].unhandled_press || _buttons[BUTTON_MODE].unhandled_press)
    {
        // Save current settings back to preset slot
        _save_preset(&_presets.presets[_current_preset_index].settings,
                     (uint16_t *) &_current_bpm,
                     (uint16_t *) &_current_beat_count);
        LOG_INFO("finished editing preset %s", _presets.presets[_current_preset_index].name);

        // Return to preset playback state
        _stop_metronome();
        _state_transition(STATE_PRESET_PLAYBACK);

        _buttons[BUTTON_ADD_DELETE].unhandled_press = false;
        _buttons[BUTTON_MODE].unhandled_press = false;
    }
    else
    {
        // handle buttons for changing beat & BPM, and for starting/stopping metronome
        lcd_update_required = _handle_metronome_settings_buttons();
    }

    bool volume_changed = _handle_volume_inputs();
    return lcd_update_required || volume_changed;
}

// Handle button inputs on the edit/delete preset menu screen
static bool _handle_edit_delete_menu_inputs(void)
{
    bool lcd_update_required = false;

    static const unsigned int num_options = 3u;
    static const char *options[num_options] = {"Edit preset", "Delete preset", "Cancel"};
    static unsigned int selected = 0u;

    if (_buttons[BUTTON_UP].unhandled_press)
    {
        if (selected > 0u)
        {
            selected -= 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_UP].unhandled_press = false;
    }
    else if (_buttons[BUTTON_DOWN].unhandled_press)
    {
        if (selected < (num_options - 1u))
        {
            selected += 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_DOWN].unhandled_press = false;
    }
    else if (_buttons[BUTTON_SELECT].unhandled_press)
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
        _buttons[BUTTON_SELECT].unhandled_press = false;
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

    if (_buttons[BUTTON_UP].unhandled_press)
    {
        if (selected > 0u)
        {
            selected -= 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_UP].unhandled_press = false;
    }
    else if (_buttons[BUTTON_DOWN].unhandled_press)
    {
        if (selected < (num_options - 1u))
        {
            selected += 1u;
            lcd_update_required = true;
            LOG_INFO("selected: %s", options[selected]);
        }

        _buttons[BUTTON_DOWN].unhandled_press = false;
    }
    else if (_buttons[BUTTON_SELECT].unhandled_press)
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

        _buttons[BUTTON_SELECT].unhandled_press = false;
        lcd_update_required = true;
    }

    return lcd_update_required;
}

// Handle button inputs for changing the volume
static bool _handle_volume_inputs(void)
{
    bool lcd_update_required = false;

    if (_buttons[BUTTON_VOLUP].unhandled_press)
    {
        lcd_update_required = _increment_volume();
        _buttons[BUTTON_VOLUP].unhandled_press = false;
        LOG_INFO("volume: %u%%", _volume);
    }
    else if (_buttons[BUTTON_VOLDOWN].unhandled_press)
    {
        lcd_update_required = _decrement_volume();
        _buttons[BUTTON_VOLDOWN].unhandled_press = false;
        LOG_INFO("volume: %u%%", _volume);
    }

    return lcd_update_required;
}

// Handle button inputs on the metronome screen
static bool _handle_metronome_inputs(void)
{
    bool lcd_update_required = false;

    if (_buttons[BUTTON_MODE].unhandled_press)
    {
        _saved_metronome_bpm = _current_bpm;

        // Switch to preset state (also clears button states)
        _state_transition(STATE_PRESET_PLAYBACK);
        lcd_update_required = true;

        // Load current preset data
        _load_preset(_presets.presets[_current_preset_index].settings,
                     (uint16_t *) &_current_bpm,
                     (uint16_t *) &_current_beat_count);
        _tc4_set_period(_current_bpm);
        _preset_change_complete = true;
    }
    else if (_buttons[BUTTON_ADD_DELETE].unhandled_press)
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

    bool volume_changed = _handle_volume_inputs();
    return lcd_update_required || volume_changed;
}

// Handle button inputs in "preset playback" mode
static bool _handle_preset_playback_inputs(void)
{
    bool lcd_update_required = false;

    if (_buttons[BUTTON_UP].unhandled_press)
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
            _load_preset(_presets.presets[_current_preset_index].settings,
                         (uint16_t *) &_current_bpm,
                         (uint16_t *) &_current_beat_count);
            _preset_change_complete = true;
        }

        _buttons[BUTTON_UP].unhandled_press = false;
    }
    else if (_buttons[BUTTON_DOWN].unhandled_press)
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
            _load_preset(_presets.presets[_current_preset_index].settings,
                         (uint16_t *) &_current_bpm,
                         (uint16_t *) &_current_beat_count);
            _preset_change_complete = true;
        }

        _buttons[BUTTON_DOWN].unhandled_press = false;
    }
    else if (_buttons[BUTTON_SELECT].unhandled_press)
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

        _buttons[BUTTON_SELECT].unhandled_press = false;
    }
    else if (_buttons[BUTTON_MODE].unhandled_press)
    {
        // Switch to metronome state (also clears button states)
        _state_transition(STATE_METRONOME);
        _current_bpm = _saved_metronome_bpm;
        lcd_update_required = true;
    }
    else if (_buttons[BUTTON_ADD_DELETE].unhandled_press)
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
            _load_preset(_presets.presets[_requested_preset_index].settings,
                         (uint16_t *) &_current_bpm,
                         (uint16_t *) &_current_beat_count);
            _current_preset_index = _requested_preset_index;
            _preset_change_requested = false;
        }
    }

    bool volume_changed = _handle_volume_inputs();
    return lcd_update_required || volume_changed;
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
    if (_buttons[BUTTON_MODE].unhandled_press || _buttons[BUTTON_ADD_DELETE].unhandled_press)
    {
        // Cancel, switch back to metronome state (also clears button states)
        _state_transition(STATE_METRONOME);
        lcd_update_required = true;
    }
    else if (_buttons[BUTTON_SELECT].unhandled_press)
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
            _save_preset(&preset_slot->settings, (uint16_t *) &_current_bpm, (uint16_t *) &_current_beat_count);

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

        _buttons[BUTTON_SELECT].unhandled_press = false;
    }
    else if (_buttons[BUTTON_UP].unhandled_press)
    {
        lcd_update_required = _change_alphanum_row(false);
        _buttons[BUTTON_UP].unhandled_press = false;
    }
    else if (_buttons[BUTTON_DOWN].unhandled_press)
    {
        lcd_update_required = _change_alphanum_row(true);
        _buttons[BUTTON_DOWN].unhandled_press = false;
    }
    else if (_buttons[BUTTON_LEFT].unhandled_press)
    {
        lcd_update_required = _change_alphanum_col(false);
        _buttons[BUTTON_LEFT].unhandled_press = false;
    }
    else if (_buttons[BUTTON_RIGHT].unhandled_press)
    {
        lcd_update_required = _change_alphanum_col(true);
        _buttons[BUTTON_RIGHT].unhandled_press = false;
    }

    return lcd_update_required;
}

// Calls the correct input handler function based on current mode.
// Returns true if screen update is required.
static bool _handle_inputs(void)
{
    bool button_changes = 0u;

    // Handle debouncing for buttons
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        switch(_buttons[i].state)
        {
            case DEBOUNCE_IDLE:
                if (_buttons[i].pressed)
                {
                    // make sure we keep running input handlers if button is held
                    button_changes = true;
                }
                break;
            case DEBOUNCE_FORCE:
                // Special case for CLI injected inputs, force pressed despite no GPIO state change
                _buttons[i].state = DEBOUNCE_IDLE;
                _buttons[i].unhandled_press = true;
                button_changes = true;
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
                    int pinval = digitalRead(_buttons[i].gpio_pin);
                    bool is_pressed = (pinval == _buttons[i].pressed_state);
                    if (is_pressed != _buttons[i].pressed)
                    {
                        // Button state has changed after debounce period
                        _buttons[i].pressed = is_pressed;
                        _buttons[i].unhandled_press = is_pressed;
                        button_changes = true;
                    }

                    // Re-enable interrupt for this pin
                    attachInterrupt(_buttons[i].gpio_pin,
                                    _buttons[i].callback,
                                    (_buttons[i].pressed_state) ? RISING : FALLING);

                    _buttons[i].state =  DEBOUNCE_IDLE;
                }
                break;
            }
        }
    }

    if (!button_changes)
    {
        // If no button states have changed, we can stop here
        return false;
    }

    // Handle any button states that have changed
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
        default:
            LOG_ERROR("Unrecognized state (%d)", _current_state);
            break;
    }

    return lcd_update_required;
}

void setup()
{

    // Required for snprintf to support floats :(
    asm(".global _printf_float");

    // Configure button input pins
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        pinMode(_buttons[i].gpio_pin, INPUT_PULLUP);
        attachInterrupt(_buttons[i].gpio_pin,
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

    // Enable LCD screen
    lcd.begin(20,4);

#if ENABLE_UART_LOGGING || ENABLE_UART_CLI
    Serial.begin(115200);
#endif // ENABLE_UART_LOGGING || ENABLE_UART_CLI

    // Initialize I2S
    ModifiedI2S.onTransmit(_i2s_complete_handler);
    if (!ModifiedI2S.begin(I2S_PHILIPS_MODE , SAMPLE_RATE, SAMPLE_WIDTH))
    {
        LOG_ERROR("Failed to initialize I2S :(");
        while (1) {}; // Loop forever
    }
    ModifiedI2S.disable();

    LOG_INFO("Version " METRONOME_SKETCH_VERSION);
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

    if (_preset_change_complete)
    {
        LOG_INFO("loaded preset '%s'", _presets.presets[_current_preset_index].name);
        _preset_change_complete = false;
    }

    if (_handle_inputs())
    {
        // _handle_inputs returns true if the char LCD needs to be re-drawn
        _update_char_lcd();
    }

    // Handle on/off switch in all states
    if (_buttons[BUTTON_ON_OFF_SWITCH].unhandled_press)
    {
        _power_off();
    }
}
