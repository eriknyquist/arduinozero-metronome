
#include <stdarg.h>

#include <Arduino_CRC32.h>
#include <FlashStorage.h>

// If 0, no log messages will be sent to the UART, and CLI will not be available
#define ENABLE_UART          (1u)

// Only used for debugging, useful if testing something unrelated to
// preset storage and want to avoid wasting flash write cycles
#define ENABLE_FLASH_WRITE   (0u)

// Max. number of characters for a preset name string
#define MAX_PRESET_NAME_LEN  (32u)

// Max. number of presets that can be saved in flash
#define MAX_PRESET_COUNT     (256u)

// Button debounce time, milliseconds
#define BUTTON_DEBOUNCE_MS   (100u)

#define MIN_BPM              (1u)
#define MAX_BPM              (256u)
#define MAX_BEAT             (8u)
#define MIN_BEAT             (1u)



// Calculate the time in milliseconds for a single beat, based on BPM
#define BPM_TO_BEAT_TIME_MS(bpm) ((unsigned long) (60.0f / ((float) (bpm)) * 1000.0f))

// Checks if character is space or tab
#define IS_SPACE(c) ((c == ' ') || (c == '\t'))


// Enumerates all states the metronome can be in, button inputs
// mean different things for each of these states.
typedef enum
{
    STATE_METRONOME,         // Standard metronome mode
    STATE_PRESET,            // Preset playback mode
    STATE_PRESET_NAME_ENTRY  // Preset name entry mode
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
} buttons_e;

// Holds all data for a single metronome preset
typedef struct
{
    // Bits 0 through 7: BPM, 0-255 representing 1-256BPM
    // Bits 8 through 10: beat count, 0-7 representing 1-8 beats
    // Bits 11 through 15: reserved
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
    int gpio_pin;            // GPIO pin number for button
} button_info_t;

#if ENABLE_UART
// Holds all data required to handle a CLI command
typedef struct
{
    const char *cmd_word;
    void (*cmd_handler)(char *cmd_args);
} cli_command_t;

// Forward declaration of CLI command handlers
static void _help_cmd_handler(char *cmd_args);
static void _up_cmd_handler(char *cmd_args);
static void _down_cmd_handler(char *cmd_args);
static void _left_cmd_handler(char *cmd_args);
static void _right_cmd_handler(char *cmd_args);
static void _select_cmd_handler(char *cmd_args);
static void _mode_cmd_handler(char *cmd_args);
static void _add_del_cmd_handler(char *cmd_args);


// Number of CLI commands we can handle (must be manually synced with _cli_commands)
#define CLI_COMMAND_COUNT (8u)

static cli_command_t _cli_commands[CLI_COMMAND_COUNT] =
{
    {"help", _help_cmd_handler},
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
#endif // ENABLE_UART


// RAM copy of saved preset data
static metronome_presets_t _presets;

// State tracking/debouncing for all buttons
static volatile button_info_t _buttons[BUTTON_COUNT] = 
{
    {false, LOW, DEBOUNCE_IDLE, 0u, 11},  // BUTTON_UP
    {false, LOW, DEBOUNCE_IDLE, 0u, 12},  // BUTTON_DOWN
    {false, LOW, DEBOUNCE_IDLE, 0u, 13},  // BUTTON_LEFT
    {false, LOW, DEBOUNCE_IDLE, 0u, 14},  // BUTTON_RIGHT
    {false, LOW, DEBOUNCE_IDLE, 0u, 15},  // BUTTON_SELECT
    {false, LOW, DEBOUNCE_IDLE, 0u, 16},  // BUTTON_MODE
    {false, LOW, DEBOUNCE_IDLE, 0u, 17}   // BUTTON_ADD_DELETE
};

// Runtime values for BPM, beat count, metronome mode, and preset index
static volatile uint16_t _current_bpm = 123u;
static volatile uint16_t _current_beat_count = 4u;
static volatile metronome_state_e _current_state = STATE_METRONOME;
static volatile uint16_t _current_preset_index = 0u;

// Tracks whether metronome is running (in metronome mode)
static volatile bool _metronome_running = false;

// Tracks current beat in bar
static volatile uint16_t _current_beat = 0u;


// Tracks requested preset index (we only load in a new preset before the first beat of the bar)
static volatile uint16_t _requested_preset_index = 0u;
static volatile bool _preset_change_requested = false;


// Table of alphanumeric characters, used for preset name entry
static char _alphanum_table[3][20] =
{
    {'\0', '\0', 'q',  'w',  'e',  'r',  't',  'y',  'u',  'i',  'o',  'p',  '\0', '1',  '2',  '3',  '4',  '\0', '\0', '\0'},
    {'\0', '\0', 'a',  's',  'd',  'f',  'g',  'h',  'j',  'k',  'l',  '_',  '\0', '5',  '6',  '7',  '8',  '\0', '\0', '\0'},
    {'\0', '\0', '\0', 'z',  'x',  'c',  'v',  'b',  'n',  'm',  '\0', '<',  '\0', '\0', '9',  '0',  '\0', '\0', '\0', '\0'},
};


#if ENABLE_UART
#define LOG_INFO(fmt, ...) log("INFO", __func__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) log("ERROR", __func__, __LINE__, fmt, ##__VA_ARGS__)
#else
#define LOG_INFO(fmt, ...) {}
#define LOG_ERROR(fmt, ...) {}
#endif // ENABLE_UART


Arduino_CRC32 crc_generator;
FlashStorage(preset_store, metronome_presets_t);


// GPIO callback wrapper, initiates debounce for button pin if not already in progress
static void _gpio_callback(buttons_e button)
{
    if (DEBOUNCE_IDLE == _buttons[button].state)
    {
        _buttons[button].state = DEBOUNCE_START;
    }
}

// GPIO interrupt callbacks for buttons
void _up_button_callback(void) { _gpio_callback(BUTTON_UP); }
void _down_button_callback(void) { _gpio_callback(BUTTON_DOWN); }
void _left_button_callback(void) { _gpio_callback(BUTTON_LEFT); }
void _right_button_callback(void) { _gpio_callback(BUTTON_RIGHT); }
void _select_button_callback(void) { _gpio_callback(BUTTON_SELECT); }
void _mode_button_callback(void) { _gpio_callback(BUTTON_MODE); }
void _add_delete_button_callback(void) { _gpio_callback(BUTTON_ADD_DELETE); }

// Helper function to change state and log the transition
void _state_transition(metronome_state_e new_state)
{
    const char *statename = "";
    switch(new_state)
    {
        case STATE_METRONOME:
            statename = "METRONOME";
            break;
        case STATE_PRESET:
            statename = "PRESET";
            break;
        case STATE_PRESET_NAME_ENTRY:
            statename = "PRESET_NAME_ENTRY";
            break;
    }
    LOG_INFO("changing to state %s", statename);

    // Reset all button states
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        _buttons[i].pressed = false;
        _buttons[i].state = DEBOUNCE_IDLE;
    }

    // Update state value
    _current_state = new_state;
}

#if ENABLE_UART
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

// 'help' CLI command handler
void _help_cmd_handler(char *cmd_args)
{
    Serial.println("-------- CLI command reference ---------");
    Serial.println("help   - Show this printout");
    Serial.println("u      - Inject UP button press");
    Serial.println("d      - Inject DOWN button press");
    Serial.println("l      - Inject LEFT button press");
    Serial.println("r      - Inject RIGHT button press");
    Serial.println("s      - Inject SELECT button press");
    Serial.println("m      - Inject MODE button press");
    Serial.println("a      - Inject ADD/DEL button press");
    Serial.println("----------------------------------------");
}

// 'up' simulated button press CLI command handler
static void _up_cmd_handler(char *cmd_args)
{
    _buttons[BUTTON_UP].pressed = true;
    _buttons[BUTTON_UP].state = DEBOUNCE_FORCE;
}

// 'down' simulated button press CLI command handler
static void _down_cmd_handler(char *cmd_args)
{
    _buttons[BUTTON_DOWN].pressed = true;
    _buttons[BUTTON_DOWN].state = DEBOUNCE_FORCE;
}

// 'mode' simulated button press CLI command handler
static void _left_cmd_handler(char *cmd_args)
{
    _buttons[BUTTON_LEFT].pressed = true;
    _buttons[BUTTON_LEFT].state = DEBOUNCE_FORCE;
}

// 'select' simulated button press CLI command handler
static void _right_cmd_handler(char *cmd_args)
{
    _buttons[BUTTON_RIGHT].pressed = true;
    _buttons[BUTTON_RIGHT].state = DEBOUNCE_FORCE;
}

// 'beat' simulated button press CLI command handler
static void _select_cmd_handler(char *cmd_args)
{
    _buttons[BUTTON_SELECT].pressed = true;
    _buttons[BUTTON_SELECT].state = DEBOUNCE_FORCE;
}

// 'add/del' simulated button press CLI command handler
static void _mode_cmd_handler(char *cmd_args)
{
    _buttons[BUTTON_MODE].pressed = true;
    _buttons[BUTTON_MODE].state = DEBOUNCE_FORCE;
}

// 'start/stop' simulated button press CLI command handler
static void _add_del_cmd_handler(char *cmd_args)
{
    _buttons[BUTTON_ADD_DELETE].pressed = true;
    _buttons[BUTTON_ADD_DELETE].state = DEBOUNCE_FORCE;
}

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
#endif // ENABLE_UART

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

    *preset = ((_current_bpm - 1u) & 0xFFu) | (((_current_beat_count - 1u) & 0x7u) << 0x8u);
}

// Populate current BPM and beat count from a saved preset slot
static void _load_preset(uint16_t *preset)
{
    _current_bpm = ((*preset) & 0xFFu) + 1u;
    _current_beat_count = (((*preset) >> 8u) & 0x7u) + 1u;
}

// Draw current state to character LCD, based on current state
static void _update_char_lcd(void)
{
    if (STATE_METRONOME == _current_state)
    {
        // TODO
    }
    else if (STATE_PRESET == _current_state)
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
static void _stream_subbeat_sound()
{
    // TODO: implement
}

// Schedule timer interrupt to fire at a specific absolute time in milliseconds
static void _schedule_next_beat(void (*callback)(void), unsigned long beat_time_ms)
{
    // TODO: implement
}

void _start_streaming_next_beat(void)
{
    unsigned long now = millis();

    if (0u == _current_beat)
    {
        // First beat of bar, check if preset change was requested
        if (_preset_change_requested)
        {
            _load_preset(&_presets.presets[_requested_preset_index].settings);
            _current_preset_index = _requested_preset_index;
            _preset_change_requested = false;
        }

        _stream_beat_sound();
    }
    else
    {
        _stream_subbeat_sound();
    }

    if ((_current_beat_count - 1) == _current_beat)
    {
        _current_beat = 0u;
    }
    else
    {
        _current_beat += 1u;
    }

    _schedule_next_beat(_start_streaming_next_beat, now + BPM_TO_BEAT_TIME_MS(_current_bpm));
}

// Start metronome with current settings (enable timer interrupt)
static void _start_metronome(void)
{
    if (_metronome_running)
    {
        return;
    }

    _current_beat = 0u;
    _metronome_running = true;
    _start_streaming_next_beat();
}

// Stop metronome (disable timer interrupt)
static void _stop_metronome(void)
{
    if (!_metronome_running)
    {
        return;
    }

    _metronome_running = false;
    // TODO: stop timer
}

static void _max_preset_count_exceeded(void)
{
    // Display "no more room for presets" message and wait 2s
}

// Handle button inputs in "metronome" mode
static bool _handle_metronome_inputs(void)
{
    bool lcd_update_required = false;
    if (_buttons[BUTTON_UP].pressed)
    {
        // Increment BPM
        if (MAX_BPM > _current_bpm)
        {
            lcd_update_required = true;
            _current_bpm += 1u;
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

        lcd_update_required = true;
        _buttons[BUTTON_RIGHT].pressed = false;
    }
    else if (_buttons[BUTTON_SELECT].pressed)
    {
        // Start/stop metronome
        _metronome_running = !_metronome_running;
        if (_metronome_running)
        {
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
        // Load current preset data
        _load_preset(&_presets.presets[_current_preset_index].settings);

        // Switch to preset state (also clears button states)
        _state_transition(STATE_PRESET);
        lcd_update_required = true;
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
            _state_transition(STATE_PRESET_NAME_ENTRY);
            lcd_update_required = true;
        }

        lcd_update_required = true;
    }

    return lcd_update_required;
}

// Handle button inputs in "preset" mode
static bool _handle_preset_inputs(void)
{
    bool lcd_update_required = false;

    if (_buttons[BUTTON_UP].pressed)
    {
        // Increment requested preset index
        _requested_preset_index = (_current_preset_index + 1u) % _presets.preset_count;
        _preset_change_requested = true;
        _buttons[BUTTON_UP].pressed = false;
    }
    else if (_buttons[BUTTON_DOWN].pressed)
    {
        // Decrement requested preset index
        if (0u == _current_preset_index)
        {
            _requested_preset_index = _presets.preset_count - 1u;
        }
        else
        {
            _requested_preset_index = _current_preset_index - 1u;
        }

        _buttons[BUTTON_DOWN].pressed = false;
        _preset_change_requested = true;
    }
    else if (_buttons[BUTTON_SELECT].pressed)
    {
        // Start/stop metronome
        _metronome_running = !_metronome_running;
        if (_metronome_running)
        {
            // Load current preset
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
        lcd_update_required = true;
    }

    if (_preset_change_requested)
    {
        // Update the screen right away on preset change
        lcd_update_required = true;

        // If metronome not running, we can update metronome settings right now
        if (!_metronome_running)
        {
            _load_preset(&_presets.presets[_requested_preset_index].settings);
            _current_preset_index = _requested_preset_index;
            _preset_change_requested = false;
        }
    }

    return lcd_update_required;
}

// Handle button inputs in "preset name entry" mode
static bool _handle_name_entry_inputs(void)
{
    return false;
}

// Calls the correct input handler function based on current mode.
// Returns true if screen update is required.
static bool _handle_inputs(void)
{
    unsigned int buttons_pressed = 0u;

    // Handle debouncing for buttons
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        if (DEBOUNCE_IDLE == _buttons[i].state)
        {
            continue;
        }
        else if (DEBOUNCE_FORCE == _buttons[i].state)
        {
            _buttons[i].state = DEBOUNCE_IDLE;
            buttons_pressed += 1u;
            continue;
        }
        else if (DEBOUNCE_START == _buttons[i].state)
        {
            // Start debouncing this signal
            _buttons[i].start_ms = millis();
            _buttons[i].state = DEBOUNCE_ACTIVE;
        }
        else if (DEBOUNCE_ACTIVE == _buttons[i].state)
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

                _buttons[i].state =  DEBOUNCE_IDLE;
            }
        }
    }

    if (0u == buttons_pressed)
    {
        // If no buttons have been pressed, we can stop here
        return false;
    }

    // Handle any buttons that have been pressed
    if (STATE_METRONOME == _current_state)
    {
        return _handle_metronome_inputs();
    }
    else if (STATE_PRESET == _current_state)
    {
        return _handle_preset_inputs();
    }
    else if (STATE_PRESET_NAME_ENTRY == _current_state)
    {
        return _handle_name_entry_inputs();
    }
    else
    {
        // Error
    }

    return false;
}

void setup()
{
    // Required for snprintf to support floats :(
    asm(".global _printf_float");

    // Configure button input pins
    for (unsigned int i = 0u; i < BUTTON_COUNT; i++)
    {
        pinMode(_buttons[i].gpio_pin, INPUT);
    }

    Serial.begin(115200);

    LOG_INFO("power on");
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
#endif // ENABLE_FLASH_WRITE
    }
    else
    {
        LOG_INFO("%u presets stored", _presets.preset_count);
    }
}

void loop()
{
#if ENABLE_UART
    _handle_cli_commands();
#endif // ENABLE_UART

    if (_handle_inputs())
    {
        // _handle_inputs returns true if the char LCD needs to be re-drawn
        _update_char_lcd();
    }
}