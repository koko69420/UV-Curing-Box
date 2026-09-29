// ============================================================================
// UV PCB Curing Box Firmware
// ----------------------------------------------------------------------------
// MCU:        CH32V003A4M6 (RISC-V, WCH)
// Framework:  Community-PIO-CH32V Arduino core
//
// Revision 4 — reworked key bindings (# = add-time/resume/start, * =
// pause/reset/clear), raw-seconds manual entry with live clamping to an
// 8-minute cap, new presets, custom TIMER-idle and STOPWATCH displays.
//
// Highest-priority safety rule (see enterState() / updatePWM()):
//   The UV MOSFET on A2 is only ever allowed to be energized while the
//   timing engine is in the RUNNING state. Every other state forces it off,
//   both eagerly (the instant the state changes) and continuously (every
//   pass of updatePWM() re-checks this before doing anything else).
//
// State machine (unchanged):
//   BOOT -> IDLE -> READY -> RUNNING -> PAUSED -> FINISHED -> IDLE
// ============================================================================

#include <Arduino.h>
#include <stdint.h>

// ============================================================================
// USER-EDITABLE SETTINGS
// ----------------------------------------------------------------------------
// Everything you're likely to want to tune lives here, at the top, in one
// place: preset cure times, default brightness, default mode, and the
// MOSFET's switching polarity.
// ============================================================================

// ---- Preset cure times (A / B / C / D keys), in minutes + seconds ----
// Edit these pairs directly — no need to hand-convert to total seconds.
constexpr uint16_t PRESET_A_MINUTES = 1, PRESET_A_SECONDS_PART = 0;    // A = 1:00
constexpr uint16_t PRESET_B_MINUTES = 1, PRESET_B_SECONDS_PART = 30;   // B = 1:30
constexpr uint16_t PRESET_C_MINUTES = 2, PRESET_C_SECONDS_PART = 0;    // C = 2:00
constexpr uint16_t PRESET_D_MINUTES = 3, PRESET_D_SECONDS_PART = 0;    // D = 3:00

// ---- Brightness ----
constexpr uint8_t BRIGHTNESS_DEFAULT = 100; // 0-100 %, applied at boot

// ---- Timing mode the box boots into (TIMER counts down, STOPWATCH counts up) ----
enum class TimeMode : uint8_t {
    TIMER,
    STOPWATCH
};
constexpr TimeMode DEFAULT_TIME_MODE = TimeMode::TIMER;

// ---- UV MOSFET polarity ----
// Confirmed N-channel, wired as a standard low-side switch: gate HIGH turns
// the LEDs ON, gate LOW turns them OFF. Every single place in this file
// that touches the MOSFET pin goes through setUvGate() below, so if your
// wiring is ever the other way around, this is the ONLY line you need to
// flip.
constexpr bool MOSFET_ACTIVE_LOW = false;

// ============================================================================
// PIN ASSIGNMENTS
// ============================================================================
//
// NOTE: this framework (framework-arduinoch32v003, a ch32v003fun-based
// Arduino core) names its digital pin macros as just the port letter plus
// pin number — C1, C2, ... D4, A1, A2 — NOT PC1/PD4/PA1 as on some other
// Arduino cores.

// Keypad rows (outputs, driven low one at a time while scanning)
constexpr uint8_t PIN_ROW1 = C1;
constexpr uint8_t PIN_ROW2 = C2;
constexpr uint8_t PIN_ROW3 = C3;
constexpr uint8_t PIN_ROW4 = C4;

// Keypad columns (inputs with pull-ups, read low when a key is pressed)
constexpr uint8_t PIN_COL1 = C6;
constexpr uint8_t PIN_COL2 = C7;
constexpr uint8_t PIN_COL3 = D4;
constexpr uint8_t PIN_COL4 = D5;

// TM1637 display
constexpr uint8_t PIN_TM1637_CLK = D6;
constexpr uint8_t PIN_TM1637_DIO = A1;

// UV LED MOSFET gate (software PWM — see the PWM section below for why).
constexpr uint8_t PIN_UV_MOSFET = A2;

constexpr uint8_t KEYPAD_ROWS = 4;
constexpr uint8_t KEYPAD_COLS = 4;

// ============================================================================
// KEYPAD LAYOUT
// ============================================================================

static const char KEYPAD_MAP[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'},
};

static const uint8_t ROW_PINS[KEYPAD_ROWS] = {PIN_ROW1, PIN_ROW2, PIN_ROW3, PIN_ROW4};
static const uint8_t COL_PINS[KEYPAD_COLS] = {PIN_COL1, PIN_COL2, PIN_COL3, PIN_COL4};

constexpr char KEY_NONE = '\0';

// ============================================================================
// KEY BINDINGS (see dispatchTap() for the full state-by-state behavior)
// ============================================================================
//
//   #  Setup, no time set (IDLE):  Prime a default 5-second timer (TIMER
//                                  mode) / start immediately (STOPWATCH)
//      Setup, time set (READY):   Start
//      RUNNING:                   Add 5 seconds (clamped to the 8:00 cap)
//      PAUSED:                    Resume
//
//   *  Setup (IDLE/READY): Clear entry
//      RUNNING:            Pause
//      PAUSED:             Reset the timer to zero (back to IDLE)
//
//   In the brightness menu specifically: digits type a 0-100 percentage,
//   '*' clears the digits typed so far, '#' saves and exits. Holding D
//   again while already in the menu cancels (exits without saving).

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

constexpr uint32_t DEBOUNCE_MS      = 20;    // Debounce stabilization time
constexpr uint32_t LONG_PRESS_MS    = 2000;  // Hold time for the D / C secondary functions
constexpr uint32_t ROW_SETTLE_US    = 5;     // GPIO settle time between row switch and column read
constexpr uint32_t ONE_SECOND_MS    = 1000;  // Timer tick period
constexpr uint32_t COLON_BLINK_MS   = 500;   // Colon toggles twice per second while running
constexpr uint32_t FINISHED_DISPLAY_MS = 5000; // How long FINISHED shows before auto-return to IDLE
constexpr uint32_t FINISHED_BLINK_MS   = 500;  // Flash rate while FINISHED (TIMER mode only)
constexpr uint32_t LED_TOGGLE_FEEDBACK_MS = 1000; // How long the "on"/"off" overlay shows after 'D' in STOPWATCH mode

// Software PWM period for the UV MOSFET. See the big comment above
// updatePWM() for why this needs to be large (tens of ms), not small —
// short version: updatePWM() only samples once per loop() pass, and a
// TM1637 write can stall loop() for ~100-150us, so the "on" window at low
// brightness must stay comfortably bigger than that or pulses get skipped
// outright, which shows up as irregular flicker and unstable brightness.
// 20 ms (50 Hz) gives a huge margin at any brightness down to 1%, and this
// application has zero need for a visually-smooth high PWM frequency —
// nobody watches a closed curing box, only the total delivered dose (i.e.
// the time-average) matters.
constexpr uint32_t PWM_PERIOD_US    = 20000;

// ============================================================================
// DERIVED TIMER / PRESET CONSTANTS
// ============================================================================

// Hard ceiling shared by both modes: the longest a TIMER can be set to, and
// the point at which a STOPWATCH run auto-stops.
constexpr uint16_t MAX_TIMER_SECONDS = 8 * 60; // 8:00
constexpr uint16_t MAX_ENTRY_VALUE   = 9999;   // 4 digit rolling entry cap

constexpr uint16_t PRESET_A_SECONDS = (PRESET_A_MINUTES * 60) + PRESET_A_SECONDS_PART;
constexpr uint16_t PRESET_B_SECONDS = (PRESET_B_MINUTES * 60) + PRESET_B_SECONDS_PART;
constexpr uint16_t PRESET_C_SECONDS = (PRESET_C_MINUTES * 60) + PRESET_C_SECONDS_PART;
constexpr uint16_t PRESET_D_SECONDS = (PRESET_D_MINUTES * 60) + PRESET_D_SECONDS_PART;

static_assert(PRESET_A_SECONDS <= MAX_TIMER_SECONDS, "Preset A exceeds the 8:00 cap");
static_assert(PRESET_B_SECONDS <= MAX_TIMER_SECONDS, "Preset B exceeds the 8:00 cap");
static_assert(PRESET_C_SECONDS <= MAX_TIMER_SECONDS, "Preset C exceeds the 8:00 cap");
static_assert(PRESET_D_SECONDS <= MAX_TIMER_SECONDS, "Preset D exceeds the 8:00 cap");

// ============================================================================
// BRIGHTNESS CONSTANTS
// ============================================================================

constexpr uint8_t BRIGHTNESS_MIN = 0;
constexpr uint8_t BRIGHTNESS_MAX = 100;

// ============================================================================
// STATE MACHINE
// ============================================================================

enum class SystemState : uint8_t {
    BOOT,
    IDLE,
    READY,
    RUNNING,
    PAUSED,
    FINISHED
};

static SystemState currentState = SystemState::BOOT;
static TimeMode currentMode = DEFAULT_TIME_MODE;

// ============================================================================
// GLOBAL RUNTIME VARIABLES
// ============================================================================

// Timing engine. targetSeconds is the TIMER-mode goal (set via manual entry
// or a preset); timerSeconds is what's actually shown/ticking — it counts
// down from targetSeconds in TIMER mode, or up from zero in STOPWATCH mode.
static uint16_t targetSeconds = 0;
static uint16_t timerSeconds = 0;
static uint32_t lastSecondTickMs = 0;

// Manual digit entry buffer. This is a plain rolling raw number (NOT split
// into MM+SS digit positions) — see handleTimerEntryKey() for why. Only
// meaningful in TIMER mode.
static uint16_t entryValue = 0;

// Display blink state, shared between the RUNNING colon blink and the
// TIMER-mode FINISHED flash (the two states are mutually exclusive, so
// reusing one timer/flag pair avoids keeping duplicate state around).
static bool blinkVisible = false;
static uint32_t lastBlinkToggleMs = 0;

// FINISHED state auto-timeout bookkeeping.
static uint32_t finishedEnteredMs = 0;

// Brightness (0-100 %), kept in RAM as no EEPROM/Flash storage API is
// exposed by this core for the CH32V003. If persistent storage becomes
// available it can be added here (e.g. via a Flash-backed settings page).
static uint8_t brightness = BRIGHTNESS_DEFAULT;
static bool inBrightnessMenu = false;
static SystemState stateBeforeBrightnessMenu = SystemState::IDLE;

// Live-typed brightness percentage while the menu is open (0-100). Not
// committed to `brightness` until '#' (save) is pressed.
static uint8_t brightnessEntryValue = BRIGHTNESS_DEFAULT;
static bool brightnessEntryDirty = false; // true once the user has typed a digit

// ============================================================================
// SOFTWARE PWM STATE (UV MOSFET on A2)
// ============================================================================
//
// analogWrite() is deliberately NOT used here: on this core, hardware PWM
// on A2 is unreliable/unsupported (it's a documented problem pin). Rather
// than depend on undocumented, chip-variant-specific timer/channel register
// access, brightness is implemented as a software PWM: updatePWM() figures
// out, from micros(), whether the MOSFET should currently be on or off, and
// sets it — every loop() iteration.
//
// IMPORTANT: this is a *sampled* PWM, not a hardware-timer PWM. It is only
// as accurate as loop()'s call rate is fast and regular. loop() also scans
// the keypad and occasionally writes to the TM1637 (a handful of bit-banged
// transactions, each taking on the order of 100us). If the "on" window at
// the current brightness is smaller than one of those stalls, a whole pulse
// can be skipped, which shows up as irregular flicker and unstable
// perceived brightness — worse at low brightness, since the on-window
// shrinks toward (and eventually below) the size of a typical stall. This
// is why PWM_PERIOD_US is tens of milliseconds rather than ~1ms: it keeps
// the on-window comfortably larger than any realistic loop() stall at every
// brightness level, including very low ones.
//
// The pin is re-asserted every single call (not just on a computed change)
// as cheap extra robustness: it means any transient glitch on the gate
// line only lasts until the next loop() iteration rather than persisting
// until the next intended change.

static uint32_t pwmCycleStartUs = 0;
static bool pwmOutputState = false;

// Single point of contact for the MOSFET pin. Every other function in this
// file that needs to turn the LEDs on or off calls this — never
// digitalWrite(PIN_UV_MOSFET, ...) directly — so the active-low/active-high
// polarity is defined in exactly one place (MOSFET_ACTIVE_LOW above).
//
// NOTE: HIGH/LOW are passed directly (not via an intermediate uint8_t) on
// purpose — this core's digitalWrite() takes a strongly-typed PinStatus
// enum, and an intermediate uint8_t variable fails to implicitly convert
// ("invalid conversion from 'uint8_t' to 'PinStatus'").
static void setUvGate(bool energized) {
    if (MOSFET_ACTIVE_LOW) {
        digitalWrite(PIN_UV_MOSFET, energized ? LOW : HIGH);
    } else {
        digitalWrite(PIN_UV_MOSFET, energized ? HIGH : LOW);
    }
}

// ============================================================================
// KEYPAD SCANNING / DEBOUNCE STATE
// ============================================================================
//
// Timestamps here are stored as uint16_t (truncated millis()) rather than
// the full 32-bit millis() value to save RAM across the 16-key grid — the
// CH32V003 only has 2 KB of SRAM in total. This is safe because unsigned
// subtraction wraps correctly modulo 2^16, and every window we ever measure
// against these timestamps (DEBOUNCE_MS, LONG_PRESS_MS) is far shorter than
// the ~65.5 s a uint16_t can represent before it wraps.

struct KeyState {
    uint16_t lastChangeMs = 0;    // Truncated millis() of last raw state change
    uint16_t pressStartMs = 0;    // Truncated millis() when key became stable-pressed
    bool stablePressed = false;   // Debounced, currently-pressed state
    bool rawPressed = false;      // Instantaneous raw reading from last scan
    bool longPressFired = false;  // Whether the long-press event already fired this hold
};

static KeyState keyStates[KEYPAD_ROWS][KEYPAD_COLS];

// Events produced by scanKeypad() for the current loop iteration.
struct KeyEvent {
    char key = KEY_NONE;        // Key that generated the event ('\0' if none)
    bool pressed = false;       // True on the debounced press edge
    bool released = false;      // True on the debounced release edge
    bool longPress = false;     // True once, mid-hold, when LONG_PRESS_MS is crossed
    bool suppressTap = false;   // Set on the release event if longPress already fired this hold
};

// ============================================================================
// TM1637 BIT-BANGED DRIVER
// ============================================================================
//
// Minimal TM1637 driver implemented directly against the two GPIO pins.
// No external library dependency is required. delayMicroseconds() is used
// only here, for TM1637 bit timing, per the "no delay() anywhere else" rule.

namespace TM1637 {

constexpr uint8_t CMD_DATA_AUTO_INC = 0x40; // Write data, auto-increment address
constexpr uint8_t CMD_ADDR_BASE     = 0xC0; // Start address 0 (first digit)
constexpr uint8_t CMD_DISPLAY_CTRL  = 0x88; // Display ON + brightness bits (0-7)
constexpr uint8_t DISPLAY_MAX_BRIGHTNESS = 0x02; // TM1637 brightness level (0-7); lowered from max on purpose

// 7-segment encoding for characters used on this display.
// Segment bit order: 0=A,1=B,2=C,3=D,4=E,5=F,6=G (bit7 = decimal point / colon)
constexpr uint8_t SEG_BLANK = 0x00;
static const uint8_t DIGIT_SEGMENTS[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
};
constexpr uint8_t SEG_LETTER_B = 0x7C; // lowercase 'b', used in the brightness menu
constexpr uint8_t SEG_LETTER_T = 0x78; // lowercase 't', used for the TIMER-idle "t---" display
constexpr uint8_t SEG_LETTER_S = 0x6D; // uppercase 'S' (same glyph as digit 5), used in STOPWATCH mode
constexpr uint8_t SEG_DASH     = 0x40; // '-', used for "t---" and "S8:--"

inline void bitDelay() {
    // Short delay for TM1637 clock timing (datasheet allows > ~1us; some
    // clone TM1637 chips want more margin, so 5us is used here rather than
    // cutting it close).
    delayMicroseconds(5);
}

inline void clkHigh() { pinMode(PIN_TM1637_CLK, OUTPUT); digitalWrite(PIN_TM1637_CLK, HIGH); }
inline void clkLow()  { pinMode(PIN_TM1637_CLK, OUTPUT); digitalWrite(PIN_TM1637_CLK, LOW); }
inline void dioHigh() { pinMode(PIN_TM1637_DIO, OUTPUT); digitalWrite(PIN_TM1637_DIO, HIGH); }
inline void dioLow()  { pinMode(PIN_TM1637_DIO, OUTPUT); digitalWrite(PIN_TM1637_DIO, LOW); }

inline void start() {
    dioHigh();
    clkHigh();
    bitDelay();
    dioLow();
    bitDelay();
    clkLow();
    bitDelay();
}

inline void stop() {
    clkLow();
    bitDelay();
    dioLow();
    bitDelay();
    clkHigh();
    bitDelay();
    dioHigh();
    bitDelay();
}

// Writes one byte LSB-first and reads back the ACK bit (not strictly
// required, but kept for robustness / future error handling).
inline void writeByte(uint8_t data) {
    for (uint8_t bitIndex = 0; bitIndex < 8; ++bitIndex) {
        clkLow();
        if (data & 0x01) {
            dioHigh();
        } else {
            dioLow();
        }
        bitDelay();
        clkHigh();
        bitDelay();
        data >>= 1;
    }

    // ACK cycle: release DIO and pulse clock, then read the ack (ignored).
    clkLow();
    dioHigh();
    pinMode(PIN_TM1637_DIO, INPUT_PULLUP);
    bitDelay();
    clkHigh();
    bitDelay();
    clkLow();
    bitDelay();
    pinMode(PIN_TM1637_DIO, OUTPUT);
}

inline void setBrightnessMax() {
    start();
    writeByte(CMD_DISPLAY_CTRL | DISPLAY_MAX_BRIGHTNESS);
    stop();
}

// Sends four raw segment bytes (already including any decimal point /
// colon bit) to the display starting at digit 0.
inline void writeRaw(const uint8_t segments[4]) {
    start();
    writeByte(CMD_DATA_AUTO_INC);
    stop();

    start();
    writeByte(CMD_ADDR_BASE);
    for (uint8_t i = 0; i < 4; ++i) {
        writeByte(segments[i]);
    }
    stop();

    setBrightnessMax();
}

inline void init() {
    pinMode(PIN_TM1637_CLK, OUTPUT);
    pinMode(PIN_TM1637_DIO, OUTPUT);
    digitalWrite(PIN_TM1637_CLK, HIGH);
    digitalWrite(PIN_TM1637_DIO, HIGH);
    setBrightnessMax();
}

// Displays two 2-digit numbers (MM and SS) with an optional colon.
inline void showMinutesSeconds(uint8_t minutesValue, uint8_t secondsValue, bool colon) {
    uint8_t segments[4];
    segments[0] = DIGIT_SEGMENTS[(minutesValue / 10) % 10];
    segments[1] = DIGIT_SEGMENTS[minutesValue % 10];
    segments[2] = DIGIT_SEGMENTS[(secondsValue / 10) % 10];
    segments[3] = DIGIT_SEGMENTS[secondsValue % 10];

    // On common TM1637 clock modules the colon is wired to the decimal
    // point of the second digit position (index 1).
    if (colon) {
        segments[1] |= 0x80;
    }

    writeRaw(segments);
}

// Displays "b" followed by a 0-100 brightness value, e.g. "b100" / "b 50".
inline void showBrightnessMenu(uint8_t percentValue) {
    uint8_t segments[4];
    segments[0] = SEG_LETTER_B;

    if (percentValue >= 100) {
        segments[1] = DIGIT_SEGMENTS[1];
        segments[2] = DIGIT_SEGMENTS[0];
        segments[3] = DIGIT_SEGMENTS[0];
    } else {
        segments[1] = SEG_BLANK;
        segments[2] = DIGIT_SEGMENTS[(percentValue / 10) % 10];
        segments[3] = DIGIT_SEGMENTS[percentValue % 10];
    }

    writeRaw(segments);
}

// Blanks all four digits (used for the TIMER-mode FINISHED flash effect).
inline void clear() {
    uint8_t segments[4] = {SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK};
    writeRaw(segments);
}

} // namespace TM1637

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

static KeyEvent scanKeypad();
static void updateTimer();
static void updateFinishedTimeout();
static void updateDisplay();
static void updatePWM();
static void handleUI(const KeyEvent &event);
static void handleNormalKey(const KeyEvent &event);
static void handleBrightnessMenuKey(char key);
static void dispatchTap(char key);
static void handleTimerEntryKey(char key);
static void startTiming();
static void addFiveSeconds();
static void enterState(SystemState newState);
static void enterBrightnessMenu();
static void saveAndExitBrightnessMenu();
static void cancelBrightnessMenu();
static void landFromBrightnessMenu();
static bool isSetupState(SystemState state);
static bool isBrightnessMenuAllowed(SystemState state);
static bool isAmbiguousKey(char key);
static bool isUvActive();
static void forceUvOff();
static void resetEntry();
static void renderFinished();
static void renderIdleTimer();
static void renderStopwatch(uint16_t elapsedSeconds, bool colonOn);
static void renderStopwatchMax();

// ============================================================================
// SMALL STATE PREDICATES (kept centralized to avoid duplicated conditions)
// ============================================================================

static bool isSetupState(SystemState state) {
    return state == SystemState::IDLE || state == SystemState::READY;
}

static bool isBrightnessMenuAllowed(SystemState state) {
    // Brightness may be changed while IDLE, READY, or FINISHED — never while
    // RUNNING or PAUSED, so it can never interrupt an active curing cycle.
    return state == SystemState::IDLE ||
           state == SystemState::READY ||
           state == SystemState::FINISHED;
}

static bool isAmbiguousKey(char key) {
    // Keys that carry both a tap function and a distinct long-press
    // function; their tap action must wait until release so it doesn't
    // fire the instant a long-press turns out to be starting.
    return key == 'C' || key == 'D';
}

static bool isUvActive() {
    // THE single source of truth for whether the UV LEDs are allowed to be
    // energized at all. Everything else (PWM duty, brightness) is secondary
    // to this check.
    return currentState == SystemState::RUNNING;
}

static void forceUvOff() {
    setUvGate(false);
    pwmOutputState = false;
}

// ============================================================================
// STATE TRANSITION HELPER
// ============================================================================

static void enterState(SystemState newState) {
    currentState = newState;

    // Highest-priority safety rule: force the MOSFET off the instant we are
    // not RUNNING, rather than waiting for the next updatePWM() pass (which,
    // in practice, is only ever a fraction of a millisecond away, but this
    // makes the guarantee explicit and independent of call ordering).
    if (newState != SystemState::RUNNING) {
        forceUvOff();
    }

    switch (newState) {
        case SystemState::IDLE:
            resetEntry();
            targetSeconds = 0;
            timerSeconds = 0;
            blinkVisible = false;
            break;

        case SystemState::READY:
            blinkVisible = false;
            break;

        case SystemState::RUNNING:
            lastSecondTickMs = millis();
            lastBlinkToggleMs = millis();
            pwmCycleStartUs = micros();
            blinkVisible = true;
            break;

        case SystemState::PAUSED:
            blinkVisible = false;
            break;

        case SystemState::FINISHED:
            finishedEnteredMs = millis();
            lastBlinkToggleMs = millis();
            blinkVisible = true; // Start the TIMER-mode flash "on"
            break;

        case SystemState::BOOT:
        default:
            break;
    }
}

// ============================================================================
// MANUAL ENTRY HELPERS
// ============================================================================

static void resetEntry() {
    entryValue = 0;
}

// Handles a tap on a TIMER-mode entry key (digits or A/B/C/D presets).
// Only ever called while in a setup state (IDLE/READY) and TIMER mode.
static void handleTimerEntryKey(char key) {
    if (key >= '0' && key <= '9') {
        const uint16_t digit = static_cast<uint16_t>(key - '0');
        entryValue = (entryValue * 10 + digit) % (MAX_ENTRY_VALUE + 1);

        // Split the typed digits the normal MM:SS way — last two digits are
        // seconds, everything left of that is minutes — but unlike the old
        // version, a seconds-part over 59 is NOT rejected; it's just let to
        // carry into extra minutes via the arithmetic below. That's what
        // makes "80" naturally become 80 seconds = 1:20 instead of being
        // stuck as an invalid "0 min 80 sec", while "130" still reads as
        // the normal 1:30. Any total is then clamped to the 8:00 cap, live,
        // on every keystroke.
        const uint16_t minutesPart = entryValue / 100;
        const uint16_t secondsPart = entryValue % 100;
        uint16_t totalSeconds = minutesPart * 60 + secondsPart;
        if (totalSeconds > MAX_TIMER_SECONDS) {
            totalSeconds = MAX_TIMER_SECONDS;
        }

        targetSeconds = totalSeconds;
        timerSeconds = totalSeconds;
        enterState(SystemState::READY);
        return;
    }

    uint16_t presetSeconds;
    switch (key) {
        case 'A': presetSeconds = PRESET_A_SECONDS; break;
        case 'B': presetSeconds = PRESET_B_SECONDS; break;
        case 'C': presetSeconds = PRESET_C_SECONDS; break;
        case 'D': presetSeconds = PRESET_D_SECONDS; break;
        default:  return; // Not an entry key
    }

    entryValue = 0; // Presets bypass/replace the manual digit buffer
    targetSeconds = presetSeconds;
    timerSeconds = presetSeconds;
    enterState(SystemState::READY);
}

// Starts the timing engine from a setup state, per the current mode.
//
// TIMER mode with nothing set yet (targetSeconds == 0): rather than doing
// nothing, this primes a default 5-second timer and moves to READY — a
// second '#' press (which re-enters this function, now with
// targetSeconds != 0) actually starts it.
static void startTiming() {
    if (currentMode == TimeMode::TIMER) {
        if (targetSeconds == 0) {
            targetSeconds = 5;
            timerSeconds = 5;
            enterState(SystemState::READY);
            return;
        }
        timerSeconds = targetSeconds;
    } else {
        timerSeconds = 0; // Stopwatch always starts fresh from zero
    }
    enterState(SystemState::RUNNING);
}

// '#' while RUNNING: extend the current run by 5 seconds, clamped to the
// shared 8:00 cap. In STOPWATCH mode, hitting that cap this way means the
// run is complete, same as the natural tick-based ceiling check in
// updateTimer() — it does NOT just sit at the cap while still "running".
// In TIMER mode, being at the cap doesn't mean "done" (zero does), so it
// just clamps and keeps counting down normally.
static void addFiveSeconds() {
    constexpr uint16_t ADD_SECONDS = 5;
    uint16_t newValue = timerSeconds + ADD_SECONDS;
    if (newValue > MAX_TIMER_SECONDS) {
        newValue = MAX_TIMER_SECONDS;
    }
    timerSeconds = newValue;

    if (currentMode == TimeMode::STOPWATCH && timerSeconds >= MAX_TIMER_SECONDS) {
        enterState(SystemState::FINISHED);
    }
}

// ============================================================================
// BRIGHTNESS MENU HELPERS
// ============================================================================

static void enterBrightnessMenu() {
    inBrightnessMenu = true;
    stateBeforeBrightnessMenu = currentState;
    brightnessEntryValue = brightness; // Show the current value until the user types
    brightnessEntryDirty = false;
}

static void landFromBrightnessMenu() {
    inBrightnessMenu = false;

    // FINISHED is a transient, auto-expiring display state; re-entering it
    // after a menu detour (which may have taken an arbitrary amount of time)
    // would revive a stale flash/timeout. Land back in IDLE instead.
    SystemState returnState = (stateBeforeBrightnessMenu == SystemState::FINISHED)
                                   ? SystemState::IDLE
                                   : stateBeforeBrightnessMenu;
    enterState(returnState);
}

static void saveAndExitBrightnessMenu() {
    brightness = brightnessEntryValue;
    landFromBrightnessMenu();
}

static void cancelBrightnessMenu() {
    // brightnessEntryValue is simply discarded; `brightness` is untouched.
    landFromBrightnessMenu();
}

static void handleBrightnessMenuKey(char key) {
    if (key >= '0' && key <= '9') {
        const uint8_t digit = static_cast<uint8_t>(key - '0');

        // First keystroke after opening the menu (or after a clear) starts
        // a fresh entry instead of appending to the previously-saved value.
        const uint8_t base = brightnessEntryDirty ? brightnessEntryValue : 0;
        const uint16_t candidate = static_cast<uint16_t>(base) * 10 + digit;

        if (candidate <= BRIGHTNESS_MAX) {
            brightnessEntryValue = static_cast<uint8_t>(candidate);
            brightnessEntryDirty = true;
        }
        // Candidate > 100 is impossible as a percentage: ignore the digit,
        // keep whatever was typed so far.
        return;
    }

    switch (key) {
        case '*': // Clear the digits typed so far, stay in the menu
            brightnessEntryValue = 0;
            brightnessEntryDirty = true;
            break;

        case '#': // Save and exit
            saveAndExitBrightnessMenu();
            break;

        default:
            // Ignored while in the brightness menu.
            break;
    }
}

// ============================================================================
// KEY DISPATCH
// ============================================================================

// Handles a resolved "tap" — i.e. we already know this is the key's normal
// (non-long-press) action, and it's safe to act on it now.
static void dispatchTap(char key) {
    if (isSetupState(currentState)) {
        if (key == '*') {
            enterState(SystemState::IDLE); // Clear entry / start over
            return;
        }
        if (key == '#') {
            startTiming();
            return;
        }
        if (currentMode == TimeMode::TIMER) {
            handleTimerEntryKey(key);
        }
        // Digits and presets are ignored in STOPWATCH mode.
        return;
    }

    switch (currentState) {
        case SystemState::RUNNING:
            if (key == '#') {
                addFiveSeconds();
            } else if (key == '*') {
                enterState(SystemState::PAUSED);
            }
            break;

        case SystemState::PAUSED:
            if (key == '#') {
                enterState(SystemState::RUNNING); // Resume
            } else if (key == '*') {
                enterState(SystemState::IDLE); // Reset to zero
            }
            break;

        case SystemState::FINISHED:
            // Any key dismisses FINISHED early (in addition to the 5 s
            // auto-timeout handled in updateFinishedTimeout()).
            enterState(SystemState::IDLE);
            break;

        case SystemState::BOOT:
        case SystemState::IDLE:
        case SystemState::READY:
        default:
            break; // Setup states are already handled above
    }
}

static void handleNormalKey(const KeyEvent &event) {
    if (event.key == KEY_NONE) {
        return;
    }

    // --- Long-press actions: fire once, immediately, mid-hold ---
    if (event.longPress) {
        if (event.key == 'D') {
            if (inBrightnessMenu) {
                cancelBrightnessMenu(); // Second long-press-D exits without saving
            } else if (isBrightnessMenuAllowed(currentState)) {
                enterBrightnessMenu();
            }
            // Explicitly does NOTHING while RUNNING or PAUSED.
        } else if (event.key == 'C') {
            if (currentState == SystemState::IDLE && !inBrightnessMenu) {
                currentMode = (currentMode == TimeMode::TIMER) ? TimeMode::STOPWATCH : TimeMode::TIMER;
                enterState(SystemState::IDLE); // Clear any stale entry, redraw
            }
        }
        return;
    }

    // --- Tap actions ---
    // Ambiguous keys (C, D) resolve on release, once we're certain no
    // long-press fired during the hold. Every other key resolves
    // immediately on press for a responsive feel.
    const bool isTapEdge = isAmbiguousKey(event.key)
                                ? (event.released && !event.suppressTap)
                                : event.pressed;
    if (!isTapEdge) {
        return;
    }

    if (inBrightnessMenu) {
        handleBrightnessMenuKey(event.key);
        return;
    }

    dispatchTap(event.key);
}

// ============================================================================
// KEYPAD SCANNING
// ============================================================================

static KeyEvent scanKeypad() {
    KeyEvent event;
    const uint16_t now = static_cast<uint16_t>(millis());

    // Ensure all rows are idle (HIGH / not driving) before scanning.
    for (uint8_t r = 0; r < KEYPAD_ROWS; ++r) {
        pinMode(ROW_PINS[r], OUTPUT);
        digitalWrite(ROW_PINS[r], HIGH);
    }

    for (uint8_t r = 0; r < KEYPAD_ROWS; ++r) {
        digitalWrite(ROW_PINS[r], LOW);
        delayMicroseconds(ROW_SETTLE_US); // Let the row line settle before reading columns

        for (uint8_t c = 0; c < KEYPAD_COLS; ++c) {
            bool pressedNow = (digitalRead(COL_PINS[c]) == LOW);
            KeyState &ks = keyStates[r][c];

            if (pressedNow != ks.rawPressed) {
                ks.rawPressed = pressedNow;
                ks.lastChangeMs = now;
            }

            // Debounce: only commit the raw state once it has been stable
            // for DEBOUNCE_MS.
            if (static_cast<uint16_t>(now - ks.lastChangeMs) >= DEBOUNCE_MS &&
                ks.rawPressed != ks.stablePressed) {
                ks.stablePressed = ks.rawPressed;

                if (ks.stablePressed) {
                    // Debounced press edge.
                    ks.pressStartMs = now;
                    ks.longPressFired = false;
                    event.key = KEYPAD_MAP[r][c];
                    event.pressed = true;
                } else {
                    // Debounced release edge.
                    event.key = KEYPAD_MAP[r][c];
                    event.released = true;
                    event.suppressTap = ks.longPressFired;
                }
            }

            // Long-press detection for currently stable-pressed keys.
            if (ks.stablePressed && !ks.longPressFired &&
                static_cast<uint16_t>(now - ks.pressStartMs) >= LONG_PRESS_MS) {
                ks.longPressFired = true;
                event.key = KEYPAD_MAP[r][c];
                event.longPress = true;
            }
        }

        digitalWrite(ROW_PINS[r], HIGH);
    }

    return event;
}

// ============================================================================
// TIMER UPDATE
// ============================================================================

static void updateTimer() {
    if (currentState != SystemState::RUNNING) {
        return;
    }

    const uint32_t now = millis();
    // (now - lastSecondTickMs) is correct across millis() overflow because
    // unsigned subtraction wraps modulo 2^32, matching the wraparound of
    // millis() itself.
    if ((now - lastSecondTickMs) >= ONE_SECOND_MS) {
        lastSecondTickMs += ONE_SECOND_MS; // Accumulate ticks, no drift over time

        if (currentMode == TimeMode::TIMER) {
            if (timerSeconds > 0) {
                timerSeconds--;
            }
            if (timerSeconds == 0) {
                enterState(SystemState::FINISHED);
            }
        } else { // STOPWATCH
            if (timerSeconds < MAX_TIMER_SECONDS) {
                timerSeconds++;
            }
            if (timerSeconds >= MAX_TIMER_SECONDS) {
                enterState(SystemState::FINISHED);
            }
        }
    }
}

static void updateFinishedTimeout() {
    if (currentState != SystemState::FINISHED) {
        return;
    }
    if ((millis() - finishedEnteredMs) >= FINISHED_DISPLAY_MS) {
        enterState(SystemState::IDLE);
    }
}

// ============================================================================
// DISPLAY UPDATE
// ============================================================================

// ----------------------------------------------------------------------------
// Dirty-checked rendering.
//
// updateDisplay() runs every loop() iteration, and loop() itself has no
// pacing — on this MCU it can easily run thousands of times a second. Left
// unguarded, that would mean TM1637::writeRaw() (bit-banged start/byte/stop
// transactions) firing thousands of times a second too, nonstop, even when
// nothing on the display has actually changed — which stresses the bus
// timing and is a likely cause of segments randomly flashing on cheap
// TM1637 clones. The fix is a plain dirty-check: remember what was last
// actually sent, and only issue a new transaction when the content
// genuinely changes.

enum class DisplayFrameKind : uint8_t {
    NONE,
    TIME,           // Standard TIMER-mode MM:SS
    MENU,           // Brightness menu "bXXX"
    BLANK,          // All segments off (TIMER FINISHED flash, "off" phase)
    IDLE_TIMER,     // TIMER-mode IDLE placeholder "t---"
    STOPWATCH,      // Normal STOPWATCH running/paused/idle display
    STOPWATCH_MAX   // STOPWATCH FINISHED (hit the 8:00 cap): "S8:--"
};

static DisplayFrameKind lastFrameKind = DisplayFrameKind::NONE;
static uint8_t lastFrameA = 0;     // TIME/STOPWATCH: minutes : MENU: percentage
static uint8_t lastFrameB = 0;     // TIME/STOPWATCH: seconds
static bool lastFrameColon = false; // TIME/STOPWATCH only

static void renderTime(uint16_t totalSeconds, bool colonOn) {
    const uint8_t minutesPart = static_cast<uint8_t>(totalSeconds / 60);
    const uint8_t secondsPart = static_cast<uint8_t>(totalSeconds % 60);

    if (lastFrameKind == DisplayFrameKind::TIME &&
        lastFrameA == minutesPart && lastFrameB == secondsPart && lastFrameColon == colonOn) {
        return; // Unchanged since the last frame — skip the TM1637 write entirely
    }

    TM1637::showMinutesSeconds(minutesPart, secondsPart, colonOn);
    lastFrameKind = DisplayFrameKind::TIME;
    lastFrameA = minutesPart;
    lastFrameB = secondsPart;
    lastFrameColon = colonOn;
}

static void renderMenu(uint8_t percentValue) {
    if (lastFrameKind == DisplayFrameKind::MENU && lastFrameA == percentValue) {
        return;
    }

    TM1637::showBrightnessMenu(percentValue);
    lastFrameKind = DisplayFrameKind::MENU;
    lastFrameA = percentValue;
}

static void renderBlank() {
    if (lastFrameKind == DisplayFrameKind::BLANK) {
        return;
    }

    TM1637::clear();
    lastFrameKind = DisplayFrameKind::BLANK;
}

// TIMER-mode IDLE placeholder: "t---" (no colon).
static void renderIdleTimer() {
    if (lastFrameKind == DisplayFrameKind::IDLE_TIMER) {
        return;
    }

    const uint8_t segments[4] = {
        TM1637::SEG_LETTER_T,
        TM1637::SEG_DASH,
        TM1637::SEG_DASH,
        TM1637::SEG_DASH,
    };
    TM1637::writeRaw(segments);
    lastFrameKind = DisplayFrameKind::IDLE_TIMER;
}

// STOPWATCH mode, not yet at the cap: "S" + (blank until 1 min, then the
// minutes digit) + ":" + seconds.
static void renderStopwatch(uint16_t elapsedSeconds, bool colonOn) {
    const uint8_t minutesPart = static_cast<uint8_t>(elapsedSeconds / 60);
    const uint8_t secondsPart = static_cast<uint8_t>(elapsedSeconds % 60);

    if (lastFrameKind == DisplayFrameKind::STOPWATCH &&
        lastFrameA == minutesPart && lastFrameB == secondsPart && lastFrameColon == colonOn) {
        return;
    }

    uint8_t segments[4];
    segments[0] = TM1637::SEG_LETTER_S;
    segments[1] = (minutesPart == 0) ? TM1637::SEG_BLANK : TM1637::DIGIT_SEGMENTS[minutesPart];
    segments[2] = TM1637::DIGIT_SEGMENTS[(secondsPart / 10) % 10];
    segments[3] = TM1637::DIGIT_SEGMENTS[secondsPart % 10];
    if (colonOn) {
        segments[1] |= 0x80;
    }

    TM1637::writeRaw(segments);
    lastFrameKind = DisplayFrameKind::STOPWATCH;
    lastFrameA = minutesPart;
    lastFrameB = secondsPart;
    lastFrameColon = colonOn;
}

// STOPWATCH mode, hit the 8:00 cap: static "S8:--".
static void renderStopwatchMax() {
    if (lastFrameKind == DisplayFrameKind::STOPWATCH_MAX) {
        return;
    }

    const uint8_t segments[4] = {
        TM1637::SEG_LETTER_S,
        static_cast<uint8_t>(TM1637::DIGIT_SEGMENTS[8] | 0x80), // '8' with the colon lit
        TM1637::SEG_DASH,
        TM1637::SEG_DASH,
    };
    TM1637::writeRaw(segments);
    lastFrameKind = DisplayFrameKind::STOPWATCH_MAX;
}

// FINISHED has a different look per mode: TIMER flashes "0000" (no colon)
// for FINISHED_DISPLAY_MS; STOPWATCH shows a static "S8:--" the whole time.
static void renderFinished() {
    if (currentMode == TimeMode::STOPWATCH) {
        renderStopwatchMax();
        return;
    }

    const uint32_t now = millis();
    if ((now - lastBlinkToggleMs) >= FINISHED_BLINK_MS) {
        lastBlinkToggleMs += FINISHED_BLINK_MS;
        blinkVisible = !blinkVisible;
    }
    if (blinkVisible) {
        renderTime(timerSeconds, false);
    } else {
        renderBlank();
    }
}

static void updateDisplay() {
    if (inBrightnessMenu) {
        renderMenu(brightnessEntryValue);
        return;
    }

    if (currentState == SystemState::FINISHED) {
        renderFinished();
        return;
    }

    // Unified colon rule for everything else:
    //   RUNNING          -> blinking
    //   READY / PAUSED   -> solid on
    //   IDLE             -> off
    bool colonOn = false;
    if (currentState == SystemState::RUNNING) {
        const uint32_t now = millis();
        if ((now - lastBlinkToggleMs) >= COLON_BLINK_MS) {
            lastBlinkToggleMs += COLON_BLINK_MS;
            blinkVisible = !blinkVisible;
        }
        colonOn = blinkVisible;
    } else if (currentState == SystemState::READY || currentState == SystemState::PAUSED) {
        colonOn = true;
    }

    if (currentMode == TimeMode::STOPWATCH) {
        renderStopwatch(timerSeconds, colonOn);
        return;
    }

    // TIMER mode
    if (currentState == SystemState::IDLE) {
        renderIdleTimer();
        return;
    }

    renderTime(timerSeconds, colonOn);
}

// ============================================================================
// PWM UPDATE (software PWM on the UV MOSFET pin)
// ============================================================================

static void updatePWM() {
    // Highest-priority safety check, repeated every call: the UV LEDs may
    // only ever be on while the timing engine is actively RUNNING.
    if (!isUvActive()) {
        forceUvOff();
        return;
    }

    if (brightness == BRIGHTNESS_MIN) {
        forceUvOff();
        return;
    }

    if (brightness >= BRIGHTNESS_MAX) {
        setUvGate(true);
        pwmOutputState = true;
        return;
    }

    const uint32_t now = micros();

    // Re-derive our position within the current PWM period directly via
    // modulo, rather than incrementally "catching up" one period at a time.
    // This matters because pwmCycleStartUs can go stale for a long time
    // (e.g. the box sits IDLE for ten minutes, where updatePWM() short-
    // circuits through forceUvOff() every call and never touches
    // pwmCycleStartUs at all). With a single-step catch-up, resuming after
    // a long idle gap could take minutes of real time to resync — during
    // which the duty comparison below would never see elapsedInCycle fall
    // below onTimeUs, so the LEDs would appear to just never turn on at any
    // brightness under 100%. Modulo resolves this in one step, always.
    const uint32_t elapsedInCycle = (now - pwmCycleStartUs) % PWM_PERIOD_US;
    pwmCycleStartUs = now - elapsedInCycle; // Realign to the start of this cycle

    const uint32_t onTimeUs = (static_cast<uint32_t>(PWM_PERIOD_US) * brightness) / 100;
    const bool shouldBeOn = (elapsedInCycle < onTimeUs);

    // Re-assert every call (not just on change) — see the comment on
    // pwmOutputState above for why: it bounds how long any transient
    // glitch on the gate line can persist.
    setUvGate(shouldBeOn);
    pwmOutputState = shouldBeOn;
}

// ============================================================================
// UI DISPATCH
// ============================================================================

static void handleUI(const KeyEvent &event) {
    if (event.key == KEY_NONE) {
        return;
    }
    handleNormalKey(event);
}

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

void setup() {
    // Keypad rows: outputs, idle HIGH.
    for (uint8_t r = 0; r < KEYPAD_ROWS; ++r) {
        pinMode(ROW_PINS[r], OUTPUT);
        digitalWrite(ROW_PINS[r], HIGH);
    }

    // Keypad columns: inputs with pull-ups (key press pulls the line low).
    for (uint8_t c = 0; c < KEYPAD_COLS; ++c) {
        pinMode(COL_PINS[c], INPUT_PULLUP);
    }

    // UV MOSFET: output, forced off at boot — LED OFF is the very first
    // thing this firmware guarantees.
    pinMode(PIN_UV_MOSFET, OUTPUT);
    forceUvOff();

    // Display init.
    TM1637::init();

    pwmCycleStartUs = micros();

    enterState(SystemState::IDLE); // Also forces the MOSFET off again, defensively
}

void loop() {
    const KeyEvent event = scanKeypad();
    handleUI(event);
    updateTimer();
    updateFinishedTimeout();
    updateDisplay();
    updatePWM();
}