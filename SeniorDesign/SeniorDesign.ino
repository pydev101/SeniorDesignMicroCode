#include <I2S.h>
#include <pio_i2s.pio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "st7789_display.hpp"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <cstdint>
#include <atomic>
#include "hardware/timer.h"
#include "hardware/adc.h"

// --- Configuration -----------------------------------------------------

// Serial - Owned by Core 0

// Digital IN  - Owned by Core 0
#define DMUX_OUT 3
#define OCT_DOWN_P 14
#define OCT_UP_P 15

// Digital OUT - Owned by Core 0
#define DMUX_S0 6
#define DMUX_S1 5
#define DMUX_S2 4
#define DMUX_S3 2
#define DMUX_S4 1
#define DMUX_S5 0

const uint8_t DMUXIN_PINS[] = {DMUX_S0, DMUX_S1, DMUX_S2, DMUX_S3, DMUX_S4, DMUX_S5};
uint32_t DMUXIN_MASKS[6];

#define AMUX_S0 19
#define AMUX_S1 20
#define AMUX_S2 21

const uint8_t AMUXIN_PINS[] = {AMUX_S0, AMUX_S1, AMUX_S2};
uint32_t AMUXIN_MASKS[3];

// Screen - Owned by Core 0
#define LCD_MISO 8
#define LCD_CS 9
#define LCD_SCK 10
#define LCD_MOSI 11
#define LCD_RST 12
#define LCD_RS 13

//Analog IN - Owned by Core 0
// ADC 1
#define AMUX_OUT 27
// ADC 2
#define AUDIO_IN 28

// I2S - Owned by Core 1
#define SCK_PIN 16
#define WS_PIN 17 // pWS is DEFINED as pBCLK+1
#define SD_PIN 18

static Display disp(spi1, LCD_SCK, LCD_MOSI, LCD_CS, LCD_RS, LCD_RST);

constexpr int SAMPLE_RATE = 44100;
constexpr int MAX_AMPLITUDE = 32767;
constexpr float SAMPLE_PERIOD = 1.0 / SAMPLE_RATE;
constexpr int GENERATION_SAMPLES = 128;
#define MIN_AMP_ANALOG 0.0000001f

constexpr float TWOPI =  2*PI;

float sampleValues[GENERATION_SAMPLES];
int16_t samples[GENERATION_SAMPLES];

I2S i2s(OUTPUT, SCK_PIN, SD_PIN);

class Note {
public:
    float freq, A, D, S, R;
    float ASlope, DSlope, RSlope;
    float phase, phaseStep;
    int mode; // 0=Silent, 1=A, 2=D, 3=S, 4=R
    float amp;
    bool inputPressed;

    Note(float f, float a, float d, float s, float r) {
        updateFrequency(f);
        updateControls(a, d, s, r);
        mode = 0;
        amp = 0;
        inputPressed = false;
    }

    void updateFrequency(float f){
        freq = (f < 0) ? 0 : f;
        phase = 0;
        phaseStep = TWOPI*freq*SAMPLE_PERIOD;
    }

    void updateControls(float a, float d, float s, float r) {
        A = (a < 0) ? 0 : a;
        D = (d < 0) ? 0 : d;
        S = (s < 0.0f) ? 0.0f : (s > 1.0f ? 1.0f : s);
        R = (r < 0) ? 0 : r;

        ASlope = SAMPLE_PERIOD / A;
        DSlope = ((S - 1.0f) / D) * SAMPLE_PERIOD;
        RSlope = -1.0f * (S / R) * SAMPLE_PERIOD;

        amp = 0;
        mode = 0; // TODO Change how this works for user quality; may only stay in mode 3 if already in mode 3
    }

    void updateInput(bool isKeyPress){
        if(isKeyPress != inputPressed){
            inputPressed = isKeyPress;

            if(isKeyPress){
                mode = 1;
            }else{
                mode = 4;
            }
        }
    }

    void getSample() {
        if (mode == 0) return;

        // TODO Linear amp function may sound clunky could look into expoential functions
        for (int i = 0; i < GENERATION_SAMPLES; i++) {
            if (mode == 1) { // Attack
                amp += ASlope;
                if (amp >= 1.0f) { amp = 1.0f; mode = 2; }
            } else if (mode == 2) { // Decay
                amp += DSlope;
                if (amp <= S) { amp = S; mode = 3; }
            } else if (mode == 4) { // Release
                amp += RSlope;
                if (amp <= MIN_AMP_ANALOG) {
                    amp = 0; 
                    mode = 0; 
                    phase = 0;
                    return;
                }
            }

            //sampleValues[i] += amp * sinf(phase); // TODO Sine lookup table or parabolic approx if pure sine function is too slow when several notes play

            constexpr float B = 4.0f / 3.14159265f;
            constexpr float C = -4.0f / (3.14159265f * 3.14159265f);
            constexpr float P = 0.225f;
            float y = (B * phase + C * phase * std::abs(phase));
            y = P * (y * std::abs(y) - y) + y;   
            sampleValues[i] += amp * y;

            phase += phaseStep;
            if (phase >= PI) phase -= TWOPI; // Bound between -PI and PI
        }
    }
};

int getKey(int idx) {
    int reg = 0;

    if (idx < 24) {
        reg = idx;
    } else {
        reg = (idx - 21) << 3;
    }

    // Direct Register Write Loop
    for (int b = 0; b < 6; b++) {
        if ((reg >> b) & 1) {
            sio_hw->gpio_set = DMUXIN_MASKS[b]; // Atomic SET
        } else {
            sio_hw->gpio_clr = DMUXIN_MASKS[b]; // Atomic CLEAR
        }
    }

    // Waste time
    for(int i=0; i<300; i++){
        asm volatile ("NOP");
    }

    // Fast Read: Check if the DMUX_OUT pin bit is set in the input register
    return (sio_hw->gpio_in & (1ul << DMUX_OUT)) ? 1 : 0;
}

float rawAnalogRead[4];

void readAnalog() {
    sio_hw->gpio_clr = AMUXIN_MASKS[2];  // S2 always 0 for channels 0-3

    for (int ch = 0; ch < 4; ch++) {
        // Set S0 from bit 0 of ch
        if (ch & 0x01) {
            sio_hw->gpio_set = AMUXIN_MASKS[0];
        } else {
            sio_hw->gpio_clr = AMUXIN_MASKS[0];
        }

        // Set S1 from bit 1 of ch
        if (ch & 0x02) {
            sio_hw->gpio_set = AMUXIN_MASKS[1];
        } else {
            sio_hw->gpio_clr = AMUXIN_MASKS[1];
        }

        delayMicroseconds(50);    // allow AMUX output to settle
        analogRead(AMUX_OUT);     // dummy read to flush ADC sample-and-hold
        delayMicroseconds(10);

        rawAnalogRead[ch] = analogRead(AMUX_OUT) / 1023.0;
    }
}

typedef struct {
    unsigned long int keyInputs;
    int lastKey;
    float A;
    float D;
    float S;
    float R;
} SharedInfo;

std::atomic<SharedInfo> SHARED_inputs;
std::atomic<uint8_t> SHARED_flags;
SharedInfo oldInfo = {0, 0, 0, 0, 0, 0};

volatile bool updateInputFlag = false;
volatile bool updateScreenFlag = false;

// Humans feel lag in >15mS
const int scanPeriod = 5000; //uS or slower (Must be slower than the funtion, but faster than at least 5mS)
const int screenPeriod = 10000; //uS or slower (Must be slower than the funtion, but faster than at least 5mS)
constexpr int numNotes = 25;

const int rawInputLen = 3;
unsigned long int lastRAWInputs[rawInputLen];
unsigned int lawRAWInputsIndex = 0;
int lastKeyPress = 0;

bool scanInputs(struct repeating_timer *t){
  updateInputFlag = true;
  return true;
}

bool updateScreen(struct repeating_timer *t){
  //updateScreenFlag = true;
  // TODO Time and intergrate
  return true;
}

struct repeating_timer scanTimer;
struct repeating_timer screenTimer;

// =============================================================================
// SETUP — CORE 0
// =============================================================================
void setup() {
    Serial.begin(115200);

    // Digital In
    pinMode(DMUX_OUT, INPUT);
    pinMode(OCT_DOWN_P, INPUT);
    pinMode(OCT_UP_P, INPUT);
    pinMode(AMUX_OUT, INPUT);

    // Digital Out
    for(int i = 0; i < 6; i++) {
    gpio_init(DMUXIN_PINS[i]);
    gpio_set_dir(DMUXIN_PINS[i], GPIO_OUT);
    DMUXIN_MASKS[i] = (1ul << DMUXIN_PINS[i]); // Pre-calculate the bitmask
    }

    for(int i = 0; i < 3; i++) {
    gpio_init(AMUXIN_PINS[i]);
    gpio_set_dir(AMUXIN_PINS[i], GPIO_OUT);
    AMUXIN_MASKS[i] = (1ul << AMUXIN_PINS[i]); // Pre-calculate the bitmask
    }

    add_repeating_timer_us(-scanPeriod,   scanInputs,   NULL, &scanTimer);
    add_repeating_timer_us(screenPeriod, updateScreen, NULL, &screenTimer);
}

// =============================================================================
// LOOP — CORE 0
// =============================================================================



float stableAnalogRead[4] = {-1,-1,-1,-1};
void loop() {
    if (updateInputFlag) {
        // Timed at 500uS - 4/16/26 at 1:28pm

        SharedInfo newInfo = {0, 0, 0, 0, 0, 0};

        // --- Key scanning -----------------------------------
        unsigned long int inputs = 0;
        for (int i = 0; i < numNotes; i++) {
            inputs |= getKey(i) << i;
        }

        lastRAWInputs[lawRAWInputsIndex] = inputs;
        lawRAWInputsIndex = (lawRAWInputsIndex + 1) % rawInputLen;

        inputs = 0;
        for (int i = 0; i < rawInputLen; i++) {
            inputs |= lastRAWInputs[i];
        }
        newInfo.keyInputs = inputs;

        long int changedANDdepressed = (oldInfo.keyInputs ^ inputs) & inputs;
        if (changedANDdepressed > 0) {
            for (int i = 0; i < numNotes; i++) {
                if ((changedANDdepressed >> i) & 1) {
                    lastKeyPress = i;
                }
            }
        }
        newInfo.lastKey = lastKeyPress;

        // --- Pot scanning (all 4 channels) ------------------------------
        bool flagAnalog = false;

        readAnalog();
        for (int ch = 0; ch < 4; ch++) {
            float v = rawAnalogRead[ch] - stableAnalogRead[ch];
            if(v < 0){
                v = -v;
            }

            if(v >= 0.05){
                if(stableAnalogRead[ch] > 0){ // Failsafe to prevent overriding defaults when initalizing
                    flagAnalog = true;
                }
                stableAnalogRead[ch] = rawAnalogRead[ch];
            }
        }

        // TODO make this log ish for between 0.001 and 1 s; GOodl what standard times are
        newInfo.A = stableAnalogRead[0]*0.1;
        newInfo.D = stableAnalogRead[0]*0.1;
        newInfo.R = stableAnalogRead[0]*0.1;

        newInfo.S = stableAnalogRead[0]; // Stays between 0 and 1

        oldInfo = newInfo;
        SHARED_inputs = newInfo;

        if(flagAnalog){
            SHARED_flags = 1;
        }

        updateInputFlag = false;
    }

    // TODO Each chunk must be less than 4mS pref 3mS
    if (updateScreenFlag) {
        SharedInfo info = SHARED_inputs;

        float a = info.A;
        float d = info.D;
        float s = info.S;
        float r = info.R;

        // Static elements drawn once on first call
        static bool screenInitialised = false;
        if (!screenInitialised) {
            disp.fill_screen(BLACK);

            disp.rect(  4, 32,  28, 282, CYAN, 4);
            disp.rect( 38, 32,  28, 282, CYAN, 4);
            disp.rect(414, 32,  28, 282, CYAN, 4);
            disp.rect(448, 32,  28, 282, CYAN, 4);
            disp.rect( 72, 32, 335, 282, CYAN, 4);

            disp.char_draw(  6, 4, 'A', CYAN, NO_BG, 3);
            disp.char_draw( 40, 4, 'D', CYAN, NO_BG, 3);
            disp.char_draw(416, 4, 'S', CYAN, NO_BG, 3);
            disp.char_draw(450, 4, 'R', CYAN, NO_BG, 3);
            disp.text(132, 4, "ENVELOPE", CYAN, NO_BG, 3);
            disp.draw_logo(324, 4, CYAN, NO_BG);

            screenInitialised = true;
        }

        // Only redraw if a value changed beyond threshold
        static float last_a = -1.0f, last_d = -1.0f,
                     last_s = -1.0f, last_r = -1.0f;
        static const float THRESHOLD = 0.02f;

        bool changed = fabsf(a - last_a) >= THRESHOLD ||
                       fabsf(d - last_d) >= THRESHOLD ||
                       fabsf(s - last_s) >= THRESHOLD ||
                       fabsf(r - last_r) >= THRESHOLD;

        if (changed) {
            // Redraw bars
            float bar_data[4] = { a, d, s, r };
            static const int X_STARTS[4] = { 10,  44, 420, 454 };
            static const int X_STOPS[4]  = { 29,  63, 439, 473 };

            for (int i = 0; i < 4; i++) {
                disp.graph_bar_from_array(
                    X_STARTS[i], X_STOPS[i], 308,
                    bar_data, i,
                    16, 270.0f, CYAN, (int32_t)BLACK);
            }

            // Erase old envelope with black lines, draw new envelope
            static int prev_x[5] = {-1, -1, -1, -1, -1};
            static int prev_y[5] = {-1, -1, -1, -1, -1};

            const int d_px   = (int)(27 * d);
            const int s_px   = (int)(27 * s);
            const int r_px   = (int)(27 * r);
            const int peak_y = (int)(119 - 70 * a);

            const int new_x[5] = {
                81,
                116,
                151 + d_px,
                269 + d_px + s_px,
                316 + d_px + s_px + r_px
            };
            const int new_y[5] = { 289, peak_y, 202, 202, 289 };

            if (prev_x[0] != -1) {
                for (int i = 0; i < 4; i++) {
                    disp.line(prev_x[i], prev_y[i],
                              prev_x[i+1], prev_y[i+1], BLACK);
                }
            }

            for (int i = 0; i < 4; i++) {
                disp.line(new_x[i], new_y[i],
                          new_x[i+1], new_y[i+1], CYAN);
            }

            for (int i = 0; i < 5; i++) {
                prev_x[i] = new_x[i];
                prev_y[i] = new_y[i];
            }

            last_a = a; last_d = d; last_s = s; last_r = r;
        }

        updateScreenFlag = false;
    }
}

// =============================================================================
// CORE 1 — Digital Synth
// =============================================================================
void setup1() {
  i2s.setBitsPerSample(16);
  i2s.begin(SAMPLE_RATE);
}

#define AS 0.01f
#define DS 0.01f
#define SS 0.3f
#define RS 0.01f
Note notes[numNotes] = {
    Note(82.4f,   AS, DS, SS, RS), // E2
    Note(87.3f,   AS, DS, SS, RS), // F2
    Note(98.0f,   AS, DS, SS, RS), // G2
    Note(110.0f,  AS, DS, SS, RS), // A2
    Note(123.47f, AS, DS, SS, RS), // B2
    Note(130.81f, AS, DS, SS, RS), // C3
    Note(146.83f, AS, DS, SS, RS), // D3
    Note(164.81f, AS, DS, SS, RS), // E3
    Note(174.61f, AS, DS, SS, RS), // F3
    Note(196.0f,  AS, DS, SS, RS), // G3
    Note(220.0f,  AS, DS, SS, RS), // A3
    Note(246.94f, AS, DS, SS, RS), // B3
    Note(261.63f, AS, DS, SS, RS), // C4 (Middle C)
    Note(293.67f, AS, DS, SS, RS), // D4
    Note(329.63f, AS, DS, SS, RS), // E4
    Note(349.23f, AS, DS, SS, RS), // F4
    Note(392.0f,  AS, DS, SS, RS), // G4
    Note(440.0f,  AS, DS, SS, RS), // A4
    Note(493.88f, AS, DS, SS, RS), // B4
    Note(523.25f, AS, DS, SS, RS), // C5
    Note(587.33f, AS, DS, SS, RS), // D5
    Note(659.26f, AS, DS, SS, RS), // E5
    Note(698.46f, AS, DS, SS, RS), // F5
    Note(783.99f, AS, DS, SS, RS), // G5
    Note(880.00f, AS, DS, SS, RS)  // A5
};

// Loop CORE 1
void loop1() {
    SharedInfo newInfo = SHARED_inputs;
    uint8_t readFlags = SHARED_flags;

    if(readFlags > 0){
        SHARED_flags = 0;
        //notes[newInfo.lastKey].updateControls(newInfo.A, newInfo.D, newInfo.S, newInfo.R);
    }

    // Clear buffer
    for (int i = 0; i < GENERATION_SAMPLES; i++){
        sampleValues[i] = 0;
        samples[i] = 0;
    }

    unsigned long shiftKeyInputs = newInfo.keyInputs;

    for(int i=0; i<numNotes; i++){
        notes[i].updateInput(((shiftKeyInputs >> i) & 1) > 0); // TODO Ensure actually works
        notes[i].getSample(); // TODO Test with keyboard so not all high by default
    }

    for (int i = 0; i < GENERATION_SAMPLES; i++) {
        float v = sampleValues[i];
        
        // Soft clipping (2uS)
        if (v > 1.0f) v = 0.66f;
        else if (v < -1.0f) v = -0.66f;
        else v = v - 0.33f * v * v * v;

        samples[i] = (int16_t)(v * MAX_AMPLITUDE);

        i2s.write(samples[i]); // TODO Takes 10 uS each per sample
        i2s.write(samples[i]); 
    }

}
