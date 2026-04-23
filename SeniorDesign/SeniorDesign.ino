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

#define DEFAULT_OCTAVE 4
#define MIN_OCTAVE 2
#define MAX_OCTAVE 5

static Display disp(spi1, LCD_SCK, LCD_MOSI, LCD_CS, LCD_RS, LCD_RST);

constexpr int SAMPLE_RATE = 8000;
constexpr int MAX_AMPLITUDE = 32767;
constexpr float SAMPLE_PERIOD = 1.0 / SAMPLE_RATE;
constexpr int GENERATION_SAMPLES = 128;
#define MIN_AMP_ANALOG 0.0000001f

constexpr float TWOPI =  2*PI;

float sampleValues[GENERATION_SAMPLES];
int16_t samples[GENERATION_SAMPLES];

I2S i2s(OUTPUT, SCK_PIN, SD_PIN);

int octave = DEFAULT_OCTAVE;

class Note {
public:
    float baseFreq;
    float freq, A, D, S, R;
    float ASlope, DSlope, RSlope;
    float phase, phaseStep;
    int mode; // 0=Silent, 1=A, 2=D, 3=S, 4=R
    float amp;
    bool inputPressed;

    Note(float f, float a, float d, float s, float r) {
        baseFreq = f;
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

    void setOctave(int newOct){
        newOct = newOct < MIN_OCTAVE ? MIN_OCTAVE : newOct;
        newOct = newOct > MAX_OCTAVE ? MAX_OCTAVE : newOct;
        int p = newOct - DEFAULT_OCTAVE;
        updateFrequency(baseFreq * pow(2, p));
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
            sampleValues[i] += amp * (B * phase + C * phase * (phase < 0 ? -phase : phase));

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

        delayMicroseconds(500);    // allow AMUX output to settle
        analogRead(AMUX_OUT);     // dummy read to flush ADC sample-and-hold
        delayMicroseconds(10);

        rawAnalogRead[ch] = analogRead(AMUX_OUT) / 1023.0;
    }
}

int oct = DEFAULT_OCTAVE;

typedef struct {
    unsigned long int keyInputs;
    int lastKey;
    float A;
    float D;
    float S;
    float R;
    int octave = DEFAULT_OCTAVE;
} SharedInfo;

std::atomic<SharedInfo> SHARED_inputs;
std::atomic<uint8_t> SHARED_flags;

SharedInfo oldInfo = {0, 0, 0, 0, 0, 0, DEFAULT_OCTAVE};

volatile bool updateInputFlag = false;
volatile bool updateScreenFlag = false;

// Humans feel lag in >15mS
const int scanPeriod = 5000; //uS or slower (Must be slower than the funtion, but faster than at least 5mS)
const int screenPeriod = 15000; //uS or slower (Must be slower than the funtion, but faster than at least 5mS)
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
  updateScreenFlag = true;
  // TODO Time and intergrate
  return true;
}

struct repeating_timer scanTimer;
struct repeating_timer screenTimer;
float stableAnalogRead[4] = {-1,-1,-1,-1};

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

    readAnalog();
    for (int ch = 0; ch < 4; ch++) {
        stableAnalogRead[ch] = rawAnalogRead[ch];
    }
}

// =============================================================================
// LOOP — CORE 0
// =============================================================================


bool wasOctBTNPress = false;
bool didTryChangeAnalog = false;
bool wasOctUpPressed = false; // Debounce
bool wasOctDownPressed = false; // Debounce
int upCount = 0;
int downCount = 0;
bool ADSR_MODE = false;
int drawStage = 0;

void loop() {
    SharedInfo newInfo = oldInfo;
    if (updateInputFlag) {
        bool isOctUpPressed = (digitalRead(OCT_UP_P) == HIGH);
        bool isOctDownPressed = (digitalRead(OCT_DOWN_P) == HIGH);

        if(isOctUpPressed && wasOctUpPressed){
            upCount++;
        }

        if(isOctDownPressed && wasOctDownPressed){
            downCount++;
        }

        bool wasUpShortRelease = false;
        bool wasUpLongRelease = false;
        bool wasDownShortRelease = false;
        bool wasDownLongRelease = false;
        constexpr int decisionBoundry = 20; // Each count is 5ms

        if((!isOctUpPressed) && (!wasOctUpPressed) && (upCount > 0)){
            // Rlease occured
            if(upCount < decisionBoundry){
                wasUpShortRelease = true;
                Serial.println("A");
            }else{
                wasUpLongRelease = true;
                Serial.println("B");
            }

            upCount = 0;
        }

        if((!isOctDownPressed) && (!wasOctDownPressed) && (downCount > 0)){
            // Rlease occured
            if(downCount < decisionBoundry){
                wasDownShortRelease = true;
                Serial.println("C");
            }else{
                wasDownLongRelease = true;
                Serial.println("D");
            }

            downCount = 0;
        }

        if(wasDownShortRelease){
            ADSR_MODE = !ADSR_MODE;
            if(ADSR_MODE){drawStage=0;}
        }
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

        //Serial.println(inputs, BIN);

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
        bool tryChangeAnalog = false;

        if(ADSR_MODE){
            readAnalog();
            for (int ch = 0; ch < 4; ch++) {
                float avgV = rawAnalogRead[ch];
                float v = avgV - stableAnalogRead[ch];
                if(v < 0){
                    v = -v;
                }

                if(v >= 0.05){
                    tryChangeAnalog = true;
                }
            }

            // only update ADSR for last key if octave down is short pressed
            if(tryChangeAnalog){
                Serial.println("TRIED");
                if(didTryChangeAnalog){
                    flagAnalog = true;
                    for (int ch = 0; ch < 4; ch++) {
                        stableAnalogRead[ch] = rawAnalogRead[ch];
                    }
                    didTryChangeAnalog = false;
                }else{
                    didTryChangeAnalog = true;
                }
            }else{
                didTryChangeAnalog = false;
            }
            // TODO make this log ish for between 0.001 and 1 s; GOodl what standard times are
            newInfo.A = stableAnalogRead[0];
            newInfo.D = stableAnalogRead[1];
            newInfo.R = stableAnalogRead[3];
            newInfo.S = stableAnalogRead[2]; // Stays between 0 and 1
        }


        if(wasUpLongRelease){
            octave++;
        }
        if(wasDownLongRelease){
            octave--;
        }
        octave = octave < MIN_OCTAVE ? MIN_OCTAVE : octave;
        octave = octave > MAX_OCTAVE ? MAX_OCTAVE : octave;
        newInfo.octave = octave;

        SHARED_inputs = newInfo;

        if(flagAnalog){
            SHARED_flags |= 1;
        }
        
        if(newInfo.octave != oldInfo.octave){
            SHARED_flags |= 2;
        }

        oldInfo = newInfo;
        updateInputFlag = false;
        wasOctUpPressed = isOctUpPressed;
        wasOctDownPressed = isOctDownPressed;
    }


    // Static elements drawn once on first call
    static bool screenInitialised = false;
    if (!screenInitialised) {
        Serial.println("JFIsjdsijfijsoifsjfidjsidfji");
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

    // TODO Each chunk must be less than 4mS pref 3mS
    if (updateScreenFlag && ADSR_MODE) {
        float a = oldInfo.A;
        float d = oldInfo.D;
        float s = oldInfo.S;
        float r = oldInfo.R;


        float bar_data[4] = { a, d, s, r };
        static const int X_STARTS[4] = { 10,  44, 420, 454 };
        static const int X_STOPS[4]  = { 29,  63, 439, 473 };
        
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

        switch(drawStage){
            case 0:
                disp.graph_bar_from_array(X_STARTS[0], X_STOPS[0], 308, bar_data, 0, 16, 270.0f, CYAN, (int32_t)BLACK);
                break;
            case 1:
                disp.graph_bar_from_array(X_STARTS[1], X_STOPS[1], 308, bar_data, 1, 16, 270.0f, CYAN, (int32_t)BLACK);
                break;
            case 2:
                disp.graph_bar_from_array(X_STARTS[2], X_STOPS[2], 308, bar_data, 2, 16, 270.0f, CYAN, (int32_t)BLACK);
                break;
            case 3:
                disp.graph_bar_from_array(X_STARTS[3], X_STOPS[3], 308, bar_data, 3, 16, 270.0f, CYAN, (int32_t)BLACK);
                break;
            case 4:
                // Erase old envelope with black lines, draw new envelope
                if (prev_x[0] != -1) {
                    for (int i = 0; i < 4; i++) {
                        disp.line(prev_x[i], prev_y[i],
                                    prev_x[i+1], prev_y[i+1], BLACK);
                    }
                }
                break;
            case 5:
                for (int i = 0; i < 4; i++) {
                    disp.line(new_x[i], new_y[i],
                            new_x[i+1], new_y[i+1], CYAN);
                }
                break;
            case 6:
                for (int i = 0; i < 5; i++) {
                    prev_x[i] = new_x[i];
                    prev_y[i] = new_y[i];
                }
                break;
        }
        drawStage++;
        if(drawStage > 6){drawStage=0;}

        updateScreenFlag = false;
    }
}

// =============================================================================
// CORE 1 — Digital Synth
// =============================================================================
void setup1() {
  i2s.setBitsPerSample(16);
  i2s.setStereo(false);
  i2s.begin((long)SAMPLE_RATE);
}

#define AS 0.01f
#define DS 0.01f
#define SS 0.25f
#define RS 0.01f
Note notes[numNotes] = {
    Note(130.81f, AS, DS, SS, RS), // C3
    Note(138.59f, AS, DS, SS, RS), // C#3
    Note(146.83f, AS, DS, SS, RS), // D3
    Note(155.56f, AS, DS, SS, RS), // D#3
    Note(164.81f, AS, DS, SS, RS), // E3
    Note(174.61f, AS, DS, SS, RS), // F3
    Note(185.00f, AS, DS, SS, RS), // F#3
    Note(196.00f, AS, DS, SS, RS), // G3
    Note(207.65f, AS, DS, SS, RS), // G#3
    Note(220.00f, AS, DS, SS, RS), // A3
    Note(233.08f, AS, DS, SS, RS), // A#3
    Note(246.94f, AS, DS, SS, RS), // B3
    Note(261.63f, AS, DS, SS, RS), // C4 (Middle C)
    Note(277.18f, AS, DS, SS, RS), // C#4
    Note(293.67f, AS, DS, SS, RS), // D4
    Note(311.13f, AS, DS, SS, RS), // D#4
    Note(329.63f, AS, DS, SS, RS), // E4
    Note(349.23f, AS, DS, SS, RS), // F4
    Note(369.99f, AS, DS, SS, RS), // F#4
    Note(392.00f, AS, DS, SS, RS), // G4
    Note(415.30f, AS, DS, SS, RS), // G#4
    Note(440.00f, AS, DS, SS, RS), // A4
    Note(466.16f, AS, DS, SS, RS), // A#4
    Note(493.88f, AS, DS, SS, RS), // B4
    Note(523.25f, AS, DS, SS, RS)  // C5
};

// Loop CORE 1
void loop1() {
    long int t0 = (long)micros();

    SharedInfo newInfo = SHARED_inputs;
    uint8_t readFlags = SHARED_flags;

    if((readFlags & 1) > 0){
        SHARED_flags &= ~(1);
        /*
        Serial.print(newInfo.lastKey);
        Serial.print(", ");
        Serial.print(newInfo.A);
        Serial.print(", ");
        Serial.print(newInfo.D);
        Serial.print(", ");
        Serial.print(newInfo.S);
        Serial.print(", ");
        Serial.print(newInfo.R);
        Serial.println();*/
        notes[newInfo.lastKey].updateControls(newInfo.A, newInfo.D, newInfo.S, newInfo.R);
    }

    if((readFlags & 2) > 0){
        SHARED_flags &= ~(2);
        for(int i=0; i<numNotes; i++){
            notes[i].setOctave(newInfo.octave);
        }
    }


    // Clear buffer
    for (int i = 0; i < GENERATION_SAMPLES; i++){
        sampleValues[i] = 0;
    }

    unsigned long shiftKeyInputs = newInfo.keyInputs;

    int activeNotes = 0;
    for(int i=0; i<numNotes; i++){
        notes[i].updateInput(((shiftKeyInputs >> i) & 1) > 0);
        notes[i].getSample();
        if(notes[i].mode > 0) activeNotes++; // Count how many oscillators are running
    }

    float scalar = 1.0f / (1.0f + (activeNotes * 0.2f));

    for (int i = 0; i < GENERATION_SAMPLES; i++) {
        float v = sampleValues[i]*scalar;

        // v = v > 1 ? 1 : v;
        // v = v < -1 ? -1 : v;

        if (v > 1.0f) v = 0.666f;
        else if (v < -1.0f) v = -0.666f;
        else v = v - (0.333f * v * v * v);

        i2s.write((int16_t)(v * MAX_AMPLITUDE * 0.95));
        //Serial.println((int16_t)(v * MAX_AMPLITUDE * 0.95));

    }

    long int t1 = (long)micros();
    //Serial.println(max);
    //Serial.print(((float)(t1-t0)) / GENERATION_SAMPLES);
    //Serial.print(" ");
    //Serial.println(GENERATION_SAMPLES*22 - (t1-t0));
}
