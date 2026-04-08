#include <I2S.h>
#include <pio_i2s.pio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <cstdint>
#include <atomic>
#include "hardware/timer.h"

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

// TODO VERIFY LCD, OCT, AMUX

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

            sampleValues[i] += amp * sinf(phase); // TODO Sine lookup table or parabolic approx if pure sine function is too slow when several notes play

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
    sleep_us(2);

    // Fast Read: Check if the DMUX_OUT pin bit is set in the input register
    return (sio_hw->gpio_in & (1ul << DMUX_OUT)) ? 1 : 0;
}

//0-3
float readAnalog(int key){
  // Set AMUX
  for (int b = 0; b < 3; b++) {
    if ((key >> b) & 1) {
        sio_hw->gpio_set = AMUXIN_MASKS[b]; // Atomic SET
    } else {
        sio_hw->gpio_clr = AMUXIN_MASKS[b]; // Atomic CLEAR
    }
  }

  sleep_us(2);
  
  // Read Analog
  return (float)analogRead(AMUX_OUT)/(1024.0f);
}


typedef struct{
  unsigned long int keyInputs;
  int lastKey;
  float A;
  float D;
  float S;
  float R;
  int flags;
} SharedInfo;

std::atomic<SharedInfo> SHARED_inputs;
SharedInfo oldInfo = {0, 0, 0, 0, 0, 0, 0};

volatile bool updateInputFlag = false;
volatile bool updateScreenFlag = false;

// Humans feel lag in >15mS
const int scanPeriod = 5000; //uS or slower (Must be slower than the funtion, but faster than at least 5mS)
const int screenPeriod = 10000; //uS or slower (Must be slower than the funtion, but faster than at least 5mS)

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
  return true;
}

struct repeating_timer scanTimer;
struct repeating_timer screenTimer;

// Setup CORE 0
void setup() {
  Serial.begin(115200);

  // Digital In
  pinMode(DMUX_OUT, INPUT);
  pinMode(OCT_DOWN_P, INPUT);
  pinMode(OCT_UP_P, INPUT);

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

  // Analog IN
  pinMode(AMUX_OUT, INPUT);
  pinMode(AUDIO_IN, INPUT);

  add_repeating_timer_us(-scanPeriod, scanInputs, NULL, &scanTimer);
  add_repeating_timer_us(-screenPeriod, updateScreen, NULL, &screenTimer);
}

// Loop CORE 0
void loop() {
  if(updateInputFlag){
    SharedInfo newInfo = {0, 0, 0, 0, 0, 0, 0};
    
    // Key Press Scanning
    unsigned long int inputs = 0;
    
    for(int i=0; i<25; i++){
      inputs |= getKey(i)<<i;
    }

    inputs = ~inputs; // TODO TEMP while pull down resistor problem gets fixed

    // Debounce logic (Pins are pulled low by default thus any high would start immediate attack; Any high pins will stay high creating debounce logic prevening premature release)
    lastRAWInputs[lawRAWInputsIndex] = inputs;
    lawRAWInputsIndex = (lawRAWInputsIndex + 1) % rawInputLen; // Circular buffer for inputs
    
    inputs = 0;
    for(int i=0; i<rawInputLen; i++){
      inputs |= lastRAWInputs[i]; // Keep any high bits high (immedate attack); only once all previous samples of the bit is clear then the key is released (debounce)
    }
    newInfo.keyInputs = inputs;
    

    long int changedANDdepressed = (oldInfo.keyInputs ^ inputs) & inputs;
    if(changedANDdepressed > 0){
      for(int i=0; i<32; i++){
        if((changedANDdepressed>>i)&1){
          lastKeyPress = i;
        }
      }
    }
    newInfo.lastKey = lastKeyPress;


    Serial.print(inputs, BIN);
    Serial.print(", ");
    Serial.print(newInfo.lastKey + 1);
    Serial.println();
    
    // Pent Scanning
    float A = 0; // Bounds?
    // TODO, need filter (EMA?), minimum steps?
    // TODO Add Update Flag
    
    // TODO Octave select
    
    oldInfo = newInfo;
    SHARED_inputs = newInfo;
    updateInputFlag = false;
  }
  
  if(updateScreenFlag){
    // TODO
    updateScreenFlag = false;
  }
}

/// CORE 1 -------------------------------------------------
// Digital Syth Core

// Setup CORE 1
void setup1() {
  i2s.setBitsPerSample(16);
  i2s.begin(SAMPLE_RATE);
}

Note myNote(440.0f, 3.0f, 1.0f, 0.4f, 3.0f); // TODO Implement all notes
Note myNote2(20000.0f, 1.5f, 0.8f, 0.5f, 1.0f);

// Loop CORE 1
void loop1() {
  SharedInfo newInfo = SHARED_inputs;

  // TODO Update Inputs, Configs here
  myNote.updateInput((newInfo.keyInputs >> 0)& 1 > 0); // Key 1
  myNote2.updateInput((newInfo.keyInputs >> 9)& 1 > 0); // Key 10
  
  // Clear buffer
  for (int i = 0; i < GENERATION_SAMPLES; i++){
      sampleValues[i] = 0;
      samples[i] = 0;
  }
  
  myNote.getSample();
  myNote2.getSample();
  
  for (int i = 0; i < GENERATION_SAMPLES; i++) {
      float v = sampleValues[i];
      
      // Soft clipping
      if (v > 1.0f) v = 0.66f;
      else if (v < -1.0f) v = -0.66f;
      else v = v - 0.33f * v * v * v;
  
      samples[i] = (int16_t)(v * MAX_AMPLITUDE);
      i2s.write(samples[i]);
      i2s.write(samples[i]);
  }
}
