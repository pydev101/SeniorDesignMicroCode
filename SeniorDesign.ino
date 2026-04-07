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
#define AMUX_S0 19
#define AMUX_S1 20
#define AMUX_S2 21

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

// TODO VERIFY DMUX, LCD, OCT, AMUX, I2S
// TODO Add volitile and mutexs

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
        A = (a < MIN_AMP_ANALOG) ? MIN_AMP_ANALOG : a;
        D = (d < MIN_AMP_ANALOG) ? MIN_AMP_ANALOG : d;
        S = (s < 0.0f) ? 0.0f : (s > 1.0f ? 1.0f : s);
        R = (r < MIN_AMP_ANALOG) ? MIN_AMP_ANALOG : r;

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

            sampleValues[i] += amp * sinf(phase); // TODO Sine lookup table or taylor series if pure sine function is too slow

            phase += phaseStep;
            if (phase >= PI) phase -= TWOPI; // Bound between -PI and PI
        }
    }
};

Note myNote(440.0f, 3.0f, 1.0f, 0.4f, 3.0f);
Note myNote2(500.0f, 1.5f, 0.8f, 0.5f, 1.0f);

// Setup CORE 0
void setup() {
  Serial.begin(115200);
  
  i2s.setBitsPerSample(16); // TODO Move to core 1
  i2s.begin(SAMPLE_RATE);

  // Digital In
  pinMode(DMUX_OUT, INPUT);
  pinMode(DMUX_OUT, INPUT_PULLDOWN); // TODO Pullup or pull down?
  pinMode(OCT_DOWN_P, INPUT);
  pinMode(OCT_DOWN_P, INPUT_PULLDOWN);
  pinMode(OCT_UP_P, INPUT);
  pinMode(OCT_UP_P, INPUT_PULLDOWN);
  
  pinMode(DMUX_S0, OUTPUT);
  pinMode(DMUX_S1, OUTPUT);
  pinMode(DMUX_S2, OUTPUT);
  pinMode(DMUX_S3, OUTPUT);
  pinMode(DMUX_S4, OUTPUT);
  pinMode(DMUX_S5, OUTPUT);
  
  pinMode(AMUX_S0, OUTPUT);
  pinMode(AMUX_S1, OUTPUT);
  pinMode(AMUX_S2, OUTPUT);

  pinMode(AMUX_OUT, INPUT);
  pinMode(AUDIO_IN, INPUT);
}

std::atomic<unsigned long int> x; //32 bits
void scanInputs(){
  // Key Press Scanning

  // Pent Scanning
  
}

// Loop CORE 0
void loop() {
  // Input Scanning
  
  
    // Process Note
    while(Serial.available() > 0){
        char x = Serial.read();
        Serial.println(x);
        if(x=='a'){
            myNote.updateInput(true);
        }else if(x=='b'){
            myNote.updateInput(false);
        }

        if(x=='c'){
            myNote2.updateInput(true);
        }else if(x=='d'){
            myNote2.updateInput(false);
        }
    }

  
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

/// CORE 1 -------------------------------------------------
// Digital Syth Core

// Setup CORE 1
void setup1() {
  // Initialize anything specific to Core 1 here
}

// Loop CORE 1
void loop1() {
  // Simulate a complex task
  delay(1000); 
}
