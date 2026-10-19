/*
 * ============================================================================
 * KID-FRIENDLY ACTIVE NOISE CANCELLER (50 Lines of Easy Code!)
 * Works on a $5 ESP32 (No extra DAC chips needed!)
 * ============================================================================
 *
 * HOW IT WORKS IN 3 EASY STEPS:
 * 1. READ:  The Outdoor Mic hears the noise wave coming in.
 * 2. WAIT:  We save the wave in a list (`waveMemory`) and wait a tiny split
 *           second until the real sound wave reaches the speaker.
 * 3. FLIP:  We multiply the wave by -1 (flipping peaks into valleys!) and
 *           play it out of the speaker to cancel the noise!
 *
 * WIRING (Only 3 Parts!):
 * - Outdoor Mic (MAX4466/MAX9814) OUT  ---> ESP32 Pin 34
 * - Indoor Error Mic (Optional)   OUT  ---> ESP32 Pin 35
 * - ESP32 Audio Output            Pin 25 -> Mini Amp (PAM8403) -> Speaker
 */

#include <Arduino.h>

const int OUTDOOR_MIC_PIN = 34;   // Hears incoming street noise
const int INDOOR_MIC_PIN  = 35;   // Checks if the room is quiet yet
const int SPEAKER_DAC_PIN = 25;   // Built-in audio output pin on ESP32

// Memory list to hold the last 200 sound measurements
int waveMemory[200];
int writePos = 0;

// Settings you can control from your computer keyboard!
int   delaySteps = 12;            // How long to wait for the wave (in steps)
float volume     = 0.80;          // How strong the anti-noise wave is
bool  ancEnabled = true;          // Turn Noise Cancelling ON or OFF
bool  autoTune   = true;          // Let the code tune itself automatically!

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);       // Read sound from 0 to 4095 (middle is 2048)

  Serial.println("=== NOISE CANCELLER READY! ===");
  Serial.println("Type SPACE to turn ON/OFF | 'a'/'d' to change Delay | 'w'/'s' to change Volume");
}

void loop() {
  // 1. READ THE MICROPHONE (Subtract 2048 so silence = 0)
  int incomingWave = analogRead(OUTDOOR_MIC_PIN) - 2048;
  int indoorError  = analogRead(INDOOR_MIC_PIN)  - 2048;

  // 2. SAVE THE WAVE IN MEMORY
  waveMemory[writePos] = incomingWave;

  // 3. GRAB THE WAVE FROM 'delaySteps' AGO (Wait for sound to reach speaker)
  int readPos = (writePos - delaySteps + 200) % 200;
  int delayedWave = waveMemory[readPos];

  // 4. FLIP THE WAVE UPSIDE DOWN (Multiply by -1!)
  int antiNoise = 0;
  if (ancEnabled) {
    antiNoise = (int)(-1.0 * delayedWave * volume);
  }

  // 5. SEND TO SPEAKER (Convert back to 0..255 for ESP32 DAC pin)
  int dacValue = 128 + (antiNoise / 16);
  dacValue = constrain(dacValue, 0, 255);
  dacWrite(SPEAKER_DAC_PIN, dacValue);

  // 6. SIMPLE AUTO-TUNER (Nudges 'volume' to make indoorError as small as possible)
  if (ancEnabled && autoTune) {
    // If indoor mic and delayed wave have the same sign, increase anti-noise slightly
    if (indoorError * delayedWave > 0) {
      volume += 0.00001;
    } else {
      volume -= 0.00001;
    }
    volume = constrain(volume, 0.1, 2.0);
  }

  // Move to the next spot in memory and wait 100 microseconds (10,000 Hz)
  writePos = (writePos + 1) % 200;
  delayMicroseconds(100);

  // Check if you pressed a key on the keyboard to tweak settings
  if (Serial.available()) {
    char key = Serial.read();
    if (key == ' ') ancEnabled = !ancEnabled;
    if (key == 'd') delaySteps = constrain(delaySteps + 1, 1, 190);
    if (key == 'a') delaySteps = constrain(delaySteps - 1, 1, 190);
    if (key == 'w') volume     = constrain(volume + 0.05, 0.0, 2.0);
    if (key == 's') volume     = constrain(volume - 0.05, 0.0, 2.0);

    Serial.print("ANC: ");    Serial.print(ancEnabled ? "ON" : "OFF");
    Serial.print(" | Delay: "); Serial.print(delaySteps * 0.1); Serial.print(" ms");
    Serial.print(" | Volume: "); Serial.println(volume);
  }
}
