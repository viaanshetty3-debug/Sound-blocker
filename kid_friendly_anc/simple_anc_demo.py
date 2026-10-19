#!/usr/bin/env python3
"""
KID-FRIENDLY NOISE CANCELLER IN PYTHON (Easy to Read & Understand!)
===================================================================
Shows how just 4 simple lines of math inside a `for` loop can cancel
out a loud traffic bass wave by waiting for the wave and multiplying by -1!
"""

import math

# 1. Settings
SAMPLE_RATE = 10000          # 10,000 measurements per second
SOUND_DELAY_STEPS = 12       # Sound takes 1.2 milliseconds (12 steps) to reach speaker
TRUE_ROOM_GAIN = 0.85        # Sound loses a little energy traveling through the air

# Memory list to store the last 200 microphone readings
wave_memory = [0.0] * 200
write_pos = 0

# What our code is learning automatically:
delay_steps = 12             # Wait 12 steps so our speaker fires at the exact right moment
volume = 0.20                # Start with low speaker volume; watch the code learn the right volume!

print("================================================================")
print(" KID-FRIENDLY NOISE CANCELLER — WATCH THE CODE LEARN LIVE!")
print("================================================================")

for step in range(10000):    # Run for 1 second (10,000 steps)
    t = step / SAMPLE_RATE

    # --- STEP 1: READ THE OUTDOOR MICROPHONE ---
    # Imagine a loud 100 Hz truck rumble hitting the outdoor mic:
    incoming_wave = math.sin(2 * math.pi * 100 * t)
    wave_memory[write_pos] = incoming_wave

    # What the room hears naturally (arrives 12 steps later):
    real_pos = (write_pos - SOUND_DELAY_STEPS) % 200
    noise_reaching_room = wave_memory[real_pos] * TRUE_ROOM_GAIN

    # --- STEP 2: GRAB THE WAVE FROM 'delay_steps' AGO ---
    read_pos = (write_pos - delay_steps) % 200
    delayed_wave = wave_memory[read_pos]

    # --- STEP 3: FLIP IT UPSIDE DOWN (Multiply by -1!) ---
    anti_noise = -1.0 * delayed_wave * volume

    # --- STEP 4: CHECK WHAT IS LEFT OVER IN THE ROOM ---
    # When +Noise and -AntiNoise crash into each other, they add together:
    indoor_error = noise_reaching_room + anti_noise

    # --- STEP 5: AUTO-TUNE VOLUME (Learn to make indoor_error = 0) ---
    if indoor_error * delayed_wave > 0:
        volume += 0.0005     # Still hearing original wave -> turn speaker UP a tiny bit
    else:
        volume -= 0.0005     # Speaker is too loud -> turn speaker DOWN a tiny bit

    write_pos = (write_pos + 1) % 200

    # Print progress every 0.2 seconds
    if (step + 1) % 2000 == 0:
        cancelled_pct = (1.0 - abs(indoor_error) / (abs(noise_reaching_room) + 1e-9)) * 100
        print(
            f"Time {(step+1)/SAMPLE_RATE:.1f}s | "
            f"Learned Volume: {volume:.2f} (Target: {TRUE_ROOM_GAIN}) | "
            f"Noise Cancelled: {max(0.0, cancelled_pct):.1f}%"
        )
