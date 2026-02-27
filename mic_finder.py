#!/usr/bin/env python3
import sounddevice as sd

sr=int(sd.query_devices(None, "input")["default_samplerate"])
print(f"Default sample reate {sr}")
print(sd.query_devices())

