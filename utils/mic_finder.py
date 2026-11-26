#!/usr/bin/env python3
import sounddevice as sd
sd.default.device = (None, "UACDemoV1.0") 

sr=int(sd.query_devices(None, "input")["default_samplerate"])
print(f"Default sample reate {sr}")
print(sd.query_devices())


