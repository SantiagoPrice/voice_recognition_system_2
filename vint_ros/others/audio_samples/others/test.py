import sounddevice as sd
import soundfile as sf
sd.default.device = (None, "UACDemoV1.0") 

audio_file="button-pressed-sound.mp3"
data, fs = sf.read(audio_file)
sd.play(data, fs)
sd.wait()