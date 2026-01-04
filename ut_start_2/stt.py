import numpy as np
import soundfile as sf
import wave
import sys
import json

from vosk import Model, KaldiRecognizer, SetLogLevel


def format_timestamp(seconds: float) -> str:
    """Convert seconds to MM:SS.mmm format."""
    minutes = int(seconds // 60)
    secs = seconds % 60
    return f"{minutes:02d}:{secs:06.3f}"


# Load WAV file
audio_array, sample_rate = sf.read("output.wav")  # audio_array shape: (num_samples,) or (num_samples, channels)

num_samples = audio_array.shape[0]

# Time array in seconds
times = np.arange(num_samples) / sample_rate

print(f"Audio has {num_samples} samples at {sample_rate} Hz")
print(f"First 10 time points: {times[:10]}")
print(f"Last time point: {times[-1]} s")

SetLogLevel(0)

wf = wave.open("output.wav", "rb")
if wf.getnchannels() != 1 or wf.getsampwidth() != 2 or wf.getcomptype() != "NONE":
    print("Audio file must be WAV format mono PCM.")
    sys.exit(1)

model = Model("/home/gianni/Downloads/vosk-model-it-0.22")
rec = KaldiRecognizer(model, wf.getframerate())
rec.SetWords(True)
# rec.SetMaxAlternatives(10)
# rec.SetWords(True)

# text_file = open("results.txt", "w")

i = 0
results = []
while True:
    i += 1
    
    data = wf.readframes(4000)      # 0.25 s
    if len(data) == 0:
        break
    if rec.AcceptWaveform(data):
        output = json.loads(rec.Result())
        results.append(output)
        print("Questo e l'output: ")
        print("\t", output)
        print("Questa e la lista: ")
        print("\t", results, "\n")

# ----------------------------
# WRITE SIMPLE TIMESTAMPED FILE
# ----------------------------
with open("results.txt", "w") as f:
    for res in results:
        if "result" not in res:
            continue

        words = res["result"]
        if not words:
            continue

        # Group words into phrases using the first and last word timestamps
        start_time = words[0]["start"]
        end_time = words[-1]["end"]
        text = " ".join([w["word"] for w in words])
        print(text)

        f.write(f"{format_timestamp(start_time)} --> {format_timestamp(end_time)}\t\t{text}\n")

print(f"Saved transcript to results.txt")