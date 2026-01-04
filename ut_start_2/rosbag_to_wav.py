import os
import rosbag2_py
import numpy as np
import soundfile as sf
from rclpy.serialization import deserialize_message
from audio_common_msgs.msg import AudioStamped


bag_path = os.path.expanduser("~") + "/rosbag2_2025_12_30_16_06_38"

# Topic name
topic_name = "/audio"

# Open ROS 2 bag
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

all_audio = []
sample_rate = None
channels = None

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic != topic_name:
        continue

    # Deserialize the raw data
    msg = deserialize_message(data, AudioStamped)

    # Pick int16_data if available
    if msg.audio.audio_data.int16_data:
        audio_chunk = np.array(msg.audio.audio_data.int16_data, dtype=np.int16)
    else:
        continue

    all_audio.append(audio_chunk)

    if sample_rate is None:
        sample_rate = msg.audio.info.rate
        channels = msg.audio.info.channels

# Concatenate all audio chunks
audio_array = np.concatenate(all_audio)

# Handle multi-channel audio
if channels > 1:
    audio_array = audio_array.reshape(-1, channels)

# Write WAV
sf.write("output.wav", audio_array, samplerate=sample_rate)
print(f"Saved audio to output.wav (rate={sample_rate}, channels={channels})")