import wave
import pyaudio

FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
RECORD_SECONDS = 10
CHUNK = 16000
WAV_OUTPUT_FILENAME = "audioFile2.wav"

EXTERNAL_MIC_NAME = 'USB audio CODEC: Audio (hw:1,0)'

mic_index = None
audio = pyaudio.PyAudio()
info = audio.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')

for i in range(0, numdevices):
    print(audio.get_device_info_by_host_api_device_index(0, i).get('name'))
    if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
        if audio.get_device_info_by_host_api_device_index(0, i).get('name') == EXTERNAL_MIC_NAME:
            mic_index = i
            break

print(mic_index)

print('USB Audio Device found, recording!')
stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True,
                    frames_per_buffer=CHUNK, input_device_index=mic_index)

frames = []

for i in range(int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

# Stops the recording
stream.stop_stream()
stream.close()
audio.terminate()

wav_file = wave.open(WAV_OUTPUT_FILENAME, 'wb')
wav_file.setnchannels(CHANNELS)
wav_file.setsampwidth(2)
wav_file.setframerate(RATE)
wav_file.writeframes(b''.join(frames))
wav_file.close()