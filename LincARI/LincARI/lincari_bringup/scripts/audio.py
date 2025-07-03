#!/usr/bin/env python3

import rospy
import pyaudio
from audio_common_msgs.msg import AudioData

SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16  # 16-bit signed little-endian

class AudioPlayer:
    def __init__(self, device_name_hint='USB Audio'):
        self.p = pyaudio.PyAudio()
        self.device_index = self.find_output_device(device_name_hint)
        if self.device_index is None:
            rospy.logerr(f"No output device found containing: '{device_name_hint}'")
            raise RuntimeError("Audio output device not found")

        self.stream = self.p.open(format=FORMAT,
                                  channels=CHANNELS,
                                  rate=SAMPLE_RATE,
                                  output=True,
                                  output_device_index=self.device_index)

        rospy.loginfo(f"Streaming audio to device index {self.device_index}")

    def find_output_device(self, name_hint):
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if info['maxOutputChannels'] > 0 and name_hint in info['name']:
                return i
        return None

    def callback(self, msg):
        try:
            self.stream.write(bytes(msg.data))
        except Exception as e:
            rospy.logerr(f"Audio playback error: {e}")

    def close(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        rospy.loginfo("Audio stream closed.")

if __name__ == '__main__':
    rospy.init_node('usb_audio_player_node')

    player = AudioPlayer(device_name_hint='USB Audio')  # Match your USB device name here

    rospy.Subscriber('/humans/voices/anonymous_speaker/audio', AudioData, player.callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down audio playback node.")
    finally:
        player.close()
