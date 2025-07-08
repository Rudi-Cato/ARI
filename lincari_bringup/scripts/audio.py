# subscirbes to aris microphone and plays back in realtime on selected audio output
#!/usr/bin/env python3
import rospy
from audio_common_msgs.msg import AudioData  
import pyaudio

class AudioPlayerNode:
    def __init__(self, topic_name, rate=16000, device_name_substr='Jabra'): #jabra headset is used for testing
        rospy.init_node('audio_player_node', anonymous=True)

        self.p = pyaudio.PyAudio()

        # Find audio output device index matching a substring in device name (jabra headset for testing))
        
        self.device_index = None
        for i in range(self.p.get_device_count()):
            dev_info = self.p.get_device_info_by_index(i)
            dev_name = dev_info['name']
            rospy.loginfo(f"Audio device {i}: {dev_name}")
            if device_name_substr in dev_name:
                self.device_index = i
                rospy.loginfo(f"Selected audio output device #{i}: {dev_name}")
                break

        # If device not found, raise error and exit
        if self.device_index is None:
            rospy.logerr(f"Could not find audio output device matching '{device_name_substr}'")
            exit(1)

        
        # format = 16-bit PCM, mono channel, sample rate = 16000 Hz
        # output_device_index = index of your selected headset
        self.stream = self.p.open(format=pyaudio.paInt16,
                                  channels=1,
                                  rate=rate,
                                  output=True,
                                  output_device_index=self.device_index,
                                  frames_per_buffer=1024)

        # Subscribe to the ROS audio topic/humans/voices/anonymous_speaker/audio
        self.sub = rospy.Subscriber(topic_name, AudioData, self.audio_callback, queue_size=10)

        rospy.loginfo(f"Subscribed to {topic_name} and ready to play audio.")

    def audio_callback(self, msg):
        # msg.data contains raw PCM audio bytes
        # Write directly to the output audio stream for playback
        self.stream.write(msg.data)

    def spin(self):
        rospy.spin()
        
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()

if __name__ == '__main__':
    # You can override the audio topic name here by passing _topic:=<topic_name> as ROS param
    topic = rospy.get_param('~topic', '/humans/voices/anonymous_speaker/audio')
    
    # If you want to change the audio device, modify device_name_substr below,
    # or hardcode device index in the class above after inspecting device list logs
    player = AudioPlayerNode(topic)
    player.spin()
