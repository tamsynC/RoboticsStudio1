# AUDIO sources https://pixabay.com/sound-effects/search/explosion/



#       tried but suck
# https://github.com/ros-drivers/audio_common/tree/ros2 (link)
# git clone https://github.com/ros-drivers/audio_common.git -b ros2 (do this instead)
# sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio



#       INSTALLATION

# AUDIO NODE USED for publisher and subscribers
# https://github.com/mgonzs13/audio_common                      (Complete download explained here first) 

# https://librosa.org/doc/latest/feature.html                   (Audio analysing)
#   sudo pip install librosa

# https://pypi.org/project/noisereduce/                         (for removing noise / interference)
#   pip install noisereduce

#   sudo apt install ffmpeg                                     (required extra pkg)

#   sudo pip uninstall numpy
#   pip install numpy==1.24.3 (numba no like new numpy) (numba is used by librosa)



#       INSTALL TESTING

# ros2 run audio_common audio_player_node
# ros2 run audio_common music_node
# ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: 'elevator'}"
# ALT SYNTAX ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: /home/jet/ros2_ws/src/audio_common/audio_common/samples/elevator}"



#       CODE METHOD

# Check Pos
# If pos matches, send sound to subscriber
# Check if sound is over decimal range
# check for explosion (compare audio from subscriber to sampled audio)

# play audio after set amount of time or when turtlebot reaches location?


import pyaudio
import numpy as np
import librosa
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audio_common_msgs.msg import AudioStamped
from audio_common.utils import msg_to_array
import matplotlib.pyplot as plt
import noisereduce as nr


from concurrent.futures import ThreadPoolExecutor

class AudioComparisonNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_comparison_node")
        self.threads = ThreadPoolExecutor(max_workers=6)  # 1 thread for audio comparison

        # Load the reference audio file (MP3)
        self.reference_data, self.ref_sr = librosa.load('/home/jet/ros2_ws/src/audio_common/audio_common/samples/explosion.mp3')
        self.get_logger().info(f"Loaded reference audio with sample rate {self.ref_sr}")

        # Declare parameters for audio comparison
        self.declare_parameters("", [
            ("threshold", 0.1),  # Similarity threshold
            ("channels", 1),
            ("device", -1),
        ])

        self.sampleTime = librosa.get_duration(y = self.reference_data, sr = self.ref_sr)

        self.live_audio_buffer = np.zeros(int(self.sampleTime*self.ref_sr*0.727272))  # sample time * self.ref_sr Hz (polling rate)

        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        self.channels = self.get_parameter("channels").get_parameter_value().integer_value
        device = self.get_parameter("device").get_parameter_value().integer_value

        if device < 0:
            device = None

        self.min_length = int(len(self.live_audio_buffer)/7) # 1/7 total audio sample length
        self.ref_audio = self.reference_data[:self.min_length]

        #plotting
        self.fig, self.ax = plt.subplots(nrows=4, sharex=True)
        self.ax[0].set(title='Sample Audio File')
        self.ax[1].set(title='Live Audio')
        self.ax[2].set(title='Live Audio Snippet')
        self.ax[3].set(title='Reference Audio Snippet')

        librosa.display.waveshow(self.reference_data, sr=self.ref_sr, ax=self.ax[0])
        librosa.display.waveshow(self.ref_audio, sr=self.ref_sr, ax=self.ax[3])


        plt.ion()
        plt.show()



        # Create a subscription to the audio topic
        qos_profile = qos_profile_sensor_data
        self.sub = self.create_subscription(AudioStamped, "/audio", self.audio_callback, qos_profile)

        self.get_logger().info("AudioComparison node started")





    def audio_callback(self, msg: AudioStamped) -> None:
        # Convert the received audio message to a NumPy array
        array_data = msg_to_array(msg.audio)

        if array_data is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return
        self.liveSRRate = msg.audio.info.rate
        array_data = nr.reduce_noise(y=array_data, sr=msg.audio.info.rate, prop_decrease=1, n_jobs=1) #opt


        # Convert audio data to float and normalize
        live_audio = array_data.astype(np.float32)



        # Append live audio to the buffer and discard the oldest data
        buffer_size = len(self.live_audio_buffer)
        live_audio_size = len(live_audio)
        if live_audio_size > buffer_size:
            self.live_audio_buffer = live_audio[-buffer_size:]
        else:
            self.live_audio_buffer = np.roll(self.live_audio_buffer, -live_audio_size)
            self.live_audio_buffer[-live_audio_size:] = live_audio

        self.ax[1].cla()  # Clear the second plot (live audio)
        librosa.display.waveshow(self.live_audio_buffer, sr=msg.audio.info.rate, ax=self.ax[1])
        plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this

        self.ax[2].cla()  # Clear the second plot (live audio)
        librosa.display.waveshow(live_audio[:self.min_length], sr=self.liveSRRate, ax=self.ax[2])
        plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this


        test = self.live_audio_buffer
        self.threads.submit(self.run_audio_comparison, test) #graph gets stuck if audio comparison is on same thread



#THREAD START

    def run_audio_comparison(self, live_audio: np.ndarray) -> None:
        similarity = self.compare_audio(live_audio)
        if similarity > self.threshold:
            self.get_logger().info(f"Audio matches with similarity {similarity:.2f} (above threshold)")
        else:
            self.get_logger().info(f"Audio does not match, similarity {similarity:.2f} (below threshold)")

    def compare_audio(self, live_audio: np.ndarray) -> float:
        # Adjust lengths for comparison
        live_audio = live_audio[:self.min_length]

        # Compute similarity using correlation
        # correlation = np.correlate(live_audio, ref_audio, mode='valid')
        # similarity = np.max(np.abs(correlation)) / (np.linalg.norm(live_audio) * np.linalg.norm(ref_audio))

        xsim = librosa.segment.cross_similarity(live_audio, self.ref_audio, mode='affinity', metric='cosine', k=5)
        similarity = np.sum(xsim)
        return similarity

#THREAD END


def main(args=None):
    rclpy.init(args=args)

    # Path to your reference MP3 file

    node = AudioComparisonNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





