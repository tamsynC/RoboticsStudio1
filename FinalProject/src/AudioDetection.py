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
import threading



class AudioComparisonNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_comparison_node")
        self.threads = ThreadPoolExecutor(max_workers=6)  # 1 thread for audio comparison
        self.lock = threading.Lock()
        self.threshold = 1000
        self.setSampleRate = 4000
        self.compareSizeDivider = 6
        # File Loading
        self.reference_data, self.ref_sr = librosa.load('/home/jet/ros2_ws/src/audio_common/audio_common/samples/explosion.mp3', sr=self.setSampleRate)
        self.get_logger().info(f"Loaded reference audio with sample rate {self.setSampleRate}")

        self.sampleTime = librosa.get_duration(y = self.reference_data, sr = self.ref_sr) #Reference audio duration

        self.liveAudioBuffer = np.zeros(int(self.sampleTime*self.ref_sr))  #Rolling recording the live audio scaling to reference duration

        #Plotting
        self.fig, self.ax = plt.subplots(nrows=4, sharex=True)
        self.ax[0].set(title='Sample Audio File')
        self.ax[1].set(title='Live Audio')
        self.ax[2].set(title='Live Audio Snippet')
        self.ax[3].set(title='Reference Audio Snippet')

        #non changing plots
        librosa.display.waveshow(self.reference_data, sr=self.setSampleRate, ax=self.ax[0]) #full ref
        plt.pause(0.01)
        librosa.display.waveshow(self.reference_data[:(int(len(self.liveAudioBuffer)/7))], sr=self.setSampleRate, ax=self.ax[3]) #ref snippet
        plt.pause(0.01)

        plt.ion()
        plt.show()



        # Create a subscription to the audio topic
        qos_profile = qos_profile_sensor_data
        self.sub = self.create_subscription(AudioStamped, "/audio", self.audio_callback, qos_profile)

        self.get_logger().info("AudioComparison node started")





    def audio_callback(self, msg: AudioStamped) -> None:
        # Convert the received audio message to a NumPy array
        subbedAudio = msg_to_array(msg.audio)

        if subbedAudio is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return
            


        self.subbedSRRate = msg.audio.info.rate
        subbedAudio = nr.reduce_noise(y=subbedAudio, sr=msg.audio.info.rate, prop_decrease=1, n_jobs=1) #opt


        # Convert audio data to float and normalize
        liveAudio = subbedAudio.astype(np.float32)

        # Append live audio to the buffer and discard the oldest data
        buffer_size = len(self.liveAudioBuffer)
        liveAudioSize = len(liveAudio)
        
        if liveAudioSize > buffer_size:
            self.liveAudioBuffer = liveAudio[-buffer_size:]
        else:
            self.liveAudioBuffer = np.roll(self.liveAudioBuffer, -liveAudioSize)
            self.liveAudioBuffer[-liveAudioSize:] = liveAudio

        minLength = int(len(self.liveAudioBuffer)/self.compareSizeDivider) #Snippet size to compare audio signals

        self.liveAudio = liveAudio[:minLength]
        self.refAudio = self.reference_data[:minLength] #Cutting refAudio into a chunk to process easier

        self.threads.submit(self.run_audio_comparison, self.liveAudio, self.refAudio) #graph gets stuck if audio comparison is on same thread
        self.plotWaveforms()

    def plotWaveforms(self):

        #Whole live audio
        self.ax[1].cla()  # Clear the second plot (live audio)
        librosa.display.waveshow(self.liveAudioBuffer, sr=self.setSampleRate, ax=self.ax[1])
        self.ax[1].set(title='Live Audio')
        plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this


        #Snippet of live audio
        self.ax[2].cla()  # Clear the second plot (live snippet audio)
        librosa.display.waveshow(self.liveAudio, sr=self.setSampleRate, ax=self.ax[2])
        self.ax[2].set(title='Live Audio Snippet')
        plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this


#THREAD START

    def run_audio_comparison(self, liveAudio: np.ndarray, refAudio: np.ndarray) -> None:
        try: 
            similarity = self.compare_audio(liveAudio, refAudio)

            if similarity > self.threshold:
                self.get_logger().info(f"Audio matches with similarity {similarity:.2f} (above threshold)")
            else:
                self.get_logger().info(f"Audio does not match, similarity {similarity:.2f} (below threshold)")
        except Exception as e:
            self.get_logger().error(f"Error in audio comparison: {str(e)}")

    def compare_audio(self, liveAudio: np.ndarray, refAudio: np.ndarray) -> float:
        # Adjust lengths for comparison


        # Compute similarity using correlation
        # correlation = np.correlate(liveAudio, refAudio, mode='valid')
        # similarity = np.max(np.abs(correlation)) / (np.linalg.norm(liveAudio) * np.linalg.norm(refAudio))

        xsim = librosa.segment.cross_similarity(liveAudio, refAudio, k=2)
        similarity = np.sum(xsim)

        self.get_logger().info(str(similarity))

        return similarity

#THREAD END


def main(args=None):
    rclpy.init(args=args)

    node = AudioComparisonNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





