#This node subscribes to /audio topic, compiles multiple audio segments then analyses them, comparing to a reference audio signal
#Also checks for too high or too low sound volume


# AUDIO sources https://pixabay.com/sound-effects/search/explosion/

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



#       INSTALL TESTING (to see if it works)

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
from concurrent.futures import ThreadPoolExecutor, Future
import threading

from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge


class AudioComparisonNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_comparison_node")
        self.threads = ThreadPoolExecutor(max_workers=10)  # 1 thread for audio comparison
        self.lock = threading.Lock()
        self.threshold = 1000
        self.setSampleRate = 4300*2
        self.compareSizeDivider = 2
        self.hop_length = 128 #fidelity / resolution of cross similarity map
        self.counter = 0
        self.counter2 = 0
        self.maxSoundVol = 2000
        self.minSoundVol= 0.00001 #in a silent room my microphone reads about 0.00025
        self.thresholdVolume = 500
        self.current_task: Future = None
        self.n_fft = 1024
        self.comparison_done = False  # Flag to track if comparison is done for current graph
        self.savedSubbedAudio = np.zeros(10)



        # File Loading
        self.reference_data, self.ref_sr = librosa.load('/home/jet/ros2_ws/src/audio_common/audio_common/samples/explosion.mp3', sr=self.setSampleRate)
        self.get_logger().info(f"Loaded reference audio with sample rate {self.setSampleRate}")

        self.referenceDuration = librosa.get_duration(y = self.reference_data, sr = self.ref_sr) #Reference audio duration

        self.liveAudioBuffer = np.zeros(int(self.referenceDuration*self.ref_sr))  #Rolling recording the live audio scaling to reference duration

        self.reference_data = librosa.util.normalize(self.reference_data) # normalise 
        self.minLength = int(len(self.liveAudioBuffer)/self.compareSizeDivider) #Snippet size to compare audio signals
        self.refAudio = self.reference_data[:self.minLength]
        #Plotting
        self.fig, self.ax = plt.subplots(nrows=6, sharex=True, gridspec_kw={'height_ratios': [1, 1, 1, 1, 4, 4]})
        self.ax[0].set(title='Sample Audio File')
        self.ax[1].set(title='Live Audio')
        self.ax[2].set(title='Live Audio Snippet')
        self.ax[3].set(title='Reference Audio Snippet')
        self.ax[4].set(title='Cross Similarity')

        #non changing plots
        librosa.display.waveshow(self.reference_data, sr=self.setSampleRate, ax=self.ax[0]) #full ref
        # plt.pause(0.01)
        librosa.display.waveshow(self.refAudio, sr=self.setSampleRate, ax=self.ax[3]) #ref snippet
        plt.pause(0.01)

        plt.ion()
        plt.show()
        chroma_ref = librosa.feature.chroma_stft(y=self.refAudio, sr=self.setSampleRate, hop_length=self.hop_length, n_chroma=12, n_fft=self.n_fft)

        # chroma_ref = librosa.feature.chroma_cqt(y=self.refAudio, sr=self.setSampleRate, hop_length=self.hop_length, n_chroma=12)
        self.x_ref = librosa.feature.stack_memory(chroma_ref, n_steps=2, delay=3)


        # Create a subscription to the audio topic
        qos_profile = qos_profile_sensor_data
        self.sub = self.create_subscription(AudioStamped, "/audio", self.audio_callback, qos_profile)
        self.timer = self.create_timer((self.referenceDuration/self.compareSizeDivider), self.sample_time_callback) #call timer every x seconds
        self.get_logger().info("AudioComparison node started")



    def sample_time_callback(self):
        self.threads.submit(self.run_audio_comparison)   

    def audio_callback(self, msg: AudioStamped) -> None:
        # Convert the received audio message to a NumPy array
        subbedAudio = msg_to_array(msg.audio)

        if subbedAudio is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return
        
        subbedAudio = nr.reduce_noise(y=subbedAudio, sr=msg.audio.info.rate, prop_decrease=1) #opt

        # Convert audio data to float and normalize

        self.liveAudio = subbedAudio.astype(np.float16)
        print("byte size",(self.liveAudio.nbytes))

        self.liveAudio = librosa.util.normalize(self.liveAudio, threshold=self.thresholdVolume, fill=False)

        self.volumeCheck() #graph gets stuck if audio comparison is on same thread

        if len(self.liveAudio[self.liveAudio != 0]) > 1000: # if there is a non empty message
            # Append live audio to the buffer and discard the oldest data
            liveAudioSize = len(self.liveAudio)
            
            if liveAudioSize > len(self.liveAudioBuffer):
                self.liveAudioBuffer = self.liveAudio[-len(self.liveAudioBuffer):]
            else:
                self.liveAudioBuffer = np.roll(self.liveAudioBuffer, -liveAudioSize) #shifts whole buffer left
                self.liveAudioBuffer[-liveAudioSize:] = self.liveAudio #put new chunk of audio on teh end of buffer


            # if len(self.liveAudioBuffer[self.liveAudioBuffer != 0]) >= self.minLength:

            #         # thready.result()
            #         # self.run_audio_comparison(self.liveAudio)
        
            self.liveAudio = self.liveAudioBuffer[:self.minLength]

        # if self.liveAudio

        # if self.counter == 0 and len(self.liveAudio[self.liveAudio != 0]) > 100: #checks there is actual audio within the message

        # self.counter+=1
        print("byte size2",(self.liveAudio.nbytes))

        self.plotWaveforms()

    def volumeCheck(self):
        
        if np.max(self.liveAudio) > self.maxSoundVol:
            self.get_logger().info(f"Loud Environment {np.max(self.liveAudio): .10f} Max Sound Recorded")
        if np.mean(self.liveAudio) < self.minSoundVol and np.mean(self.liveAudio) > 0: #idk why but mic reading goes - sometimes
            self.get_logger().info(f"Average sound level of {np.mean(self.liveAudio): .10f} is unusually low, please test microphone")

    def plotWaveforms(self):

        #Whole live audio
        self.ax[1].cla()  # Clear the second plot (live audio)
        librosa.display.waveshow(self.liveAudioBuffer, sr=self.setSampleRate, ax=self.ax[1])
        self.ax[1].set(title='Live Audio')
        # plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this

        #Snippet of live audio
        if len(self.liveAudio[self.liveAudio != 0]) > 100: #checks there is actual audio within the message
            self.ax[2].cla()  # Clear the second plot (live snippet audio)
            librosa.display.waveshow(self.liveAudio, sr=self.setSampleRate, ax=self.ax[2])
            self.ax[2].set(title='Live Audio Snippet')

        try:
            librosa.display.specshow(self.xsim, x_axis='s', y_axis='s', cmap='magma_r', hop_length=self.hop_length, ax=self.ax[4])
        except Exception as e:
            self.get_logger().error(f"Error in audio comparison: {str(e)}")
        plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this



        try:
            self.ax[5].cla()  # Clear the second plot (live snippet audio)
            # librosa.display.waveshow(self.DIAGNOSTIC, sr=self.setSampleRate, ax=self.ax[5])
            librosa.display.specshow(self.DIAGNOSTIC, x_axis='s', y_axis='s', cmap='magma_r', hop_length=self.hop_length, ax=self.ax[5])
            # plt.plot(np.linspace(0,self.setSampleRate, len(self.DIAGNOSTIC)), self.DIAGNOSTIC)
            # plt.show()
            # self.ax[5].set(title='DIAGNOSTIC')
        except Exception as e:
            self.get_logger().error(f"diag: {str(e)}")

#THREAD START

    def run_audio_comparison(self) -> None:
        try: 
            similarity = self.compare_audio()

            if similarity > self.threshold:
                self.get_logger().info(f"Audio matches with similarity {similarity:.2f} (above threshold)")
            else:
                self.get_logger().info(f"Audio does not match, similarity {similarity:.2f} (below threshold)")
        except Exception as e:
            self.get_logger().error(f"Error in audio comparison: {str(e)}")


    def compare_audio(self) -> float:
        if len(self.liveAudio[self.liveAudio != 0]) > 1000: # if there is a non empty message

            # self.liveAudio, self.ref_sr = librosa.load('/home/jet/ros2_ws/src/audio_common/audio_common/samples/explosion.mp3', sr=self.setSampleRate, offset=0) #use to test if function is working
            self.DIAGNOSTIC = np.fft.fft(self.liveAudio)
            self.DIAGNOSTIC = np.abs(self.DIAGNOSTIC)
            min_len = min(len(self.liveAudio), len(self.refAudio))
            self.liveAudio = self.liveAudio[:min_len]

            # self.get_logger().info(f"Reference audio length: {len(refAudio)}, Live audio length: {len(liveAudio)}")
            # self.get_logger().info(f"Live audio sample: {liveAudio[:10]}")
            # self.get_logger().info(f"Reference audio sample: {refAudio[:10]}")

            #NEW CODE

            # chroma_comp = librosa.feature.chroma_cqt(y=liveAudio, sr=self.setSampleRate, hop_length=self.hop_length, n_chroma=12)


            chroma_comp = librosa.feature.chroma_stft(y=self.liveAudio, sr=self.setSampleRate, hop_length=self.hop_length, n_chroma=12, n_fft=self.n_fft)
            self.DIAGNOSTIC = chroma_comp


            # Use time-delay embedding to get a cleaner recurrence matrix
            x_comp = librosa.feature.stack_memory(chroma_comp, n_steps=2, delay=1)

            #NEW CODE
            # similarity = np.correlate(x_comp, self.x_ref,"full")

            self.xsim = librosa.segment.cross_similarity(x_comp, self.x_ref, mode='affinity', metric='cosine')        
            # self.get_logger().info(str(similarity))
            similarity = np.sum(self.xsim)

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





