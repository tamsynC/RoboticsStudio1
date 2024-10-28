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

import argparse
import torch
import librosa
import numpy as np
from torch import autocast
from contextlib import nullcontext

from models.mn.model import get_model as get_mobilenet
from models.dymn.model import get_model as get_dymn
from models.ensemble import get_ensemble_model
from models.preprocess import AugmentMelSTFT
from helpers.utils import NAME_TO_WIDTH, labels
from std_msgs.msg import String



class AudioComparisonNode(Node):

    def __init__(self) -> None:

        super().__init__("audio_comparison_node")
        self.threads = ThreadPoolExecutor(max_workers=10)  # 1 thread for audio comparison
        self.lock = threading.Lock()
        self.threshold = 1000
        self.setSampleRate = 32000
        self.compareSizeDivider = 1
        self.hop_length = 320 #fidelity / resolution of cross similarity map
        self.counter = 0
        self.counter2 = 0
        self.maxSoundVol = 2000
        self.minSoundVol= 0.00001 #in a silent room my microphone reads about 0.00025
        self.thresholdVolume = 500
        self.current_task: Future = None
        self.n_fft = 1024
        self.comparison_done = False  # Flag to track if comparison is done for current graph
        self.savedSubbedAudio = np.zeros(10)
        self.cutLiveAudio = np.zeros(10)
        self.audioProcessFrequency = 5
        self.useCudaCores = True
        self.timer = False
        


        model_name = 'mn10_as'
        self.device = torch.device('cuda') if self.useCudaCores and torch.cuda.is_available() else torch.device('cpu')
        self.audio_path = "resources/help.mp3"
        window_size = 800
        n_mels = 128
        strides=[2, 2, 2, 2]
        head_type="mlp"


        self.model = get_mobilenet(width_mult=NAME_TO_WIDTH(model_name), pretrained_name=model_name,
                                strides=strides, head_type=head_type)

        self.LoudNoisePub = self.create_publisher(String, "/HQAudio", 10)
 
        self.model.to(self.device)
        self.model.eval()

        # model to preprocess waveform into mel spectrograms
        self.mel = AugmentMelSTFT(n_mels=n_mels, sr=self.setSampleRate, win_length=window_size, hopsize=self.hop_length)
        self.mel.to(self.device)
        self.mel.eval()

        # File Loading
        self.reference_data, self.ref_sr = librosa.load('/home/jet/ros2_ws/src/audio_common/audio_common/samples/explosion.mp3', sr=self.setSampleRate)
        self.get_logger().info(f"Loaded reference audio with sample rate {self.setSampleRate}")

        self.referenceDuration = librosa.get_duration(y = self.reference_data, sr = self.ref_sr) #Reference audio duration

        self.liveAudioBuffer = np.float16(np.zeros(int(self.referenceDuration*self.ref_sr)))  #Rolling recording the live audio scaling to reference duration

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

        self.x_ref = librosa.feature.stack_memory(chroma_ref, n_steps=2, delay=3)


        # Create a subscription to the audio topic
        qos_profile = qos_profile_sensor_data
        self.sub = self.create_subscription(AudioStamped, "/audio", self.audio_callback, qos_profile)
        # self.timer = self.create_timer((self.referenceDuration/self.compareSizeDivider), self.sample_time_callback) #call timer every x seconds

        self.get_logger().info("AudioComparison node started")



    def audio_callback(self, msg: AudioStamped) -> None:
        # Convert the received audio message to a NumPy array
        subbedAudio = msg_to_array(msg.audio)

        if subbedAudio is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return
        
        subbedAudio = nr.reduce_noise(y=subbedAudio, sr=msg.audio.info.rate, prop_decrease=1) #opt

        # Convert audio data to float and normalize

        self.liveAudio = subbedAudio.astype(np.float16) #CHANGE TO FLOAT16 FOR GPU
        print("byte size",(self.liveAudio.nbytes))

        self.liveAudio = librosa.util.normalize(self.liveAudio, threshold=self.thresholdVolume, fill=False)

        self.volumeCheck() #graph gets stuck if audio comparison is on same thread

        if len(self.liveAudio[self.liveAudio != 0]) > 1000: # if there is a non empty message
            
            # Append live audio to the buffer and discard the oldest data
            if self.timer == False:
                threading.Timer(5.0, self.run_audio_comparison).start()
                self.timer = True

            liveAudioSize = len(self.liveAudio)
            self.counter +=1
            if liveAudioSize > len(self.liveAudioBuffer):
                self.liveAudioBuffer = self.liveAudio[-len(self.liveAudioBuffer):]
            else:
                self.liveAudioBuffer = np.roll(self.liveAudioBuffer, -liveAudioSize) #shifts whole buffer left
                self.liveAudioBuffer[-liveAudioSize:] = self.liveAudio #put new chunk of audio on teh end of buffer
            if self.counter > self.audioProcessFrequency:
                
                self.counter = 0
            self.plotWaveforms()




        # print("byte size2",(self.liveAudio.nbytes))


    def volumeCheck(self):
        
        if np.max(self.liveAudio) > self.maxSoundVol:
            self.get_logger().info(f"Loud Environment {np.max(self.liveAudio): .10f} Max Sound Recorded")
            loudMsg = String()
            loudMsg.data = "Loud Noise Detected"
            self.LoudNoisePub.publish(loudMsg)
        if np.mean(self.liveAudio) < self.minSoundVol and np.mean(self.liveAudio) > 0: #idk why but mic reading goes - sometimes
            self.get_logger().info(f"Average sound level of {np.mean(self.liveAudio): .10f} is unusually low, please test microphone")

    def plotWaveforms(self):

        #Whole live audio
        self.ax[1].cla()  # Clear the second plot (live audio)
        librosa.display.waveshow(self.liveAudioBuffer, sr=self.setSampleRate, ax=self.ax[1])
        self.ax[1].set(title='Live Audio Buffer')
        # plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this

        #Snippet of live audio
        if len(self.liveAudio[self.liveAudio != 0]) > 100: #checks there is actual audio within the message
            self.ax[2].cla()  # Clear the second plot (live snippet audio)
            librosa.display.waveshow(self.cutLiveAudio, sr=self.setSampleRate, ax=self.ax[2])
            self.ax[2].set(title='Previously Analysed Snippet')

        # try:
        #     librosa.display.specshow(self.xsim, x_axis='s', y_axis='s', cmap='magma_r', hop_length=self.hop_length, ax=self.ax[4])
        # except Exception as e:
        #     self.get_logger().error(f"Error in audio comparison: {str(e)}")

        # try:
        #     self.ax[5].cla()  # Clear the second plot (live snippet audio)
        #     # librosa.display.waveshow(self.DIAGNOSTIC, sr=self.setSampleRate, ax=self.ax[5])
        #     librosa.display.specshow(self.DIAGNOSTIC, x_axis='s', y_axis='s', cmap='magma_r', hop_length=self.hop_length, ax=self.ax[5])
        #     # plt.plot(np.linspace(0,self.setSampleRate, len(self.DIAGNOSTIC)), self.DIAGNOSTIC)
        #     # plt.show()
        #     # self.ax[5].set(title='DIAGNOSTIC')
        # except Exception as e:
        #     self.get_logger().error(f"diag: {str(e)}")
        plt.pause(0.01)  # Pause to allow the plot to update, it freaks out lower than this



    def run_audio_comparison(self) -> None:
        if self.timer == True:
            self.timer = False
        try: 
            preds = self.compare_audio()

            sorted_indexes = np.argsort(preds)[::-1]
            speech = self.process_predictions(sorted_indexes, preds, "Speech")

            coinDrop = self.process_predictions(sorted_indexes, preds, "Coin (dropping)")
            thunder = self.process_predictions(sorted_indexes, preds, "Thunder")
            gunshot = self.process_predictions(sorted_indexes, preds, "Gunshot, gunfire")
            mGun = self.process_predictions(sorted_indexes, preds, "Machine gun")
            jHammer = self.process_predictions(sorted_indexes, preds, "Jackhammer")
            breathing = self.process_predictions(sorted_indexes, preds, "Breathing")
            coughing = self.process_predictions(sorted_indexes, preds, "Cough")
            wheeze = self.process_predictions(sorted_indexes, preds, "Wheeze")
            pant = self.process_predictions(sorted_indexes, preds, "Pant")




            if coinDrop + thunder + jHammer > 0.9:
                self.get_logger().info(f"High Probability of explosion Here")

            if gunshot > 0.4 or mGun > 0.4:
                self.get_logger().info(f"High Probability of gunFire Here")

            if speech > 0.4 or breathing > 0.4 or coughing > 0.4 or wheeze > 0.4 or pant > 0.4:
                self.get_logger().info(f"High Probability of human Here")
       

            print("--------Top 10 probabilities----------")
            for k in range(10):
                print('{}: {:.3f}'.format(labels[sorted_indexes[k]],
                    preds[sorted_indexes[k]]))
            print("--------------------------------------")


        except Exception as e:
            self.get_logger().error(f"Error in audio comparison: {str(e)}")


    def process_predictions(self, sorted_indexes, preds, target_label):

        for k in range(len(preds)):
            label = labels[sorted_indexes[k]]
            value = preds[sorted_indexes[k]]
            if label == target_label:
                return value

    def compare_audio(self) -> float:

        self.cutLiveAudio = self.liveAudioBuffer[:self.minLength]
        audioProcessing = self.liveAudioBuffer[:self.minLength]
        audioProcessing = torch.from_numpy(audioProcessing[None, :]).to(self.device)

        with torch.no_grad(), autocast(device_type=self.device.type) if self.useCudaCores else nullcontext():
            spec = self.mel(audioProcessing)
            preds, features = self.model(spec.unsqueeze(0))
        preds = torch.sigmoid(preds.float()).squeeze().cpu().numpy()


        self.liveAudioBuffer = np.float16(np.zeros(len(self.liveAudioBuffer))) #reset buffer
        return preds
    

def main(args=None):
    rclpy.init(args=args)

    node = AudioComparisonNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()





