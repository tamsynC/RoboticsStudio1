#This node reads in audio from pc mic and publishes to /audio

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audio_common_msgs.msg import AudioStamped
from audio_common.utils import data_to_msg

class AudioPublisherNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_publisher_node")

        # Declare parameters for audio capture
        self.declare_parameters("", [
            ("format", pyaudio.paInt16),
            ("channels", 1),
            ("rate", 16000),
            ("chunk", 4096),
            ("device", -1),
            ("frame_id", "")
        ])

        # Retrieve parameters
        self.format = self.get_parameter("format").get_parameter_value().integer_value
        self.channels = self.get_parameter("channels").get_parameter_value().integer_value
        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        self.chunk = self.get_parameter("chunk").get_parameter_value().integer_value
        device = self.get_parameter("device").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        if device < 0:
            device = None

        # Initialize PyAudio and open an audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk,
            input_device_index=device
        )

        # Create a publisher for AudioStamped messages
        self.audio_pub = self.create_publisher(AudioStamped, "audio", qos_profile_sensor_data)

        self.get_logger().info("AudioPublisher node started")

    def destroy_node(self) -> bool:
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def publish_audio(self) -> None:
        while rclpy.ok():
            # Read audio data from the input stream
            data = self.stream.read(self.chunk)

            # Create and populate the AudioStamped message
            msg = AudioStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()

            audio_msg = data_to_msg(data, self.format)
            if audio_msg is None:
                self.get_logger().error(f"Format {self.format} unknown")
                return

            msg.audio = audio_msg
            msg.audio.info.channels = self.channels
            msg.audio.info.chunk = self.chunk
            msg.audio.info.rate = self.rate

            # Publish the message
            self.audio_pub.publish(msg)
            self.get_logger().info("Published audio data")

def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisherNode()
    try:
        node.publish_audio()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
