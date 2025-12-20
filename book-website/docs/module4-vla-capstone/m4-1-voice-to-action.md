---
title: The Speech Processing Front-End (Whisper)
---

# The Speech Processing Front-End (Whisper)

The first component of our Vision-Language-Action (VLA) pipeline is the speech processing front-end. This system will take raw audio input from a microphone, transcribe it into text, and publish that text to a ROS 2 topic. For this, we will leverage **OpenAI Whisper**, a powerful and robust Automatic Speech Recognition (ASR) model.

## Implementing the Whisper Listener Node

We will create a ROS 2 Python node that listens to audio input, calls the Whisper API, and then publishes the transcribed text.

### 1. Setup OpenAI API Key

First, ensure your OpenAI API key is securely managed. As per our specification, we will use environment variables.

```bash
export OPENAI_API_KEY="YOUR_API_KEY"
```

### 2. Create the ROS 2 Whisper Listener Node

Create a new Python file, `whisper_listener_node.py`, within a ROS 2 package (e.g., `vla_ros_nodes`).

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import openai
import os
import threading

class WhisperListenerNode(Node):
    def __init__(self):
        super().__init__('whisper_listener_node')
        self.publisher_ = self.create_publisher(String, '/command_text', 10)
        self.get_logger().info('Whisper Listener Node has been started.')

        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.sample_rate = 16000
        self.chunk = 1024
        self.record_seconds = 5 # Record for 5 seconds

        self.audio = pyaudio.PyAudio()
        self.stream = None

        # Check for OpenAI API Key
        openai.api_key = os.getenv("OPENAI_API_KEY")
        if not openai.api_key:
            self.get_logger().error("OPENAI_API_KEY environment variable not set!")
            raise ValueError("OPENAI_API_KEY environment variable not set!")

        self.recording = False
        self.recording_thread = None

        self.create_timer(1.0, self.check_for_commands) # Check for commands periodically

    def check_for_commands(self):
        # In a real system, this would be triggered by a wake word or a button press.
        # For simplicity, we'll simulate a command trigger every few seconds.
        if not self.recording:
            self.get_logger().info('Simulating command trigger. Starting recording...')
            self.start_recording()

    def start_recording(self):
        if not self.recording:
            self.recording = True
            self.recording_thread = threading.Thread(target=self._record_audio)
            self.recording_thread.start()

    def _record_audio(self):
        frames = []
        self.stream = self.audio.open(format=self.audio_format,
                                      channels=self.channels,
                                      rate=self.sample_rate,
                                      input=True,
                                      frames_per_buffer=self.chunk)
        self.get_logger().info('Recording...')
        for _ in range(0, int(self.sample_rate / self.chunk * self.record_seconds)):
            data = self.stream.read(self.chunk)
            frames.append(data)
        self.get_logger().info('Finished recording.')

        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        self.recording = False

        self._process_audio(frames)

    def _process_audio(self, frames):
        # Save audio to a temporary WAV file
        wf = wave.open("temp_command.wav", 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.audio.get_sample_size(self.audio_format))
        wf.setframerate(self.sample_rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        # Call Whisper API
        try:
            with open("temp_command.wav", "rb") as audio_file:
                response = openai.Audio.transcribe("whisper-1", audio_file)
            transcribed_text = response["text"]
            self.get_logger().info(f'Transcribed: "{transcribed_text}"')

            # Publish to ROS 2 topic
            msg = String()
            msg.data = transcribed_text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}" to /command_text')

        except Exception as e:
            self.get_logger().error(f"Error calling Whisper API: {e}")
            # FR-009: Provide audible feedback for transcription failure
            self.get_logger().warn("I'm sorry, I didn't understand that. Please try again.")

        finally:
            if os.path.exists("temp_command.wav"):
                os.remove("temp_command.wav")

def main(args=None):
    rclpy.init(args=args)
    node = WhisperListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This node will:
-   Record 5 seconds of audio using `pyaudio`.
-   Save the audio to a temporary WAV file.
-   Send the WAV file to the OpenAI Whisper API for transcription.
-   Publish the transcribed text to the `/command_text` ROS 2 topic.
-   Implement `FR-009` by logging a warning message to the user when transcription fails.

## Integrating Whisper Output into ROS 2

Once the `whisper_listener_node` is running, you can verify its output using ROS 2 command-line tools:

```bash
# Listen to the published text commands
ros2 topic echo /command_text
```

This establishes the crucial voice-to-text front-end for our VLA pipeline. In the next section, we will use this text input to drive cognitive planning with the Gemini API.
