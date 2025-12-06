# Voice-to-Action using OpenAI Whisper

Enabling robots to understand and act upon spoken commands is a significant step towards more natural and intuitive human-robot interaction. OpenAI Whisper, a powerful speech-to-text model, provides an excellent foundation for translating human voice instructions into machine-readable commands that a robot can then execute.

## 1.1 Introduction to Voice Commands in Robotics

### Motivation: Natural human-robot interaction

Traditional robot interfaces often rely on joysticks, programming languages, or graphical user interfaces. While effective for trained operators, these methods can be cumbersome and unnatural for casual users or in situations requiring quick, hands-free interaction. Voice commands offer a more intuitive and accessible way for humans to communicate with robots, mimicking natural human-human communication.

Benefits include:

*   **Accessibility:** Allows users without specialized training to interact with robots.
*   **Hands-Free Operation:** Crucial in environments where a human's hands are occupied (e.g., surgical assistance, assembly tasks).
*   **Speed and Efficiency:** Can convey complex instructions faster than typing or clicking.
*   **Intuitive:** Aligns with human communication patterns, reducing cognitive load.

### Challenges: Speech recognition accuracy, noise, latency

Implementing robust voice command systems in robotics comes with several challenges:

*   **Speech Recognition Accuracy:** Accurately transcribing diverse human speech, including varying accents, speech rates, and vocabulary.
*   **Noise Robustness:** Real-world environments are often noisy (e.g., robot motors, background conversations, factory sounds). The speech recognition system must be able to filter out noise and accurately capture the commands.
*   **Latency:** The delay between a spoken command and the robot's action needs to be minimal for a natural and responsive interaction. High latency can lead to frustration and potential safety issues.
*   **Command Parsing and Understanding:** After transcription, the raw text needs to be understood semantically and translated into executable robot actions. This involves natural language understanding (NLU) to interpret intent, entities, and parameters.
*   **Safety and Redundancy:** Voice commands alone might not be sufficient for safety-critical applications. Redundant safety mechanisms are often required.

## 1.2 Overview of OpenAI Whisper

OpenAI Whisper is a general-purpose speech recognition model developed by OpenAI. It was trained on a massive dataset of diverse audio and corresponding textual transcripts, enabling it to achieve high accuracy and robustness across various languages and conditions.

### What is Whisper? Large language model for speech-to-text

Whisper is an encoder-decoder Transformer model that takes raw audio as input and outputs the corresponding text. Unlike previous speech recognition models that often struggled with varying languages, accents, and background noise, Whisper's training on a vast and diverse dataset makes it highly capable in these areas.

### Key features: High accuracy, multilingual support, robust to noise

*   **High Accuracy:** Whisper achieves state-of-the-art results on many speech recognition benchmarks, often outperforming previous models, especially on challenging audio.
*   **Multilingual Support:** Trained on multilingual data, it can transcribe and translate speech in many languages.
*   **Robust to Noise:** Its extensive training data, including noisy audio, makes it highly resilient to background noise and environmental distractions.
*   **Speaker Diarization (limited):** While not its primary focus, Whisper can sometimes implicitly distinguish between speakers in a conversation.
*   **Punctuation and Capitalization:** Generates text with good punctuation and capitalization, improving readability and downstream natural language processing.

### Deployment options: API, local models

Whisper can be accessed in a few ways:

*   **OpenAI API:** The easiest way to use Whisper, offering various models (tiny, base, small, medium, large) with different accuracy-to-speed trade-offs. It handles the underlying computational complexity.
*   **Local Models:** OpenAI has open-sourced the Whisper models, allowing developers to run them locally on their own hardware (CPU or GPU). This is ideal for applications requiring offline functionality, custom integration, or specific privacy requirements. Several community-driven projects have also optimized Whisper for local deployment (e.g., `whisper.cpp`, `faster-whisper`).

## 1.3 Integrating Whisper for Robot Command Recognition

Integrating Whisper into a robot's control system involves capturing audio, transcribing it, and then processing the text to extract commands.

### Setting up Whisper (API client or local environment)

*   **API Client:**
    1.  Install the OpenAI Python library: `pip install openai`
    2.  Obtain an OpenAI API key and set it as an environment variable (`OPENAI_API_KEY`).
    3.  Use the `openai.Audio.transcribe` method.
*   **Local Environment:**
    1.  Install the `whisper` Python package: `pip install -U openai-whisper`
    2.  Choose a model size (`tiny`, `base`, `small`, `medium`, `large`) based on your accuracy and performance needs.
    3.  Load the model: `model = whisper.load_model("base")`

### Capturing audio input from microphone

For real-time voice commands, you need to capture audio from the robot's (or user's) microphone. Python libraries like `sounddevice` or `PyAudio` can be used for this.

Key considerations:

*   **Sampling Rate:** Match the sampling rate expected by Whisper (typically 16kHz).
*   **Audio Chunking:** Process audio in small chunks to maintain responsiveness.
*   **Voice Activity Detection (VAD):** Optionally, use VAD to detect when a user is speaking and filter out silence, improving efficiency and reducing processing load.

### Transcribing speech to text commands

Once audio is captured, it's passed to Whisper for transcription.

```python
import whisper

# Load the desired Whisper model
# model = whisper.load_model("base")

def transcribe_audio_file(audio_path):
    """Transcribes an audio file using the local Whisper model."""
    result = model.transcribe(audio_path)
    return result["text"]

# Example usage (requires an audio file)
# text = transcribe_audio_file("my_command.wav")
# print(f"Transcribed text: {text}")
```
For real-time streaming, you would buffer audio chunks and periodically send them to Whisper for transcription, or use a streaming-optimized Whisper implementation.

## 1.4 A Simple Python Example of using the Whisper API

This example outlines how you might use Whisper (conceptual API or local) to process a voice command and rudimentary parsing.

```python
import speech_recognition as sr
import openai # For OpenAI API, or local whisper if preferred
import time

# --- Configuration ---
# Use a local Whisper model
# import whisper
# model = whisper.load_model("base") 

# Or use OpenAI API
# openai.api_key = "YOUR_OPENAI_API_KEY" # Replace with your actual API key or use environment variable

def get_audio_from_mic():
    """Captures audio from the microphone until silence is detected."""
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Say something!")
        # Adjust for ambient noise for better recognition
        r.adjust_for_ambient_noise(source) 
        audio = r.listen(source, phrase_time_limit=5) # Listen for up to 5 seconds of speech
    print("Audio captured. Processing...")
    return audio

def transcribe_audio_whisper(audio_data):
    """Transcribes audio data using OpenAI Whisper (API or local)."""
    try:
        # Save audio to a temporary WAV file (required by Whisper for file input)
        with open("temp_command.wav", "wb") as f:
            f.write(audio_data.get_wav_data())

        # Option 1: Using local Whisper model
        # transcribed_text = model.transcribe("temp_command.wav")["text"]
        
        # Option 2: Using OpenAI Whisper API
        with open("temp_command.wav", "rb") as audio_file:
            response = openai.audio.transcriptions.create(
                model="whisper-1", 
                file=audio_file, 
                language="en" # Specify language for better accuracy
            )
        transcribed_text = response.text

        return transcribed_text.strip().lower()
    except Exception as e:
        print(f"Could not transcribe audio: {e}")
        return ""

def parse_robot_command(text):
    """Simple parsing of transcribed text into a robot action and parameters."""
    if "move forward" in text:
        return {"action": "move", "direction": "forward", "distance": 1.0}
    elif "move backward" in text:
        return {"action": "move", "direction": "backward", "distance": 1.0}
    elif "turn left" in text:
        return {"action": "turn", "direction": "left", "angle": 90}
    elif "turn right" in text:
        return {"action": "turn", "direction": "right", "angle": 90}
    elif "stop" in text:
        return {"action": "stop"}
    else:
        return {"action": "unknown", "command": text}

def execute_robot_action(command_data):
    """Placeholder for actually sending commands to a ROS 2 robot."""
    if command_data["action"] == "move":
        print(f"Robot command: Move {command_data['direction']} by {command_data['distance']} meters")
        # Here you would typically publish a Twist message or call a ROS 2 action
    elif command_data["action"] == "turn":
        print(f"Robot command: Turn {command_data['direction']} by {command_data['angle']} degrees")
        # Here you would typically publish a Twist message with angular velocity
    elif command_data["action"] == "stop":
        print("Robot command: Stop all motion")
        # Here you would typically publish a Twist message with zero velocities
    else:
        print(f"Unknown command: {command_data['command']}")

if __name__ == "__main__":
    while True:
        audio_data = get_audio_from_mic()
        if audio_data:
            transcribed_text = transcribe_audio_whisper(audio_data)
            print(f"Transcribed: {transcribed_text}")
            command = parse_robot_command(transcribed_text)
            execute_robot_action(command)
        else:
            print("No audio detected.")
        time.sleep(1) # Wait a bit before listening again
```

This example shows the basic flow: capturing audio, transcribing it with Whisper, and then a rudimentary parsing function to extract actionable commands. In a real [ROS 2](pathname:///docs/module1-ros2/introduction) system, `execute_robot_action` would involve publishing messages to topics (e.g., `geometry_msgs/msg/Twist` for movement) or calling [ROS 2](pathname:///docs/module1-ros2/introduction) services/actions.
