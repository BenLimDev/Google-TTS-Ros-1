GTTS Speech - ROS Text-to-Speech Package

A high-performance ROS text-to-speech node that combines Google Text-to-Speech (gTTS) with offline TTS fallback. Features dual audio generation, intelligent caching, and robust offline operation.

## Quick Start

```bash
# Install dependencies
pip install gtts pyttsx3 pygame pydub pyyaml
sudo apt-get install ffmpeg espeak espeak-data libespeak-dev

# Launch the node
roslaunch gtts_speech basic.launch

# Send text to speech
rostopic pub /text_to_speech std_msgs/String "data: 'Hello World'"
```


## Usage

```bash
# Basic launch
roslaunch gtts_speech basic.launch

# Publish messages
rostopic pub /text_to_speech std_msgs/String "data: 'Your message'"
```

### Key Parameters
- `language`: Language code (default: "en")
- `enable_cache`: Enable audio caching (default: true)
- `offline_voice_rate`: Speech rate for offline TTS (default: 150)
- `gtts_timeout`: gTTS timeout in seconds (default: 3.0)

## Integration with Nodes

**Python:**
```python
from std_msgs.msg import String
tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
tts_pub.publish(String("Hello from my node"))
```

**C++:**
```cpp
ros::Publisher tts_pub = nh.advertise<std_msgs::String>("/text_to_speech", 10);
std_msgs::String msg;
msg.data = "Hello from C++";
tts_pub.publish(msg);
```

## Features

- **Dual TTS**: gTTS (online) + pyttsx3 (offline) with 3-second smart fallback
- **Audio Caching**: WAV file caching for faster repeated playback
- **Non-blocking**: Asynchronous processing, never blocks message reception
- **Offline Ready**: Full functionality without internet connection
