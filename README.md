GTTS Speech - ROS Text-to-Speech Package

A high-performance Google Text-to-Speech (GTTS) ROS node with smart caching and multiple accent support.

## Features

🎯 **Smart Caching** - Instant playback for repeated phrases  
🌍 **Multiple Accents** - US, British, Australian voices  
⚡ **Fast Response** - Preloaded common phrases  
💰 **Completely Free** - Uses Google TTS API  
🔧 **Easy Integration** - Standard ROS topics  

## Demo

![GTTS Demo](https://img.shields.io/badge/ROS-Melodic%20%7C%20Noetic-blue)
![Python](https://img.shields.io/badge/Python-3.6%2B-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## Quick Start

### Prerequisites
- ROS (Melodic/Noetic)
- Python 3.6+
- Internet connection (for TTS generation)

### Installation

```bash
# 1. Clone the repository
cd ~/catkin_ws/src
git clone https://github.com/BenLimDev/gtts_speech

# 2. Install Python dependencies
pip3 install gtts pygame

# 3. Build the package
cd ~/catkin_ws
catkin_make

# 4. Source your workspace
source devel/setup.bash
```

### Usage

```bash
# Start the TTS node (US accent)
roslaunch gtts_speech basic.launch

# Send text to be spoken
rostopic pub /text_to_speech std_msgs/String 'data: "Hello, this is working!"'

# Try different accents
roslaunch gtts_speech british.launch    # British accent
roslaunch gtts_speech australian.launch # Australian accent
```

## Available Launch Files

| Launch File | Description | Accent |
|-------------|-------------|---------|
| `basic.launch` | Default US accent | 🇺🇸 American |
| `british.launch` | British accent | 🇬🇧 British |
| `australian.launch` | Australian accent | 🇦🇺 Australian |

## Configuration

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `language` | `en` | Language code |
| `tld` | `com` | Top-level domain (accent) |
| `enable_cache` | `true` | Enable audio caching |
| `cache_dir` | `~/.ros/tts_cache` | Cache directory |
| `max_cache_size` | `100` | Maximum cached files |
| `preload_common_phrases` | `true` | Preload common phrases |

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/text_to_speech` | `std_msgs/String` | Text to be converted to speech |

## Performance

- **First time**: 2-3 seconds (network + generation)
- **Cached phrases**: Instant playback
- **Common phrases**: Pre-cached for immediate response

## Examples

### Basic Usage
```bash
# Simple message
rostopic pub /text_to_speech std_msgs/String 'data: "Robot ready for operation"'

# Navigation feedback
rostopic pub /text_to_speech std_msgs/String 'data: "Destination reached"'

# Status updates
rostopic pub /text_to_speech std_msgs/String 'data: "Battery level low"'
```

### Integration with Other Nodes

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class MyRobotNode:
    def __init__(self):
        self.tts_pub = rospy.Publisher('/text_to_speech', String, queue_size=10)
    
    def announce_status(self, message):
        msg = String()
        msg.data = message
        self.tts_pub.publish(msg)

# Usage
robot = MyRobotNode()
robot.announce_status("Navigation complete")
```

## Troubleshooting

### Common Issues

1. **No audio output**
   ```bash
   # Check if pygame can access audio
   python3 -c "import pygame; pygame.mixer.init(); print('Audio OK')"
   ```

2. **Network errors**
   - Ensure internet connection is available
   - Check firewall settings

3. **Permission errors**
   ```bash
   # Make scripts executable
   chmod +x scripts/*.py
   ```

4. **Cache issues**
   ```bash
   # Clear cache if corrupted
   rm -rf ~/.ros/tts_cache
   ```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Dependencies

- `rospy` - ROS Python library
- `std_msgs` - ROS standard messages
- `gtts` - Google Text-to-Speech
- `pygame` - Audio playback

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Google Text-to-Speech API for free TTS service
- ROS community for the excellent robotics framework
- Pygame community for audio playback capabilities

## Support

If you find this package useful, please ⭐ star this repository!

For issues and questions, please open an [issue](https://github.com/YOUR_USERNAME/gtts_speech/issues).
