#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import threading
import queue
import os
import tempfile
import hashlib
from gtts import gTTS
import pygame
import time

class OptimizedFreeTTSNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gtts_node', anonymous=True)
        
        # Initialize pygame mixer for audio playback
        pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
        
        # Cache settings for speed optimization
        self.enable_cache = rospy.get_param('~enable_cache', True)
        self.cache_dir = rospy.get_param('~cache_dir', '/tmp/tts_cache')
        self.max_cache_size = rospy.get_param('~max_cache_size', 100)  # Max cached files
        
        # Create cache directory
        if self.enable_cache:
            os.makedirs(self.cache_dir, exist_ok=True)
        
        # TTS settings optimized for speed and quality
        self.language = rospy.get_param('~language', 'en')
        self.tld = rospy.get_param('~tld', 'com')  # com=US, co.uk=British, com.au=Australian
        self.slow_speech = rospy.get_param('~slow_speech', False)
        
        # Speed optimization settings
        self.use_threading = rospy.get_param('~use_threading', True)
        self.preload_common_phrases = rospy.get_param('~preload_common_phrases', True)
        
        # Create a queue for text messages
        self.text_queue = queue.Queue()
        
        # Cache for quick access to frequently used phrases
        self.audio_cache = {}
        
        # Preload common phrases for instant responses
        if self.preload_common_phrases:
            threading.Thread(target=self.preload_phrases, daemon=True).start()
        
        # Start TTS worker thread
        self.tts_thread = threading.Thread(target=self.tts_worker, daemon=True)
        self.tts_thread.start()
        
        # Subscribe to text messages
        self.text_subscriber = rospy.Subscriber('/text_to_speech', String, self.text_callback)
        
        rospy.loginfo("GTTS node started (Language: {}, Accent: {})".format(self.language, self.tld))
        rospy.loginfo("Cache {} - Directory: {}".format('enabled' if self.enable_cache else 'disabled', self.cache_dir if self.enable_cache else 'N/A'))
    
    def preload_phrases(self):
        """Preload common phrases for instant responses"""
        common_phrases = [
            "Hello",
            "Ready",
            "Complete",
            "Error",
            "Starting",
            "Finished",
            "Yes",
            "No",
            "Okay",
            "Please wait"
        ]
        
        rospy.loginfo("Preloading common phrases...")
        for phrase in common_phrases:
            try:
                self.generate_and_cache_audio(phrase)
                time.sleep(0.1)  # Small delay to not overwhelm the API
            except Exception as e:
                rospy.logwarn(f"Failed to preload phrase '{phrase}': {e}")
        
        rospy.loginfo("Common phrases preloaded for instant responses")
    
    def get_cache_filename(self, text):
        """Generate cache filename based on text and settings"""
        # Create hash of text + settings for unique filename
        content = f"{text}_{self.language}_{self.tld}_{self.slow_speech}"
        hash_obj = hashlib.md5(content.encode())
        return os.path.join(self.cache_dir, f"tts_{hash_obj.hexdigest()}.mp3")
    
    def cleanup_cache(self):
        """Remove old cache files if cache size exceeds limit"""
        if not self.enable_cache:
            return
        
        try:
            cache_files = [f for f in os.listdir(self.cache_dir) if f.startswith('tts_')]
            if len(cache_files) > self.max_cache_size:
                # Sort by modification time and remove oldest
                cache_files.sort(key=lambda x: os.path.getmtime(os.path.join(self.cache_dir, x)))
                files_to_remove = cache_files[:-self.max_cache_size]
                
                for file_to_remove in files_to_remove:
                    os.remove(os.path.join(self.cache_dir, file_to_remove))
                
                rospy.loginfo(f"Cleaned up {len(files_to_remove)} old cache files")
        except Exception as e:
            rospy.logwarn(f"Cache cleanup error: {e}")
    
    def generate_and_cache_audio(self, text):
        """Generate TTS audio and cache it if enabled"""
        cache_file = self.get_cache_filename(text) if self.enable_cache else None
        
        # Check if cached version exists
        if cache_file and os.path.exists(cache_file):
            return cache_file
        
        try:
            # Generate TTS
            tts = gTTS(text=text, lang=self.language, tld=self.tld, slow=self.slow_speech)
            
            if cache_file:
                # Save to cache
                tts.save(cache_file)
                return cache_file
            else:
                # Save to temporary file
                temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.mp3')
                tts.save(temp_file.name)
                return temp_file.name
                
        except Exception as e:
            rospy.logerr(f"Error generating TTS for '{text}': {e}")
            return None
    
    def play_audio_file(self, audio_file, is_temp=False):
        """Play audio file using pygame"""
        try:
            pygame.mixer.music.load(audio_file)
            pygame.mixer.music.play()
            
            # Wait for playback to finish
            while pygame.mixer.music.get_busy():
                pygame.time.wait(50)  # Smaller wait time for better responsiveness
            
            # Clean up temporary file
            if is_temp and os.path.exists(audio_file):
                os.unlink(audio_file)
                
        except Exception as e:
            rospy.logerr(f"Error playing audio: {e}")
    
    def text_callback(self, msg):
        """Callback function for received text messages"""
        text = msg.data.strip()
        if text:
            rospy.loginfo(f"Received text: {text}")
            self.text_queue.put(text)
        else:
            rospy.logwarn("Received empty text message")
    
    def tts_worker(self):
        """Worker thread to handle text-to-speech conversion"""
        while not rospy.is_shutdown():
            try:
                text = self.text_queue.get(timeout=1.0)
                
                start_time = time.time()
                rospy.loginfo(f"Processing TTS: {text}")
                
                # Generate or retrieve cached audio
                audio_file = self.generate_and_cache_audio(text)
                
                if audio_file:
                    is_temp = not (self.enable_cache and os.path.exists(self.get_cache_filename(text)))
                    generation_time = time.time() - start_time
                    
                    # Play the audio
                    play_start = time.time()
                    self.play_audio_file(audio_file, is_temp)
                    total_time = time.time() - start_time
                    
                    rospy.loginfo(f"TTS completed in {total_time:.2f}s (generation: {generation_time:.2f}s)")
                    
                    # Periodic cache cleanup
                    if self.enable_cache and len(os.listdir(self.cache_dir)) > self.max_cache_size:
                        self.cleanup_cache()
                else:
                    rospy.logerr(f"Failed to generate audio for: {text}")
                
                self.text_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in TTS worker: {e}")
    
    def run(self):
        """Main loop to keep the node running"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down Optimized Free TTS node")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        try:
            pygame.mixer.quit()
        except:
            pass

if __name__ == '__main__':
    try:
        tts_node = OptimizedFreeTTSNode()
        tts_node.run()
    except rospy.ROSInitException:
        rospy.logerr("Failed to initialize ROS node")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")