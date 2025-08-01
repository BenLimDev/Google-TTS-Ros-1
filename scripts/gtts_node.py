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
import yaml
import pyttsx3
from pydub import AudioSegment

class OptimizedFreeTTSNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gtts_node', anonymous=True)
        
        # Initialize pygame mixer for audio playback
        pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
        
        # Cache settings
        self.enable_cache = rospy.get_param('~enable_cache', True)
        self.cache_dir = rospy.get_param('~cache_dir', '/tmp/tts_cache')
        self.max_cache_size = rospy.get_param('~max_cache_size', 100)
        
        # Create cache directory
        if self.enable_cache:
            os.makedirs(self.cache_dir, exist_ok=True)
        
        # TTS settings
        self.language = rospy.get_param('~language', 'en')
        self.tld = rospy.get_param('~tld', 'ca')
        self.slow_speech = rospy.get_param('~slow_speech', False)
        
        # Offline TTS settings
        self.use_offline_fallback = rospy.get_param('~use_offline_fallback', True)
        self.offline_voice_rate = rospy.get_param('~offline_voice_rate', 50)  # reduced from 200 for slower speech
        self.offline_voice_volume = rospy.get_param('~offline_voice_volume', 0.9)
        
        # Generation timeout settings
        self.gtts_timeout = rospy.get_param('~gtts_timeout', 3.0)  # seconds - reduced for faster fallback
        self.offline_timeout = rospy.get_param('~offline_timeout', 15.0)  # seconds
        self.prefer_online = rospy.get_param('~prefer_online', True)  # prefer gTTS if both ready
        
        # Common phrases file path
        self.common_phrases_file = rospy.get_param('~common_phrases_file', 
                                                 os.path.join(os.path.dirname(__file__), 'common_phrases.yaml'))
        
        # Audio cache - stores ready-to-play audio files
        self.audio_cache = {}  # text -> audio_file_path
        self.cache_lock = threading.Lock()
        
        # Queues for processing
        self.text_input_queue = queue.Queue()  # Input texts to process
        self.playback_queue = queue.Queue()    # Ready audio files to play
        
        # Test offline TTS availability
        self.offline_available = self.test_offline_tts()
        
        # Start worker threads
        self.start_worker_threads()
        
        # Preload common phrases
        self.preload_phrases()
        
        # Subscribe to text messages
        self.text_subscriber = rospy.Subscriber('/text_to_speech', String, self.text_callback)
        
        rospy.loginfo("GTTS node started (Language: {}, Accent: {})".format(self.language, self.tld))
        rospy.loginfo("Cache {} - Directory: {}".format('enabled' if self.enable_cache else 'disabled', self.cache_dir if self.enable_cache else 'N/A'))
        rospy.loginfo("Offline TTS: {} | Prefer online: {}".format(
            'Available' if self.offline_available else 'Not available', 
            'Yes' if self.prefer_online else 'No'))
        rospy.loginfo("Generation strategy: Both start simultaneously, prefer gTTS within 3s, fallback to offline")
        rospy.loginfo("Generation timeouts: gTTS={}s, Offline={}s".format(self.gtts_timeout, self.offline_timeout))
        rospy.loginfo("Audio format: WAV (standardized)")
    
    def test_offline_tts(self):
        """Test if offline TTS is available"""
        try:
            test_tts = pyttsx3.init()
            test_tts.stop()
            rospy.loginfo("Offline TTS engine available")
            return True
        except Exception as e:
            rospy.logerr(f"Offline TTS not available: {e}")
            return False
    
    def start_worker_threads(self):
        """Start all worker threads"""
        # Audio generation thread
        self.generation_thread = threading.Thread(target=self.audio_generator, daemon=True)
        self.generation_thread.start()
        
        # Audio playback thread
        self.playback_thread = threading.Thread(target=self.audio_player, daemon=True)
        self.playback_thread.start()
    
    def load_common_phrases(self):
        """Load common phrases from YAML file"""
        try:
            if os.path.exists(self.common_phrases_file):
                with open(self.common_phrases_file, 'r') as file:
                    data = yaml.safe_load(file)
                    phrases = data.get('common_phrases', [])
                    rospy.loginfo(f"Loaded {len(phrases)} common phrases from {self.common_phrases_file}")
                    return phrases
            else:
                rospy.logwarn(f"Common phrases file not found: {self.common_phrases_file}")
                return []
        except Exception as e:
            rospy.logerr(f"Error loading common phrases: {e}")
            return []
    
    def preload_phrases(self):
        """Preload common phrases"""
        common_phrases = self.load_common_phrases()
        if not common_phrases:
            return
        
        rospy.loginfo("Preloading common phrases...")
        for phrase in common_phrases:
            self.text_input_queue.put(phrase)
    
    def text_callback(self, msg):
        """Callback for received text messages - non-blocking"""
        text = msg.data.strip()
        if text:
            rospy.loginfo(f"Received text: {text}")
            
            # Always add to generation queue immediately - never block
            self.text_input_queue.put(text)
            
        else:
            rospy.logwarn("Received empty text message")
    
    def get_cache_filename(self, text, is_offline=False):
        """Generate cache filename - all files are WAV"""
        suffix = "_offline" if is_offline else "_online"
        content = f"{text}_{self.language}_{self.tld}_{self.slow_speech}{suffix}"
        hash_obj = hashlib.md5(content.encode())
        return os.path.join(self.cache_dir, f"tts_{hash_obj.hexdigest()}.wav")
    
    def generate_gtts_audio_thread(self, text, result_dict, result_event):
        """Generate gTTS audio and convert to WAV"""
        try:
            cache_file = self.get_cache_filename(text, False) if self.enable_cache else None
            
            # Check cache first
            if cache_file and os.path.exists(cache_file):
                result_dict['gtts'] = (cache_file, False, "cached")
                result_event.set()
                return
            
            # Generate gTTS audio - add timeout and error handling for offline scenarios
            try:
                tts = gTTS(text=text, lang=self.language, tld=self.tld, slow=self.slow_speech)
                
                # Save to temporary MP3 first
                temp_mp3 = tempfile.NamedTemporaryFile(delete=False, suffix='.mp3')
                tts.save(temp_mp3.name)
                
                # Convert MP3 to WAV using pydub
                audio = AudioSegment.from_mp3(temp_mp3.name)
                
                if cache_file:
                    audio.export(cache_file, format="wav")
                    final_file = cache_file
                    is_temp = False
                else:
                    temp_wav = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
                    audio.export(temp_wav.name, format="wav")
                    final_file = temp_wav.name
                    is_temp = True
                
                # Clean up temporary MP3
                os.unlink(temp_mp3.name)
                
                result_dict['gtts'] = (final_file, is_temp, "generated")
                
            except Exception as network_error:
                # Handle network/connectivity issues gracefully
                rospy.logwarn(f"gTTS network error for '{text}': {network_error}")
                result_dict['gtts'] = (None, False, "network_failed")
            
            result_event.set()
            
        except Exception as e:
            rospy.logwarn(f"gTTS generation failed for '{text}': {e}")
            result_dict['gtts'] = (None, False, "failed")
            result_event.set()
    
    def generate_offline_audio_thread(self, text, result_dict, result_event):
        """Generate offline TTS audio in WAV format"""
        try:
            cache_file = self.get_cache_filename(text, True) if self.enable_cache else None
            
            # Check cache first
            if cache_file and os.path.exists(cache_file):
                result_dict['offline'] = (cache_file, False, "cached")
                result_event.set()
                return
            
            # Generate new audio
            if cache_file:
                output_file = cache_file
                is_temp = False
            else:
                temp_file = tempfile.NamedTemporaryFile(delete=False, suffix='.wav')
                temp_file.close()
                output_file = temp_file.name
                is_temp = True
            
            # Create fresh TTS instance for this thread
            local_tts = pyttsx3.init()
            local_tts.setProperty('rate', self.offline_voice_rate)
            local_tts.setProperty('volume', self.offline_voice_volume)
            
            local_tts.save_to_file(text, output_file)
            local_tts.runAndWait()
            local_tts.stop()
            
            result_dict['offline'] = (output_file, is_temp, "generated")
            result_event.set()
            
        except Exception as e:
            rospy.logwarn(f"Offline TTS generation failed for '{text}': {e}")
            result_dict['offline'] = (None, False, "failed")
            result_event.set()
    
    def generate_dual_audio(self, text):
        """Generate both online and offline audio simultaneously, play offline if gTTS takes too long"""
        result_dict = {}
        gtts_event = threading.Event()
        offline_event = threading.Event()
        
        start_time = time.time()
        
        # Start both generations simultaneously
        gtts_thread = threading.Thread(
            target=self.generate_gtts_audio_thread, 
            args=(text, result_dict, gtts_event), 
            daemon=True
        )
        
        offline_thread = None
        if self.offline_available and self.use_offline_fallback:
            offline_thread = threading.Thread(
                target=self.generate_offline_audio_thread, 
                args=(text, result_dict, offline_event), 
                daemon=True
            )
        
        # Start both threads
        gtts_thread.start()
        if offline_thread:
            offline_thread.start()
        
        # Wait 3 seconds for gTTS
        gtts_ready = gtts_event.wait(timeout=3.0)
        
        if gtts_ready:
            # gTTS completed within 3 seconds, check if successful
            gtts_result = result_dict.get('gtts', (None, False, "failed"))
            gtts_file, gtts_temp, gtts_status = gtts_result
            
            if gtts_file and gtts_status in ["cached", "generated"]:
                generation_time = time.time() - start_time
                rospy.loginfo(f"Using gTTS audio ({gtts_status}) for '{text}' - completed in {generation_time:.2f}s")
                return gtts_file, gtts_temp, "gTTS"
        
        # gTTS not ready after 3 seconds, check if offline is ready
        rospy.loginfo(f"gTTS not ready after 3 seconds, checking offline TTS...")
        
        if offline_thread:
            # Wait for offline to complete (if not already done)
            offline_ready = offline_event.wait(timeout=self.offline_timeout)
            
            if offline_ready:
                offline_result = result_dict.get('offline', (None, False, "failed"))
                offline_file, offline_temp, offline_status = offline_result
                
                if offline_file and offline_status in ["cached", "generated"]:
                    generation_time = time.time() - start_time
                    rospy.loginfo(f"Using offline audio ({offline_status}) for '{text}' - completed in {generation_time:.2f}s")
                    return offline_file, offline_temp, "offline"
        
        # If we get here, both methods failed or offline wasn't available
        # Wait a bit longer for gTTS as last resort
        if not gtts_ready and gtts_thread.is_alive():
            rospy.loginfo("Waiting for gTTS as last resort...")
            if gtts_event.wait(timeout=self.gtts_timeout - 3.0):  # Remaining time
                gtts_result = result_dict.get('gtts', (None, False, "failed"))
                gtts_file, gtts_temp, gtts_status = gtts_result
                
                if gtts_file and gtts_status in ["cached", "generated"]:
                    generation_time = time.time() - start_time
                    rospy.loginfo(f"Using delayed gTTS audio ({gtts_status}) for '{text}' - completed in {generation_time:.2f}s")
                    return gtts_file, gtts_temp, "gTTS"
        
        generation_time = time.time() - start_time
        rospy.logerr(f"All TTS methods failed for: {text} (total time: {generation_time:.2f}s)")
        return None, False, "none"
    
    def choose_best_audio(self, result_dict, completed_methods):
        """Choose the best available audio file"""
        # Priority: 
        # 1. Cached online (fastest, best quality)
        # 2. Cached offline 
        # 3. Generated online (preferred quality)
        # 4. Generated offline (fallback)
        
        gtts_result = result_dict.get('gtts', (None, False, "failed"))
        offline_result = result_dict.get('offline', (None, False, "failed"))
        
        gtts_file, gtts_temp, gtts_status = gtts_result
        offline_file, offline_temp, offline_status = offline_result
        
        rospy.loginfo(f"Audio generation results - gTTS: {gtts_status}, Offline: {offline_status}")
        
        # Cached online first (WAV)
        if gtts_file and gtts_status == "cached":
            rospy.loginfo("Selected cached gTTS audio (WAV)")
            return gtts_file, gtts_temp, "gTTS", "cached"
        
        # Cached offline second (WAV)
        if offline_file and offline_status == "cached":
            rospy.loginfo("Selected cached offline audio (WAV)")
            return offline_file, offline_temp, "offline", "cached"
        
        # If preferring online and both generated successfully
        if self.prefer_online and gtts_file and gtts_status == "generated":
            rospy.loginfo("Selected generated gTTS audio (WAV) - preferred")
            return gtts_file, gtts_temp, "gTTS", "generated"
        
        # Any successful generation - prefer online for quality
        if gtts_file and gtts_status == "generated":
            rospy.loginfo("Selected generated gTTS audio (WAV)")
            return gtts_file, gtts_temp, "gTTS", "generated"
        
        if offline_file and offline_status == "generated":
            rospy.loginfo("Selected generated offline audio (WAV)")
            return offline_file, offline_temp, "offline", "generated"
        
        rospy.logerr("No valid audio files generated")
        return None
    
    def audio_generator(self):
        """Worker thread to generate audio files"""
        while not rospy.is_shutdown():
            try:
                text = self.text_input_queue.get(timeout=1.0)
                
                # Check if already cached (moved from callback to here)
                with self.cache_lock:
                    if text in self.audio_cache:
                        rospy.loginfo(f"Audio already cached for: {text}")
                        self.playback_queue.put((text, self.audio_cache[text], False))
                        self.text_input_queue.task_done()
                        continue
                
                rospy.loginfo(f"Generating audio for: {text}")
                
                # Generate audio using dual approach
                audio_file, is_temp, method = self.generate_dual_audio(text)
                
                if audio_file:
                    # Cache the audio file path
                    with self.cache_lock:
                        self.audio_cache[text] = audio_file
                    
                    # Queue for playback
                    self.playback_queue.put((text, audio_file, is_temp))
                else:
                    rospy.logerr(f"Failed to generate audio for: {text}")
                
                self.text_input_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in audio generator: {e}")
                try:
                    self.text_input_queue.task_done()
                except:
                    pass
    
    def audio_player(self):
        """Worker thread to play audio files"""
        while not rospy.is_shutdown():
            try:
                text, audio_file, is_temp = self.playback_queue.get(timeout=1.0)
                
                start_time = time.time()
                rospy.loginfo(f"Playing audio: {text}")
                
                # Check file format and handle accordingly
                if not os.path.exists(audio_file):
                    rospy.logerr(f"Audio file not found: {audio_file}")
                    self.playback_queue.task_done()
                    continue
                
                rospy.loginfo(f"Playing WAV file: {os.path.basename(audio_file)}")
                
                try:
                    # Use pygame.mixer.Sound for WAV files (more reliable)
                    sound = pygame.mixer.Sound(audio_file)
                    sound.play()
                    
                    # Wait for sound to finish
                    sound_length = sound.get_length()
                    time.sleep(sound_length)
                    
                    play_time = time.time() - start_time
                    rospy.loginfo(f"Playback completed in {play_time:.2f}s: {text}")
                    
                except pygame.error as e:
                    rospy.logerr(f"Pygame audio error: {e}")
                    rospy.logerr(f"File path: {audio_file}")
                    
                    # Fallback to pygame.mixer.music
                    try:
                        rospy.loginfo("Attempting fallback playback method...")
                        pygame.mixer.music.load(audio_file)
                        pygame.mixer.music.play()
                        
                        # Wait for playback to finish
                        while pygame.mixer.music.get_busy():
                            pygame.time.wait(50)
                        
                        play_time = time.time() - start_time
                        rospy.loginfo(f"Fallback playback completed in {play_time:.2f}s: {text}")
                        
                    except Exception as fallback_e:
                        rospy.logerr(f"Fallback playback also failed: {fallback_e}")
                
                # Clean up temporary file
                if is_temp and os.path.exists(audio_file):
                    os.unlink(audio_file)
                    # Remove from cache if it was temporary
                    with self.cache_lock:
                        if text in self.audio_cache and self.audio_cache[text] == audio_file:
                            del self.audio_cache[text]
                
                self.playback_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in audio player: {e}")
                try:
                    self.playback_queue.task_done()
                except:
                    pass
    
    def run(self):
        """Main loop"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down TTS node")
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