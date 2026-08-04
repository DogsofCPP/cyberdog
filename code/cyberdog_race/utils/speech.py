"""
Speech synthesis using espeak (apt-installed) for voice announcements.
Provides Chinese text-to-speech for segment object recognition feedback.
"""
import subprocess
import threading
import os


class VoiceSpeaker:
    """Thread-safe espeak wrapper for Chinese speech synthesis.

    Usage:
        speaker = VoiceSpeaker()
        speaker.speak("识别到可乐瓶")
        speaker.speak("识别到橙色小球")
    """

    def __init__(self, rate=120, pitch=50, voice='zh'):
        self.rate = rate
        self.pitch = pitch
        self.voice = voice
        self._queue = []
        self._thread = None
        self._running = True
        self._lock = threading.Lock()

    def speak(self, text, blocking=False):
        """Speak text. If blocking=True, waits for completion."""
        if not text:
            return
        if blocking:
            self._speak_now(text)
        else:
            t = threading.Thread(target=self._speak_now, args=(text,), daemon=True)
            t.start()

    def _speak_now(self, text):
        cmd = [
            'espeak',
            '-v', f'{self.voice}',
            '-s', str(self.rate),
            '-p', str(self.pitch),
            text,
        ]
        try:
            subprocess.run(cmd, capture_output=True, timeout=5.0)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

    def announce(self, object_type):
        """Announce the recognized object type.

        Maps object type to exact competition-required phrase.
        """
        announcement_map = {
            'coke': '识别到可乐瓶',
            'orange_ball': '识别到橙色小球',
            'football': '识别到足球',
            'height_bar': '识别到限高杆',
            'obstacle': '识别到无法跨越障碍',
        }
        phrase = announcement_map.get(object_type, f'识别到{object_type}')
        self.speak(phrase)

    def stop(self):
        self._running = False

    def __del__(self):
        self.stop()
