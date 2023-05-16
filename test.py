from threading import Thread
from queue import Queue
from pynput import keyboard

class KeyboardListener(Thread):
    def __init__(self):
        super().__init__()
        self.keys_pressed = set()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def on_press(self, key):
        try:
            self.keys_pressed.add(key)
        except:
            pass

    def on_release(self, key):
        try:
            self.keys_pressed.remove(key)
        except:
            pass

if __name__ == '__main__':
    keyboard_listener = KeyboardListener()
    keyboard_listener.start()

    while True:
        if keyboard.Key.space in keyboard_listener.keys_pressed:
