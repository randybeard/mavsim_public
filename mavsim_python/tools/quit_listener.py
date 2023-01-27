from pynput import keyboard

class QuitListener():
    def __init__(self):
        def on_press(key):
            if key == keyboard.Key.esc:
                return False
            else:
                print("Press 'Esc' to exit...")
        self._listener = keyboard.Listener(on_press=on_press)
        self._listener.start() 

    def check_quit(self):
        if self._listener.running:
            return False
        else:
            return True
        
    def stop_listener(self):
        self._listener.stop()