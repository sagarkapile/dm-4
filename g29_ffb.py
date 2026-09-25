import evdev
from evdev import ecodes
import glob
import threading
import time

def find_wheel_device():
    for dev_path in sorted(glob.glob("/dev/input/event*")):
        try:
            dev = evdev.InputDevice(dev_path)
            if ecodes.EV_FF in dev.capabilities():
                print(f"✅ Found FFB Device: {dev.name} ({dev_path})")
                return dev
        except Exception:
            pass
    return None

class G29FFB:
    # Default spring strength when autocenter is enabled.
    # 22% is far too weak on G29 and feels almost loose/off.
    DEFAULT_AUTOCENTER_PCT = 70.0

    def __init__(self, dev=None):
        self.dev = dev if dev else find_wheel_device()
        if not self.dev:
            raise RuntimeError("❌ No FFB-capable device found on any event node!")
        self._last_autocenter = 0.0
        self._gain_set = False
        self.path = getattr(self.dev, "path", None)
        self.name = getattr(self.dev, "name", None)

        # Full FFB gain first; without this many G29s feel dead/loose.
        self.set_gain(100.0)

    def set_gain(self, gain_pct=100.0):
        """Set overall force-feedback gain (0..100)."""
        try:
            magnitude = int(max(0.0, min(100.0, float(gain_pct))) * 655.35)
            self.dev.write(ecodes.EV_FF, ecodes.FF_GAIN, magnitude)
            self._gain_set = True
            return True
        except Exception as exc:
            print(
                f"[G29FFB] FF_GAIN failed on "
                f"{getattr(self.dev, 'path', '?')}: {exc}"
            )
            return False

    def disable_autocenter(self):
        return self.set_hardware_autocenter(0.0)

    def set_hardware_autocenter(self, strength_pct, _internal=False):
        """Apply Logitech hardware spring autocenter.

        strength_pct: 0..100
        Returns True on success.
        """
        if not _internal:
            self._last_autocenter = float(strength_pct)

        # Ensure gain is up before spring force.
        if not self._gain_set:
            self.set_gain(100.0)

        try:
            magnitude = int(
                max(0.0, min(100.0, float(strength_pct))) * 655.35
            )
            self.dev.write(
                ecodes.EV_FF,
                ecodes.FF_AUTOCENTER,
                magnitude
            )
            return True
        except Exception as exc:
            print(
                f"[G29FFB] FF_AUTOCENTER failed on "
                f"{getattr(self.dev, 'path', '?')}: "
                f"strength={strength_pct} err={exc}"
            )
            return False

    def set_autocenter(self, strength_pct):
        return self.set_hardware_autocenter(strength_pct)

    def apply_enabled(self, enabled, strength_pct=None):
        """Convenience: enable spring at strength, or disable."""
        if strength_pct is None:
            strength_pct = self.DEFAULT_AUTOCENTER_PCT
        target = float(strength_pct) if enabled else 0.0
        ok = self.set_hardware_autocenter(target)
        return ok, target

    def set_force(self, force_val):
        return self.set_hardware_autocenter(abs(force_val) * 100.0)

    def stop(self):
        try:
            self.set_hardware_autocenter(0)
        except Exception:
            pass

    def play_rumble(self, duration_ms=1000, **kwargs):
        def _rumble():
            end = time.time() + (duration_ms / 1000.0)
            while time.time() < end:
                self.set_hardware_autocenter(100.0, _internal=True)
                time.sleep(0.02)
                self.set_hardware_autocenter(0.0, _internal=True)
                time.sleep(0.02)
            self.set_hardware_autocenter(self._last_autocenter, _internal=True)
        threading.Thread(target=_rumble, daemon=True).start()

    def play_terrain(self, duration_ms=1000, **kwargs):
        def _terrain():
            end = time.time() + (duration_ms / 1000.0)
            while time.time() < end:
                self.set_hardware_autocenter(80.0, _internal=True)
                time.sleep(0.05)
                self.set_hardware_autocenter(10.0, _internal=True)
                time.sleep(0.05)
            self.set_hardware_autocenter(self._last_autocenter, _internal=True)
        threading.Thread(target=_terrain, daemon=True).start()

    def play_kick(self, strength, duration_ms=250, **kwargs):
        def _kick():
            self._effect_playing = True
            try:
                end = time.time() + (duration_ms / 1000.0)
                while time.time() < end:
                    self.set_hardware_autocenter(100.0, _internal=True)
                    time.sleep(0.04)
                    self.set_hardware_autocenter(0.0, _internal=True)
                    time.sleep(0.04)
            finally:
                self._effect_playing = False
                self.set_hardware_autocenter(self._last_autocenter, _internal=True)
        threading.Thread(target=_kick, daemon=True).start()
