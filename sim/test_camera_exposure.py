"""The flight camera's manual exposure actually reaches the device.

usb_cam 0.8 runs `v4l2-ctl -c exposure_auto=1 -c exposure=N`; current UVC
drivers name these `auto_exposure` / `exposure_time_absolute`, so the camera
stayed on auto with a dynamic frame rate (Brio 100 on the Pi, 16 Oct 2026).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "src", "aerothon_mission", "mission_bringup"))

from mission_bringup.camera_exposure import controls, wanted  # noqa: E402

# `v4l2-ctl -l` from the fitted Brio 100.
BRIO = """
User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=5 max=2500 step=1 default=156 value=156 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=1
"""
# An older kernel's naming (what usb_cam 0.8 assumes).
LEGACY = """
                  exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=3
              exposure_absolute 0x009a0902 (int)    : min=3 max=2047 step=1 default=250 value=250
"""


def test_current_uvc_names_get_manual_exposure_and_a_fixed_frame_rate():
    present = controls(BRIO)
    assert present["auto_exposure"] == 3 and present["exposure_time_absolute"] == 156
    assert wanted(present, 100) == {"auto_exposure": 1, "exposure_time_absolute": 100,
                                    "exposure_dynamic_framerate": 0}


def test_legacy_names_are_still_served():
    assert wanted(controls(LEGACY), 100) == {"exposure_auto": 1, "exposure_absolute": 100}


def test_a_device_without_exposure_controls_is_left_alone():
    assert wanted(controls("brightness 0x00980900 (int) : min=0 max=1 value=0"), 100) is None


def test_auto_is_the_default_and_restores_aperture_priority():
    """16 Oct: a fixed 10 ms (tuned for daylight) gave a near-black indoor
    feed with gain pinned at 0. Auto is the default; manual is opt-in."""
    assert wanted(controls(BRIO), 0) == {"auto_exposure": 3}
    assert wanted(controls(LEGACY), -1) == {"exposure_auto": 3}
