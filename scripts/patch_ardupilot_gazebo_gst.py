#!/usr/bin/env python3
"""Make ardupilot_gazebo's GStreamer dependency optional.

WHY
    ardupilot_gazebo declares:

        pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

    so the whole package fails to configure without libgstreamer1.0-dev and
    libgstreamer-plugins-base1.0-dev. Installing those needs root.

    GStreamer is used by exactly one target, GstCameraPlugin (src/
    GstCameraPlugin.cc), which streams camera video over RTP/UDP. This project
    does not use it: camera frames reach ROS through the standard Gazebo
    camera sensor and the ros_gz bridge, and reach the GCS through
    web_video_server. The plugin that actually matters — ArduPilotPlugin, the
    SITL <-> Gazebo JSON interface — has no GStreamer dependency at all.

    So when GStreamer development files are absent we drop that one optional
    plugin rather than blocking the entire simulation stack on a package
    install the operator may not be able to perform.

PREFERRED ALTERNATIVE
    If you have root, this patch is unnecessary. Install the real thing:

        sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

    then re-run scripts/install_ardupilot_overlay.sh, which will detect
    GStreamer and skip patching.

This script is idempotent and only ever relaxes a requirement.

Usage:  patch_ardupilot_gazebo_gst.py <path/to/ardupilot_gazebo/CMakeLists.txt>
Exit:   0 patched or already patched, 1 unexpected file layout.
"""

import sys
from pathlib import Path

MARKER = "# AEROTHON: GStreamer made optional (see scripts/patch_ardupilot_gazebo_gst.py)"

REQUIRED_LINE = "pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)"
OPTIONAL_LINE = f"{MARKER}\npkg_check_modules(GST gstreamer-1.0 gstreamer-app-1.0)"


def patch(path: Path) -> int:
    text = path.read_text()

    if MARKER in text:
        print(f"[skip] already patched: {path}")
        return 0

    # ---- 1. drop REQUIRED ------------------------------------------------- #
    if REQUIRED_LINE not in text:
        print(f"[ERROR] expected line not found in {path}:\n  {REQUIRED_LINE}",
              file=sys.stderr)
        return 1
    text = text.replace(REQUIRED_LINE, OPTIONAL_LINE, 1)

    # ---- 2. guard the GstCameraPlugin target ------------------------------ #
    start = text.find("add_library(GstCameraPlugin")
    if start == -1:
        print("[ERROR] add_library(GstCameraPlugin ...) not found", file=sys.stderr)
        return 1

    link_start = text.find("target_link_libraries(GstCameraPlugin", start)
    if link_start == -1:
        print("[ERROR] target_link_libraries(GstCameraPlugin ...) not found",
              file=sys.stderr)
        return 1
    block_end = text.find("\n)\n", link_start)
    if block_end == -1:
        print("[ERROR] could not find end of GstCameraPlugin link block",
              file=sys.stderr)
        return 1
    block_end += len("\n)\n")

    block = text[start:block_end]
    guarded = (
        "if(GST_FOUND)\n"
        + "\n".join("  " + line if line.strip() else line
                    for line in block.rstrip("\n").split("\n"))
        + "\nelse()\n"
        "  message(STATUS \"GStreamer not found - skipping GstCameraPlugin "
        "(not used by this project)\")\n"
        "endif()\n"
    )
    text = text[:start] + guarded + text[block_end:]

    # ---- 3. install GstCameraPlugin only when it was built ---------------- #
    install_block_start = text.find("install(\n  TARGETS")
    if install_block_start == -1:
        print("[ERROR] install(TARGETS ...) block not found", file=sys.stderr)
        return 1
    install_block_end = text.find(")", install_block_start) + 1
    install_block = text[install_block_start:install_block_end]

    if "GstCameraPlugin" not in install_block:
        print("[ERROR] GstCameraPlugin not present in install block", file=sys.stderr)
        return 1

    new_install = "\n".join(
        line for line in install_block.split("\n")
        if line.strip() != "GstCameraPlugin"
    )
    new_install += (
        "\n\nif(GST_FOUND)\n"
        "  install(TARGETS GstCameraPlugin DESTINATION lib/${PROJECT_NAME})\n"
        "endif()"
    )
    text = text[:install_block_start] + new_install + text[install_block_end:]

    backup = path.with_suffix(path.suffix + ".aerothon-orig")
    if not backup.exists():
        backup.write_text(path.read_text())
    path.write_text(text)
    print(f"[patched] {path}")
    print(f"[backup ] {backup}")
    return 0


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    path = Path(sys.argv[1])
    if not path.is_file():
        print(f"[ERROR] not a file: {path}", file=sys.stderr)
        return 1
    return patch(path)


if __name__ == "__main__":
    sys.exit(main())
