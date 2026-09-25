#!/usr/bin/env python3
"""Make ardupilot_sitl's AP_DDS build optional.

WHY
    Tools/ros2/ardupilot_sitl/CMakeLists.txt invokes waf with a hardcoded flag:

        waf configure --board sitl --enable-dds
        waf build --enable-dds

    --enable-dds requires the `microxrceddsgen` code generator. That generator
    is a Gradle/Java project whose bundled Gradle 7.6 does not support the
    Java 21 JDK on this machine, and whose IDL-Parser submodule does not build
    under Gradle 8 (it uses the `classifier` property Gradle 8 removed).
    Installing a JDK 17 alongside would need root.

    This project does not use AP_DDS at all. The locked architecture talks to
    the flight controller over MAVLink via MAVROS (see docs/ARCHITECTURE.md);
    AP_DDS is an alternative ROS 2 transport that nothing here subscribes to.
    Building SITL without it costs this project nothing.

PREFERRED ALTERNATIVE
    If microxrceddsgen is on PATH, this patch is unnecessary — the installer
    detects it and skips patching. To get it properly you need a JDK that the
    upstream Gradle wrapper supports (JDK 17):

        sudo apt install openjdk-17-jdk
        JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64 ./gradlew assemble

Idempotent; only ever removes a build flag.

Usage:  patch_ardupilot_sitl_dds.py <path/to/ardupilot_sitl/CMakeLists.txt>
Exit:   0 patched or already patched, 1 unexpected file layout.
"""

import sys
from pathlib import Path

MARKER = "# AEROTHON: AP_DDS disabled (see scripts/patch_ardupilot_sitl_dds.py)"


def patch(path: Path) -> int:
    text = path.read_text()

    if MARKER in text:
        print(f"[skip] already patched: {path}")
        return 0

    if "--enable-dds" not in text:
        print(f"[ERROR] '--enable-dds' not found in {path}; upstream layout changed",
              file=sys.stderr)
        return 1

    occurrences = text.count("--enable-dds")
    text = text.replace(" --enable-dds", "")
    text = MARKER + "\n" + text

    backup = path.with_suffix(path.suffix + ".aerothon-orig")
    if not backup.exists():
        backup.write_text(path.read_text())
    path.write_text(text)

    print(f"[patched] {path}  ({occurrences} occurrence(s) of --enable-dds removed)")
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
