# Gazebo window recovery, 2026-09-11

The Gazebo window opened as a transparent rectangle showing the desktop
underneath. Its title included `[WARN:COPY MODE]`. WSLg's weston.log reported
`rdp_allocate_shared_memory: Failed to open ... Input/output error`.

`glxinfo -B` also showed llvmpipe with acceleration disabled. Setting
`GALLIUM_DRIVER=d3d12` enabled Intel UHD GPU rendering, but did not repair the
transparent window. Restarting WSL restored the display connection and removed
the COPY MODE warning. The arena then appeared.

The OGRE2 GUI still failed the menu-click check. A GDB trace of its main thread
showed QQuickWindow waiting in QWaitCondition. The basic Qt render-loop trial
produced an empty viewport and was discarded. Switching only the GUI to OGRE
with the threaded Qt loop displayed the arena and passed the menu-click check:
the Save world / configuration / About / Quit menu visibly opened.

The master launcher now defaults to D3D12 and OGRE for the GUI on WSL systems
with /dev/dxg. Server sensor rendering remains unchanged. The GUI engine can
be overridden with AEROTHON_GUI_RENDER_ENGINE. Linux outside WSL retains OGRE2.

The current viewing session runs through .scratch/open_gazebo.sh. Its generated
vehicle also adapts the installed upstream model's nested base_link references
and removes controllers for the removed gimbal. These are viewing-model
adaptations, not flight-qualified changes. The GCS is served on port 8899;
FCU telemetry is still disconnected. No autonomous flight was attempted.

Verification: visible arena, menu opened by mouse input, WSL COPY MODE warning
absent, and shell syntax checks passed. If COPY MODE recurs, restarting WSL
closes its running applications and may be needed again; renderer selection
alone did not recover that display failure.
