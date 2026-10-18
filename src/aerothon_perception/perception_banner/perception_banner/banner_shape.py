"""How long and thin a banner may look, from the banner's own size.

The aspect gate's limits were measured on the simulator's 3.7 x 1.15 m board:
0.8-8 upright, and up to 16 for a sheet laid on the ground and seen
foreshortened. The organisers give the banner's width (the corridor's, 3.5 m)
but not its height, and a banner hung from the rod at the 10 ft wall top under
an 8 ft navigation height may be a foot tall. A 3.5 x 0.3 m banner is 11.7:1
seen face-on -- refused by the upright limit of 8 on every frame -- and about
20:1 laid and seen from the 35 deg GROUND pose.

So the upper limits stretch with the banner's own aspect, never below the
measured ones; the lower limits are about what is NOT a face-on banner (seen
end-on, or a post) and stay put. Pure arithmetic, shared by the detector and
the headless world so the two cannot disagree.
"""

MEASURED_BOARD_ASPECT = 3.7 / 1.15
UPRIGHT_MAX = 8.0
GROUND_MAX = 16.0


def stretch(width_m, height_m):
    """How much thinner than the measured board this banner is (>= 1)."""
    try:
        w, h = float(width_m), float(height_m)
    except (TypeError, ValueError):
        return 1.0
    if not (w > 0.0 and h > 0.0):
        return 1.0
    return max(1.0, (w / h) / MEASURED_BOARD_ASPECT)


def max_aspect(width_m, height_m, ground=False, upright_max=UPRIGHT_MAX,
               ground_max=GROUND_MAX):
    """Largest box aspect still accepted as this banner."""
    k = stretch(width_m, height_m)
    return (ground_max if ground else upright_max) * k
