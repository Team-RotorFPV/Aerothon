"""Operator-supplied marker colours, as HSV acceptance bands.

The organisers give each banner's colour as a HEX code before the flight, and
say the banners are plain solid colours. A printed banner is still never
exactly that colour in frame: it fades and gathers dust (`world_spec`'s
`wear` takes a printed colour 12% toward grey per level), the camera's white
balance moves it, and shade moves its value. So a supplied colour is a
NOMINAL hue matched with a tolerance, never an equality.

WHY HUE CARRIES THE IDENTITY

    Saturation and value are what wear, shade and exposure move most, so
    pinning them to the supplied colour would reject the real banner on a
    cloudy afternoon. Hue is what separates green from orange, and it is the
    one component those effects mostly leave alone. The bands are therefore
    tight in hue and deliberately wide in saturation and value, with floors
    that keep grey ground and black shadow out.

A hue window near red wraps past 0/180, so a colour yields a LIST of ranges
rather than one. Callers union them. That is also what the red zone needs,
and it replaces the two hand-written red bands that used to be hard-coded.
"""

import numpy as np

# The green band this replaces was hue 35-85 in OpenCV units, i.e. the
# simulated banner's hue (71) plus or minus 25. Keeping that width means a
# supplied colour equal to the rendered one accepts exactly what it used to.
DEFAULT_HUE_TOLERANCE = 25
DEFAULT_S_FLOOR = 90          # below this it is grey, not a printed colour
DEFAULT_V_FLOOR = 60          # below this it is shadow


def parse_hex(text):
    """'#RRGGBB' (or 'RRGGBB') -> (r, g, b), 0-255. Raises ValueError."""
    if not isinstance(text, str):
        raise ValueError('a colour must be a #RRGGBB string')
    value = text.strip().lstrip('#')
    if len(value) != 6 or any(c not in '0123456789abcdefABCDEF' for c in value):
        raise ValueError(f'{text!r} is not a #RRGGBB colour')
    return tuple(int(value[i:i+2], 16) for i in (0, 2, 4))


def hsv_of(text):
    """The supplied colour in OpenCV HSV units: h 0-179, s/v 0-255."""
    import cv2
    r, g, b = parse_hex(text)
    pixel = np.array([[[b, g, r]]], dtype=np.uint8)
    h, s, v = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)[0][0]
    return int(h), int(s), int(v)


def ranges(text, hue_tolerance=DEFAULT_HUE_TOLERANCE,
           s_floor=DEFAULT_S_FLOOR, v_floor=DEFAULT_V_FLOOR):
    """Acceptance ranges for a supplied colour, as [(lo, hi), ...].

    Two ranges when the hue window wraps past 0/180, which is what any red
    needs; one otherwise. Each bound is an OpenCV HSV triple ready for
    `cv2.inRange`.
    """
    hue, _, _ = hsv_of(text)
    tolerance = int(max(1, min(90, hue_tolerance)))
    s_floor = int(max(0, min(255, s_floor)))
    v_floor = int(max(0, min(255, v_floor)))
    lo, hi = hue-tolerance, hue+tolerance
    windows = [(lo, hi)]
    if lo < 0:
        windows = [(0, hi), (180+lo, 179)]
    elif hi > 179:
        windows = [(lo, 179), (0, hi-180)]
    return [(np.array([a, s_floor, v_floor], dtype=np.uint8),
             np.array([b, 255, 255], dtype=np.uint8)) for a, b in windows]


def hue_gap(a, b):
    """Hue distance between two supplied colours, OpenCV units, around the circle."""
    d = abs(hsv_of(a)[0] - hsv_of(b)[0])
    return min(d, 180 - d)


def separated_ranges(name, supplied, hue_tolerance=DEFAULT_HUE_TOLERANCE,
                     s_floor=DEFAULT_S_FLOOR, v_floor=DEFAULT_V_FLOOR):
    """Ranges for `supplied[name]` that reach no other supplied colour.

    The organisers' red (#C4161C, hue 179) and orange (#EB5A0A, hue 11) are
    12 hue steps apart round the circle. At the default +-25 each took the
    other: in the rendered arena the red-zone detector saw the return lane's
    orange floor as a restricted zone at the return mouth, and refused the
    gate crossing there. So each window stops short of HALF the gap to its
    nearest neighbour; colours far apart (green from either) keep the full
    tolerance and behave exactly as before.
    """
    tolerance = hue_tolerance
    for other, value in supplied.items():
        if other != name and value:
            tolerance = min(tolerance, max(1, (hue_gap(supplied[name], value) - 1) // 2))
    return ranges(supplied[name], tolerance, s_floor, v_floor)


def mask(hsv_image, colour_ranges):
    """Union of `colour_ranges` over an HSV image."""
    import cv2
    out = None
    for lo, hi in colour_ranges:
        band = cv2.inRange(hsv_image, lo, hi)
        out = band if out is None else cv2.bitwise_or(out, band)
    if out is None:
        return np.zeros(hsv_image.shape[:2], np.uint8)
    return out


def validate(supplied):
    """Normalise an operator colour mapping, or raise ValueError.

    Accepts a mapping of name -> '#RRGGBB'. Unknown names are refused rather
    than ignored: a colour the aircraft silently drops is worse than one it
    rejects out loud, because the operator believes it was taken.
    """
    known = ('green', 'orange', 'red')
    if not isinstance(supplied, dict) or not supplied:
        raise ValueError('banner colours must be a non-empty mapping')
    unknown = sorted(set(supplied) - set(known))
    if unknown:
        raise ValueError(f'unknown banner colour name(s): {", ".join(unknown)}')
    out = {}
    for name, value in supplied.items():
        r, g, b = parse_hex(value)
        out[name] = '#%02X%02X%02X' % (r, g, b)
    return out
