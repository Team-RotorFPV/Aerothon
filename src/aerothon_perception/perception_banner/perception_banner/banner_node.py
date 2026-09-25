#!/usr/bin/env python3
"""Green banner detection WITH identity validation, and a bearing for alignment.

WHAT WAS WRONG (Phase 4)
    The detector found the largest green blob, checked its area and aspect
    ratio, and published `z=1.0` — "banner". Any green rectangle passed. The
    GCS then displayed ALIGNED, which meant only "a qualifying green blob is in
    frame", not that the aircraft was aligned with anything, and certainly not
    that the thing was the competition gate.

    Grass, a tarpaulin, a green vehicle roof or a hedge would all have been
    accepted, and the corridor entry heading was going to be derived from it.

WHAT IDENTITY MEANS HERE
    The competition banner is a green board carrying white "AEROTHON"
    lettering inside a white frame. That gives two structural signatures a
    plain green object does not have:

      1. WHITE CONTENT INSIDE THE GREEN REGION — the lettering and frame
         occupy a characteristic fraction of the board.
      2. MULTIPLE SEPARATE WHITE COMPONENTS arranged in a horizontal band —
         letters. A blank green tarp has none; a tarp with one white stripe has
         one.

    Neither is OCR, and neither is claimed to be. They are cheap structural
    checks that a blank green object fails, tested against decoys placed in the
    simulated world for exactly this purpose.

    `min_letter_components` is the discriminating parameter. Raising it makes
    the gate stricter; the real corpus (docs/CORPUS_SHOT_LIST.md) is what
    should eventually set it.

BEARING FOR ALIGNMENT
    Publishes the horizontal offset of the banner centre as a normalised
    bearing so the mission can yaw/translate to face it. This is what replaces
    the hardcoded `corridor_entry` waypoint (geometry audit A5).

Topics
  sub  <image_topic>              sensor_msgs/Image
  pub  /percep/banner             geometry_msgs/Vector3
        x,y in [-1,1] from image centre; z=1.0 identified banner, 0.5 green
        object rejected as not-the-banner, 0.0 nothing
  pub  /percep/banner/detail      std_msgs/String   JSON diagnostics
  pub  /percep/banner/annotated   sensor_msgs/Image
"""

import json
import time
import math

import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from perception_banner.word_reader import reads_banner
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import String


class BannerNode(Node):
    def __init__(self):
        super().__init__('perception_banner')
        p = self.declare_parameter
        p('image_topic', '/image_raw')
        p('h_lo', 35); p('h_hi', 85)
        p('s_lo', 90); p('s_hi', 255)
        p('v_lo', 60); p('v_hi', 255)
        # A FIXED image fraction is a hidden statement about range (audit E1):
        # 1% of frame is a banner 30 m away and a scrap of tarp at 3 m. The
        # gate is instead derived from how big the banner PROJECTS at the
        # farthest range worth detecting, given this camera.
        p('min_area_frac', 0.0)        # 0 = derive it; >0 pins it explicitly
        p('banner_width_m', 2.0)       # competition banner, UNCONFIRMED
        p('banner_height_m', 1.0)
        # 25 m was a guess, and it made the floor 1966 px at 1280x720 -- low
        # enough that a 97x49 sliver of banner clipped at the frame edge
        # (2134 px, MEASURED in flight) outranked it, and the sweep aligned to
        # that sliver for twelve consecutive frames. The real banner in the
        # same run measured 139932 px, a factor of 65 larger.
        #
        # The gate has to be identified from the take-off pad before the
        # aircraft descends to corridor altitude, and in every arena the
        # randomiser produces it stands within about 8 m of the start. 12 m is
        # generous for that and puts the floor at 8532 px -- comfortably above
        # every sliver measured and far below the banner.
        p('max_detect_range_m', 12.0)  # beyond this it is not worth aligning to
        p('camera_hfov', 1.0472)
        p('area_safety', 0.5)          # accept half the ideal projected area:
                                       # oblique views and partial occlusion
        # MEASURED on the arena's own gate, in flight. The derived board comes
        # out at aspect 1.10-1.22 across the views the sweep gets, and the old
        # 1.2 floor sat in the MIDDLE of that spread -- so the real banner was
        # accepted on roughly one frame in eight and rejected on the rest,
        # which is exactly the flicker that made a dwell score 8/12 and then
        # centre on whatever else was green:
        #
        #     aspect 1.13 -> REJECTED      aspect 1.10 -> REJECTED (x4)
        #     aspect 1.22 -> identified
        #
        # Sweeping the floor over the captured frames: 1.20 identifies 1 frame
        # of 16, 1.05 identifies 7, and going lower changes nothing at all --
        # including adding no false positives, because the board-area floor
        # and the reading now carry that discrimination. The aspect gate was
        # doing a job it no longer has to do.
        p('min_aspect', 0.9); p('max_aspect', 8.0)
        # ---- identity ----
        p('require_identity', True)
        # Lettering is detected RELATIVE to the board it sits on, not against
        # an absolute brightness. A live frame showed the banner filling most
        # of the image with "AEROTHON" plainly legible and the detector
        # reporting `components: 0, white_frac: 0.0` -- the lettering rendered
        # at about V=150 under flat ambient light, below a fixed V>=170 floor.
        #
        # An absolute threshold is a hidden assumption about illumination, and
        # it is the same assumption that will break on the real photo corpus
        # (overcast, shade, direct sun). What is actually invariant is that the
        # letters are BRIGHTER and much LESS SATURATED than the green board.
        p('white_v_min', 90)           # absolute floor, kept as a backstop
        p('white_s_max', 110)          # absolute ceiling, likewise
        p('white_v_ratio', 1.08)       # ...but mainly: brighter than the board
        p('white_s_ratio', 0.55)       # ...and much less saturated than it
        # Lettering DARKER than the board -- the arena's own gate has grey
        # letters on green. Same saturation test, other side of the board.
        p('dark_v_ratio', 0.85)        # ...or markedly darker than it
        p('dark_v_floor', 40)          # but not shadow or black
        # Lettering must be enclosed by the board. Kernel as a fraction of
        # the region height: fills letter-sized holes, not a gate opening.
        p('board_close_frac', 0.10)
        # Below this surviving fraction the confinement is not describing
        # a board with lettering in it, so it is not applied.
        p('board_confine_min', 0.15)
        # Width/height bounds for something that could be a character.
        p('letter_min_aspect', 0.15)
        p('letter_max_aspect', 1.60)
        p('min_white_frac', 0.02)      # white content inside the green region
        p('max_white_frac', 0.60)      # a mostly-white board is not the banner
        p('min_text_letters', 5)     # of AEROTHON's 8, in sequence
        p('min_letter_components', 3)  # separate white blobs in a band
        p('min_component_frac', 0.0015)
        p('min_board_green_frac', 0.25)
        # How often the OCR confirmation runs. Structure runs every frame.
        p('ocr_interval_s', 1.0)
        # How many candidate boards one read may try before giving up.
        p('ocr_max_regions', 3)
        # The SECOND lettering path. The brightness path above asks whether a
        # pixel is brighter than the BOARD; under a shadow gradient the board's
        # median is set by its sunlit half and the shaded letters fall under
        # it. Measured on the rendered banner with a linear shade ramp: 8
        # letters read at full light, 0 by the time the far edge is at 25%.
        #
        # This path asks a strictly local question instead -- is this pixel
        # brighter than its immediate neighbourhood -- which a shadow moves
        # uniformly and so cannot break. It runs BESIDE the brightness path,
        # not instead of it; either may confirm.
        # How many letters the banner carries. Used to judge which
        # lettering path segmented it most plausibly.
        p('expected_letters', 8)
        # OFF by default. Added for shadow robustness and measured to help on
        # a rendered fixture -- then measured in flight to do net harm: it
        # accepted a 97x49 sliver of banner at the frame edge as a whole
        # banner (which the sweep then aligned to for twelve straight frames),
        # and it fragmented the real banner badly enough that the derived
        # board came out taller than wide and the aspect gate refused it.
        # The "inverse" path above is what the shaded/grey-lettered cases
        # actually needed. Kept switchable rather than deleted.
        p('stroke_path', False)
        p('stroke_block_frac', 0.25)   # window size as a fraction of ROI height
        p('stroke_offset', 6)          # how far above the local mean to count

        image_topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()

        self.create_subscription(Image, image_topic, self.on_image, 5)
        self.pub = self.create_publisher(Vector3, '/percep/banner', 10)
        self.pub_detail = self.create_publisher(String, '/percep/banner/detail', 10)
        self.pub_annot = self.create_publisher(Image, '/percep/banner/annotated', 5)
        self._ocr_t0 = 0.0
        self._ocr_cache = {"text": "", "text_letters": 0,
                           "text_confirmed": False, "text_reader": ""}
        self._read_candidates = []
        self._ocr_box = None
        self.get_logger().info(
            f"perception_banner up (identity-gated); image_topic={image_topic}")

    def _g(self, n):
        return self.get_parameter(n).value

    def min_area_px(self, w, h):
        """Smallest blob worth calling a banner, in pixels, for THIS camera.

        Replaces `min_area_frac = 0.01` (geometry audit E1). A fixed fraction
        of the frame silently encodes a range: at 640x480 and 60 deg HFOV, 1%
        of frame is a 2 m banner at about 21 m — a number nobody chose and
        nobody can change without re-deriving it.

        Derived instead from the projection: a banner of `banner_width_m` x
        `banner_height_m` at `max_detect_range_m` covers

            (W_px * bw / ground_width) * (W_px * bh / ground_width)

        pixels, where ground_width = 2 * R * tan(HFOV/2). `area_safety` scales
        that down because a banner seen obliquely or partly occluded projects
        smaller than the ideal rectangle.

        An explicit `min_area_frac > 0` still overrides, so a specific arena
        can pin it if the derivation ever disagrees with reality.
        """
        pinned = float(self._g('min_area_frac'))
        if pinned > 0.0:
            return pinned * w * h

        rng = float(self._g('max_detect_range_m'))
        hfov = float(self._g('camera_hfov'))
        gw = 2.0 * rng * math.tan(hfov / 2.0)
        if gw <= 0.0:
            return 0.0
        px_per_m = w / gw
        area = (float(self._g('banner_width_m')) * px_per_m) * \
               (float(self._g('banner_height_m')) * px_per_m)
        return area * float(self._g('area_safety'))

    # ------------------------------------------------------------------ #
    def green_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lo = np.array([self._g('h_lo'), self._g('s_lo'), self._g('v_lo')])
        hi = np.array([self._g('h_hi'), self._g('s_hi'), self._g('v_hi')])
        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        return mask

    def identity(self, frame, bbox):
        """(is_banner, reason, stats, board_bbox) for the green region at `bbox`.

        Looks for white lettering structure inside the green region, and
        returns the bounding box of the BOARD ITSELF rather than of whatever
        green the mask happened to connect.

        WHY THAT DISTINCTION MATTERS
            The banner is mounted on a green gate that leads into a green
            corridor. In the image those are one connected green blob, many
            times the area of the board. Measuring white content over the
            whole blob put the lettering at well under the 2% floor, so the
            real banner was rejected as "no white lettering" — a live run
            swept 180 degrees without ever identifying the gate it was
            looking straight at.

            So the lettering is found FIRST and the board is derived from it:
            a horizontal band of several separate white components, inside
            green. That is the signature a blank tarp cannot fake, and it does
            not care how much other green is attached.
        """
        x, y, bw, bh = bbox
        roi = frame[y:y + bh, x:x + bw]
        if roi.size == 0:
            return False, "empty roi", {}, bbox

        # BOTH paths, every frame. Running the second only when the first
        # fails would make the stroke path unreachable whenever brightness
        # scrapes past the component floor with three fragments of a letter --
        # which is exactly what it does under a shadow gradient (measured: 3
        # components, 0 letters read). Either path may confirm; the one that
        # actually READS the lettering wins.
        # Candidates, ranked -- never merged. Two questions crossed:
        #
        #   light vs dark   is the lettering brighter or darker than the board
        #                   it sits on? The rendered banner is white on green;
        #                   the arena's own gate is GREY on green. Both exist.
        #   close scale     how big is a letter relative to the region it was
        #                   found in? On a banner filling the frame that is
        #                   large; on a gate whose bounding box spans posts and
        #                   gusset it is small. Measured, one fixed fraction
        #                   cannot serve both: 0.10 reads the gate and 0.22
        #                   reads the banner, and each fails the other.
        #
        # So all four run and _rank picks. That is the same rule that settled
        # light vs dark, and it costs four cheap threshold passes.
        # NOTE: a second, wider close scale was tried here so that one
        # configuration could read both the rendered banner (letters large
        # relative to their region) and the arena's gate (letters small
        # relative to a region spanning posts and gusset). Measured, it made
        # things WORSE than either scale alone -- it reintroduced both the
        # frame-edge speck and the 0.75 aspect rejection. Reverted rather than
        # tuned; the single scale below is the configuration that measures
        # clean on the captured frames.
        # Both sides judge against the same board: measure it once.
        ref = self._board_reference(roi)
        attempts = [(name, self.lettering_mask(roi, dark=dark, ref=ref))
                    for name, dark in (("brightness", False),
                                       ("inverse", True))]
        if bool(self._g('stroke_path')):
            attempts.append(("stroke", self.lettering_mask_stroke(roi)))

        best = None
        for path, white in attempts:
            ok, reason, info, board = self._identify_with(frame, bbox, white)
            info["lettering_path"] = path
            cand = self._rank(ok, info, int(self._g('expected_letters')))
            if best is None or cand > best[0]:
                best = (cand, (ok, reason, info, board))
        return best[1]

    @staticmethod
    def _rank(ok, info, expected_letters=8):
        """How good a candidate reading is. Higher wins.

        WHY THE LAST TERM IS NOT "MORE COMPONENTS"

            It was, and it rewarded the failure mode. When neither path reads
            the lettering, both are structural identifications, and ranking
            them by component count hands the decision to whichever one
            shattered the board into the most pieces. Over-segmentation is
            exactly what the stroke path does when it is struggling.

            Watched live on a foreshortened board, that produced rows of
            eighteen glyphs for a word with eight letters:

                BANNER  ???AER????????????   ID   via stroke

            So the tie-break is PLAUSIBILITY: how close the component count is
            to the number of letters the banner actually has. Eight beats
            eighteen and beats two.
        """
        return (bool(ok),
                bool(info.get("text_confirmed")),
                int(info.get("text_letters") or 0),
                -abs(int(info.get("components") or 0) - int(expected_letters)))

    def _identify_with(self, frame, bbox, white):
        """One lettering mask, judged. Shared by both paths on purpose: a
        second copy of this reasoning would be a second definition of what
        counts as the banner."""
        x, y, bw, bh = bbox
        bh_frame, bw_frame = frame.shape[:2]

        n_lab, _, stats, _ = cv2.connectedComponentsWithStats(white, connectivity=8)
        min_area = float(self._g('min_component_frac')) * bw * bh
        comps = [i for i in range(1, n_lab)
                 if stats[i, cv2.CC_STAT_AREA] >= min_area]

        info = {"white_frac": round(float(np.count_nonzero(white))
                                    / float(bw * bh), 4),
                "components": len(comps)}

        need = int(self._g('min_letter_components'))
        if len(comps) < need:
            return (False,
                    f"only {len(comps)} white component(s); lettering expected",
                    info, bbox)

        # The reading happens ONCE PER FRAME, on the winning region, in
        # on_image() -- not here. Calling an OCR engine per candidate per
        # lettering path measured 517 ms a frame (1.9 Hz), and the sweep needs
        # frames at camera rate to accumulate a dwell.


        band = self._lettering_band(stats, comps, bh)
        if band is None:
            return (False, "white components are not in a horizontal band",
                    info, bbox)

        bx0, bx1, by0, by1 = band
        info["band_components"] = info.get("band_components", 0)

        # The board is the lettering band grown to plausible board proportions.
        # Growing rather than guessing keeps this tied to something measured.
        pad_y = int(0.9 * (by1 - by0))
        pad_x = int(0.15 * (bx1 - bx0))
        ox0 = max(0, bx0 - pad_x)
        ox1 = min(bw, bx1 + pad_x)
        oy0 = max(0, by0 - pad_y)
        oy1 = min(bh, by1 + pad_y)
        board = (x + ox0, y + oy0, max(1, ox1 - ox0), max(1, oy1 - oy0))

        # The BOARD has to be big enough to be worth flying at, not just the
        # green region it was found inside. Those are different things and
        # only the green region was ever checked.
        #
        # MEASURED, seed 1001, watched flight: the sweep confirmed a banner on
        # 12 frames out of 12 and aligned to a board of 1408 px in a 921600 px
        # frame -- 0.15% of the image, hard against the right edge. The real
        # banner, once the aircraft turned toward it, measured 56430 px. The
        # green corridor it sat in comfortably passed the contour-area gate
        # the whole time.
        board_area = float(board[2]) * float(board[3])
        info["board_area_px"] = int(board_area)
        if board_area < self.min_area_px(bw_frame, bh_frame):
            return (False,
                    f"board {int(board_area)} px below the "
                    f"{self.min_area_px(bw_frame, bh_frame):.0f} px minimum "
                    f"for a banner worth aligning to", info, board)

        sub = white[oy0:oy1, ox0:ox1]
        frac = (float(np.count_nonzero(sub)) / float(sub.size)) if sub.size else 0.0
        info["board_white_frac"] = round(frac, 4)

        if frac < float(self._g('min_white_frac')):
            return False, f"no white lettering on the board ({frac:.3f})", info, board
        if frac > float(self._g('max_white_frac')):
            return False, f"mostly white, not a green banner ({frac:.3f})", info, board

        # The board must actually be green, not a white sign on a green fence.
        green = self.green_mask(frame[board[1]:board[1] + board[3],
                                      board[0]:board[0] + board[2]])
        green_frac = (float(np.count_nonzero(green)) / float(green.size)
                      if green.size else 0.0)
        info["board_green_frac"] = round(green_frac, 4)
        if green_frac < float(self._g('min_board_green_frac')):
            return (False, f"board is not green enough ({green_frac:.3f})",
                    info, board)

        return True, "", info, board

    def lettering_mask_stroke(self, roi):
        """Lettering by LOCAL contrast, so a shadow cannot hide it.

        WHY A SECOND PATH RATHER THAN A LOOSER THRESHOLD

            Loosening the brightness ratio to admit shaded letters also admits
            every bright patch of board, and the identity check exists to
            refuse exactly that. The two questions are different, and the
            honest answer is to ask both:

                brightness  is this pixel brighter than the board?
                stroke      is this pixel brighter than what surrounds it?

            A shadow gradient changes the first answer and not the second,
            because the window travels with the pixel. A blank green board
            fails both, which is what keeps a tarpaulin out.

        The window is sized from the ROI so it stays a few letter-strokes
        wide at any range; an absolute window would be its own hidden
        assumption about distance.
        """
        h, w = roi.shape[:2]
        if h < 8 or w < 8:
            return np.zeros((h, w), np.uint8)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blk = int(max(11, h * float(self._g('stroke_block_frac'))))
        blk |= 1                                     # adaptiveThreshold wants odd
        mask = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
            blk, -int(self._g('stroke_offset')))

        # Still has to be less saturated than the board it sits on: a green
        # highlight is locally bright and is not a letter.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        board = self.green_mask(roi)
        if np.count_nonzero(board) > 0:
            s_board = float(np.median(hsv[:, :, 1][board > 0]))
            s_max = min(float(self._g('white_s_max')),
                        max(20.0, s_board * float(self._g('white_s_ratio'))))
            mask = cv2.bitwise_and(mask, cv2.inRange(
                hsv, np.array([0, 0, 0]), np.array([179, int(s_max), 255])))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(board))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    def _board_reference(self, roi, close_frac=None):
        """What lettering_mask judges against, for one ROI: (hsv, board,
        v_board, v_min, s_max, solid). Independent of which side of the board
        the lettering falls, so identity() computes it once for both."""
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        sat, val = hsv[:, :, 1], hsv[:, :, 2]

        board = self.green_mask(roi)
        if np.count_nonzero(board) > 0:
            v_board = float(np.median(val[board > 0]))
            s_board = float(np.median(sat[board > 0]))
            v_min = max(float(self._g('white_v_min')),
                        v_board * float(self._g('white_v_ratio')))
            s_max = min(float(self._g('white_s_max')),
                        max(20.0, s_board * float(self._g('white_s_ratio'))))
        else:
            v_min = float(self._g('white_v_min'))
            s_max = float(self._g('white_s_max'))
            v_board = 255.0            # no board reference: bright path only

        # Letters are holes in the green panel: see lettering_mask for why
        # the board is closed with a kernel scaled to the region.
        cf = (float(self._g('board_close_frac')) if close_frac is None
              else float(close_frac))
        k = int(max(3, roi.shape[0] * cf)) | 1
        solid = cv2.morphologyEx(board, cv2.MORPH_CLOSE,
                                 np.ones((k, k), np.uint8))
        return hsv, board, v_board, v_min, s_max, solid

    def lettering_mask(self, roi, dark=False, close_frac=None, ref=None):
        """Pixels that are lettering ON THIS BOARD, judged against the board.

        The letters are whatever is markedly brighter and markedly less
        saturated than the green they sit on. That holds under studio light,
        flat ambient sim light, overcast and direct sun; an absolute
        "V >= 170" holds under exactly one of those, and a live frame proved
        it (`components: 0` for a banner filling the image).

        The absolute floor/ceiling are kept as a backstop so a dark green board
        cannot make near-black pixels count as lettering.
        """
        hsv, board, v_board, v_min, s_max, solid = (
            ref if ref is not None else self._board_reference(roi, close_frac))

        # CONTRAST, in either direction -- not "brighter".
        #
        # MEASURED on the arena's own gate (seed 1001, captured in flight):
        # the lettering is GREY, BGR (132,132,132), on a green board at
        # HSV V=184. The letters are DARKER than the board. The old rule
        # demanded v >= max(90, 184*1.08) = 199, which those letters can never
        # reach, so the real banner was unreadable from every angle and every
        # range -- and the stroke path added beside it failed identically,
        # because it too only ever looked for locally BRIGHTER pixels.
        #
        # What is actually invariant is that the lettering is much LESS
        # SATURATED than the board it sits on, and clearly separated from it in
        # value. Which side of the board it falls on is a property of the
        # paint, not of the alphabet.
        # Two SEPARATE candidate masks, never a union. Unioning them lets the
        # darker side contribute shadow fragments to a board whose lettering
        # the bright side already reads perfectly -- measured: it turned a
        # clean "AEROTHON" on the rendered fixture into twelve unreadable
        # glyphs. They compete in identity() and the better one wins.
        if dark:
            v_dark = v_board * float(self._g('dark_v_ratio'))
            mask = cv2.inRange(
                hsv, np.array([0, 0, int(self._g('dark_v_floor'))]),
                np.array([179, int(s_max), int(max(1, v_dark))]))
        else:
            mask = cv2.inRange(hsv, np.array([0, 0, int(v_min)]),
                               np.array([179, int(s_max), 255]))
        # Anything the green mask claims is board is not lettering, whatever
        # its brightness -- a specular highlight on green is not a letter.
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(board))

        # Lettering has to be ON the board, not merely inside its BOUNDING
        # BOX. The arena's banner is a GATE: two posts, a lettered panel and a
        # gusset, standing open in the middle. Its bounding box therefore
        # contains a large area of grey corridor wall, and the wall is the
        # same grey as the lettering -- unsaturated, mid-value. Reading it as
        # lettering is what produced fifteen glyphs of noise and a "board"
        # taller than it was wide (aspect 0.75), which the aspect gate then
        # correctly refused.
        #
        # Letters are holes in the green panel; the gate opening is a hole
        # too, but a hundred times larger. Closing the green mask with a
        # kernel scaled to the region fills the letters and leaves the opening
        # open, which separates the two without knowing anything about this
        # particular gate. That closed board is `solid`, from
        # _board_reference.
        confined = cv2.bitwise_and(mask, solid)
        # A GUARD, not a filter. On a board that fills its own region -- the
        # rendered banner, and every synthetic fixture -- the closed green
        # already contains the lettering, so this changes nothing. On a gate
        # standing open in front of a grey wall it removes the wall. But if it
        # would remove nearly EVERYTHING, the region is not a board with holes
        # in it and the constraint does not apply; keeping the confined mask
        # there would reject boards that the unconstrained mask reads fine.
        kept = float(np.count_nonzero(confined))
        total = float(np.count_nonzero(mask))
        if total > 0 and kept / total >= float(self._g('board_confine_min')):
            mask = confined
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    def _lettering_band(self, stats, comps, roi_h):
        """Bounding box of the largest horizontal run of white components.

        Lettering is several blobs at roughly the same height and of roughly
        the same size. A frame edge, a reflection or a single stripe is not.
        Returns (x0, x1, y0, y1) in ROI coordinates, or None.
        """
        need = int(self._g('min_letter_components'))
        boxes = [(stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                  stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
                 for i in comps]
        boxes = [b for b in boxes if b[3] > 1]
        # A LETTER HAS A SHAPE. Measured on the arena's own gate, the mask
        # returns the eight letters at 24-49 px wide by 84-120 tall (aspect
        # 0.28-0.42) mixed in with two structural pieces:
        #
        #     x=0   w= 29  h=443   the post edge      aspect 0.07
        #     x=57  w=359  h=186   the panel interior aspect 1.93
        #
        # Those two dragged the band's vertical extent across the whole gate,
        # which made the derived board 440x587 -- taller than wide -- and the
        # aspect gate refused the real banner on every frame. Nothing here
        # knows about this gate: a character is simply neither a hairline nor
        # a slab.
        lo = float(self._g('letter_min_aspect'))
        hi = float(self._g('letter_max_aspect'))
        boxes = [b for b in boxes if lo <= (b[2] / float(b[3])) <= hi]

        best = None
        for bx, by, bw_, bh_ in boxes:
            cy = by + bh_ / 2.0
            tol = max(3.0, bh_ * 0.8)
            row = [c for c in boxes
                   if abs((c[1] + c[3] / 2.0) - cy) <= tol
                   and 0.4 * bh_ <= c[3] <= 2.5 * bh_]
            if len(row) < need:
                continue
            x0 = min(c[0] for c in row)
            x1 = max(c[0] + c[2] for c in row)
            y0 = min(c[1] for c in row)
            y1 = max(c[1] + c[3] for c in row)
            span = x1 - x0
            if best is None or span > best[0]:
                best = (span, (x0, x1, y0, y1), len(row))
        return None if best is None else best[1]

    # ------------------------------------------------------------------ #
    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge: {e}")
            return

        h, w = frame.shape[:2]
        out = Vector3(x=0.0, y=0.0, z=0.0)
        detail = {"identified": False, "reason": "", "candidates": 0}
        # Per FRAME, not per node: the regions worth reading move.
        self._read_candidates = []

        mask = self.green_mask(frame)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detail["candidates"] = len(cnts)
        # THE LARGEST GREEN REGION, identified or not. A board seen edge-on
        # shows no lettering -- nothing here can call it the banner -- but it
        # is still where the gate is. The mission orbits it to get a face-on
        # view (banner_orbit.py); without this it only knew "something green
        # somewhere" and searched the wrong way.
        if cnts:
            big = max(cnts, key=cv2.contourArea)
            gx, gy, gw, gh = cv2.boundingRect(big)
            detail["green_px"] = [int(gx), int(gy), int(gw), int(gh)]
            detail["green_area_px"] = int(cv2.contourArea(big))
            detail["green_bearing"] = round(((gx + gw / 2.0) - w / 2.0) / (w / 2.0), 3)
            detail["image_wh"] = [int(w), int(h)]

        best = None
        for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:5]:
            area = cv2.contourArea(c)
            x, y, bw, bh = cv2.boundingRect(c)
            # Rejections used to be SILENT: a candidate dropped on area or
            # aspect left detail["reason"] empty, so the panel showed
            # "identified: false, reason: ''" and gave an operator nothing to
            # work with. Every rejection now says why.
            if area < self.min_area_px(w, h):
                if not detail["reason"]:
                    detail["reason"] = (f"green region too small "
                                        f"({area:.0f} px, need "
                                        f"{self.min_area_px(w, h):.0f})")
                continue
            # NOTE: the aspect gate is deliberately NOT applied here. It is a
            # statement about the BOARD's proportions, and the region found by
            # the green mask is the gate plus the corridor behind it plus any
            # green fence attached to them. A live probe reported
            #
            #   "green region aspect 17.40 outside 1.2-8.0"   candidates: 3
            #
            # for a frame containing the real banner: the container is a long
            # thin band, the board inside it is not. Aspect is checked on the
            # derived board, below.

            ok, reason, info, board = self.identity(frame, (x, y, bw, bh))
            if not bool(self._g('require_identity')):
                ok, reason, board = True, "", (x, y, bw, bh)

            if ok:
                board_aspect = board[2] / max(1, board[3])
                if not (self._g('min_aspect') <= board_aspect
                        <= self._g('max_aspect')):
                    ok = False
                    reason = (f"board aspect {board_aspect:.2f} outside "
                              f"{self._g('min_aspect')}-{self._g('max_aspect')}")
                info["board_aspect"] = round(board_aspect, 2)

            # TEXT RESCUE -- confirming only, never a veto.
            #
            # If the structural gates rejected this candidate but the letters
            # read as AEROTHON, accept it. Seed 1001 fails the return lap on
            # "board aspect 0.73 outside 1.2-8.0": from the delivery-zone side
            # the board presents at an aspect the gate refuses, while the
            # lettering is perfectly legible and was never consulted.
            #
            # Deliberately one-directional. The outbound identification works
            # on every arena today, and a check that can only ever turn a NO
            # into a YES cannot regress it -- the worst case is exactly
            # today's behaviour.
            if not ok and info.get("text_confirmed"):
                ok = True
                reason = ""
                info["rescued_by_text"] = True

            # Image-space box for the shared overlay stream, so the GCS can
            # show the live camera with detections drawn without any detector
            # having to own the composite.
            label = "BANNER" if ok else "GREEN, NOT BANNER"
            if info.get("rescued_by_text"):
                label = f"BANNER [{info.get('text', '')}]"
            detail.setdefault("boxes", []).append(
                {"rect": [int(board[0]), int(board[1]),
                          int(board[2]), int(board[3])],
                 "label": label, "ok": bool(ok)})

            colour = (0, 255, 0) if ok else (0, 0, 255)
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (120, 120, 120), 1)
            cv2.rectangle(frame, (board[0], board[1]),
                          (board[0] + board[2], board[1] + board[3]), colour, 2)
            cv2.putText(frame, "BANNER" if ok else "GREEN, NOT BANNER",
                        (board[0], max(0, board[1] - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)

            # Collect every derived BOARD as something the reader could look
            # at. Not the raw green blob: for the rendered banner the biggest
            # blob is 1249x472 of gate and corridor and tesseract reads
            # nothing from it, while the 1078x300 board beside it reads
            # AEROTHON. And not merely the largest board either, for the same
            # reason -- the gate blob's board is the larger of the two.
            self._read_candidates.append(
                (int(board[0]), int(board[1]), int(board[2]), int(board[3])))

            if ok and best is None:
                # NOTE: first-accepted wins, in whatever order contours come
                # out. That is suspected of being wrong -- watched live, the
                # tracked box stayed at a roughly fixed place in frame while
                # the aircraft yawed 25 degrees, which the banner would not do
                # -- but "largest wins" was tried and regressed nine tests on
                # the rendered frontal frame. `board_px` / `board_area_px` are
                # published below so the next flight can say what was actually
                # being tracked instead of leaving it to inference.
                best = (board[0], board[1], board[2], board[3], info)
            elif not ok and not detail["reason"]:
                detail["reason"] = reason
                detail.update(info)

        # ---- ONE reading per frame, rate-limited ---- #
        #
        # Structure runs at camera rate and decides; the reading confirms. A
        # fresh OCR pass per green candidate per lettering path measured
        # 517 ms a frame (1.9 Hz) and starved the sweep of the frames its
        # dwell is counted from. Once a second costs about 25 ms amortised and
        # still confirms long before a five-second dwell completes.
        if best is not None:
            read_boxes = [tuple(best[:4])]
        else:
            # Biggest first, after dropping shapes that cannot be a board.
            #
            # Widest-first was tried and put a 1130x12 sliver (aspect 94) and
            # a 344x60 offcut ahead of the real 1078x300 banner, so the reader
            # spent its budget on degenerate strips and never saw it. The
            # bounds here are deliberately loose and independent of the
            # identity gate's own aspect range -- this only decides what to
            # SPEND A READ ON, and the reading still judges.
            plausible = [b for b in self._read_candidates
                         if b[3] >= 20 and 0.8 <= b[2] / float(max(1, b[3])) <= 12.0]
            read_boxes = sorted(plausible, key=lambda b: -(b[2] * b[3]))
        read_boxes = read_boxes[:int(self._g('ocr_max_regions'))]
        read_box = read_boxes[0] if read_boxes else None

        # A cached reading describes the region it came from. When the
        # aircraft yaws to the next sweep heading that region jumps, and
        # carrying the text across would attribute one heading's banner to
        # another -- the exact class of error this rewrite exists to remove.
        if read_box is not None and self._ocr_box is not None:
            px, py, pw, ph = self._ocr_box
            jump = math.hypot(read_box[0] - px, read_box[1] - py)
            if (jump > 0.5 * max(pw, ph)
                    or read_box[2] * read_box[3] < 0.4 * pw * ph):
                self._ocr_cache = {"text": "", "text_letters": 0,
                                   "text_confirmed": False, "text_reader": ""}
                self._ocr_t0 = 0.0

        now = time.monotonic()
        if (read_box is not None
                and now - self._ocr_t0 >= float(self._g('ocr_interval_s'))):
            self._ocr_box = tuple(read_box)
            self._ocr_t0 = now
            for rx, ry, rw, rh in read_boxes:
                sub = frame[ry:ry + rh, rx:rx + rw]
                if not sub.size:
                    continue
                ok_txt, txt, reader, nletters = reads_banner(
                    sub, min_letters=int(self._g('min_text_letters')),
                    board_mask=self.green_mask(sub))
                self._ocr_cache = {"text": txt, "text_letters": int(nletters),
                                   "text_confirmed": bool(ok_txt),
                                   "text_reader": reader}
                self._ocr_box = (rx, ry, rw, rh)
                if ok_txt:
                    break

        # TEXT RESCUE, at the level the reading now happens.
        #
        # It used to live inside the per-candidate check, which could see the
        # reading because the reading was taken there. Moving OCR to once per
        # frame (for speed) silently removed the rescue: the structural gates
        # rejected a board, the engine read AEROTHON off it a moment later,
        # and nothing connected the two.
        #
        # Still one-directional -- it can only turn a NO into a YES, so the
        # worst case is exactly the structural verdict.
        if (best is None and self._ocr_cache.get("text_confirmed")
                and self._ocr_box is not None):
            rx, ry, rw, rh = self._ocr_box
            best = (rx, ry, rw, rh, {"rescued_by_text": True})
            detail["rescued_by_text"] = True

        if best is not None:
            x, y, bw, bh, info = best
            cx, cy = x + bw / 2, y + bh / 2
            out.x = float((cx - w / 2) / (w / 2))
            out.y = float((cy - h / 2) / (h / 2))
            out.z = 1.0
            detail.update({"identified": True, "reason": "", **info,
                           "bearing": round(out.x, 3),
                           # What was actually tracked, so a bearing that does
                           # not respond to yaw can be told apart from one that
                           # is tracking the wrong object.
                           "board_px": [int(x), int(y), int(bw), int(bh)],
                           "board_area_px": int(bw) * int(bh)})
        elif detail["candidates"]:
            # Green things present, none of them the banner. Distinguish this
            # from "nothing green in frame": they are very different for an
            # operator watching the panel.
            out.z = 0.5

        # LAST, so nothing can clobber it. Applied earlier, the per-candidate
        # info's own (empty) text fields silently overwrote it: the OCR ran
        # correctly every frame and its answer never reached the topic.
        detail.update(self._ocr_cache)

        self.pub.publish(out)
        self.pub_detail.publish(String(data=json.dumps(detail)))
        # The boxes above are drawn regardless -- the OCR reads from this same
        # frame -- but the 2.7 MB conversion is only for a GCS that watches.
        if self.pub_annot.get_subscription_count():
            try:
                self.pub_annot.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
            except Exception:  # noqa: BLE001
                pass


def main():
    rclpy.init()
    node = BannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
