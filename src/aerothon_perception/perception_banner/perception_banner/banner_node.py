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
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

from perception_banner.glyphs import reads_as_target
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Vector3
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
        p('max_detect_range_m', 25.0)  # beyond this it is not worth aligning to
        p('camera_hfov', 1.0472)
        p('area_safety', 0.5)          # accept half the ideal projected area:
                                       # oblique views and partial occlusion
        p('min_aspect', 1.2); p('max_aspect', 8.0)
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
        p('min_white_frac', 0.02)      # white content inside the green region
        p('max_white_frac', 0.60)      # a mostly-white board is not the banner
        p('min_text_letters', 5)     # of AEROTHON's 8, in sequence
        p('min_letter_components', 3)  # separate white blobs in a band
        p('min_component_frac', 0.0015)
        p('min_board_green_frac', 0.25)
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
        p('stroke_path', True)
        p('stroke_block_frac', 0.25)   # window size as a fraction of ROI height
        p('stroke_offset', 6)          # how far above the local mean to count

        image_topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()

        self.create_subscription(Image, image_topic, self.on_image, 5)
        self.pub = self.create_publisher(Vector3, '/percep/banner', 10)
        self.pub_detail = self.create_publisher(String, '/percep/banner/detail', 10)
        self.pub_annot = self.create_publisher(Image, '/percep/banner/annotated', 5)
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
        attempts = [("brightness", self.lettering_mask(roi))]
        if bool(self._g('stroke_path')):
            attempts.append(("stroke", self.lettering_mask_stroke(roi)))

        best = None
        for path, white in attempts:
            ok, reason, info, board = self._identify_with(frame, bbox, white)
            info["lettering_path"] = path
            cand = (ok, bool(info.get("text_confirmed")),
                    int(info.get("text_letters") or 0),
                    int(info.get("components") or 0))
            if best is None or cand > best[0]:
                best = (cand, (ok, reason, info, board))
        return best[1]

    def _identify_with(self, frame, bbox, white):
        """One lettering mask, judged. Shared by both paths on purpose: a
        second copy of this reasoning would be a second definition of what
        counts as the banner."""
        x, y, bw, bh = bbox

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

        # Read the lettering, once, so the result is available both as a
        # rescue below and as evidence in the detail topic. Confirming only:
        # it may overturn a rejection, never an acceptance.
        ocr_ok, ocr_text, ocr_n = self.read_lettering(white, stats, comps)
        info["text"] = ocr_text
        info["text_letters"] = ocr_n
        info["text_confirmed"] = bool(ocr_ok)

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

    def read_lettering(self, white, stats, comps):
        """(confirmed, text, matched) from the white blobs already segmented.

        Left-to-right over the components gate 3 found, resampled to the 5x7
        templates in perception_banner.glyphs. Costs about a millisecond and
        adds no dependency -- it answers one question, "do these blobs spell
        AEROTHON", which is the question seed 1001 needs answered when the
        aspect gate rejects the board from the delivery-zone side.
        """
        boxes = sorted(
            ((stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
              stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
             for i in comps),
            key=lambda b: b[0])
        patches = [white[by:by + bh_, bx:bx + bw_]
                   for bx, by, bw_, bh_ in boxes
                   if bw_ > 0 and bh_ > 0]
        if len(patches) < int(self._g('min_text_letters')):
            return False, "", 0
        return reads_as_target(patches,
                               min_letters=int(self._g('min_text_letters')))

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

    def lettering_mask(self, roi):
        """Pixels that are lettering ON THIS BOARD, judged against the board.

        The letters are whatever is markedly brighter and markedly less
        saturated than the green they sit on. That holds under studio light,
        flat ambient sim light, overcast and direct sun; an absolute
        "V >= 170" holds under exactly one of those, and a live frame proved
        it (`components: 0` for a banner filling the image).

        The absolute floor/ceiling are kept as a backstop so a dark green board
        cannot make near-black pixels count as lettering.
        """
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

        mask = cv2.inRange(hsv, np.array([0, 0, int(v_min)]),
                           np.array([179, int(s_max), 255]))
        # Anything the green mask claims is board is not lettering, whatever
        # its brightness -- a specular highlight on green is not a letter.
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(board))
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
        # Ignore long thin runs: the banner's own white frame is one of these
        # and would otherwise masquerade as a row of letters.
        boxes = [b for b in boxes if b[2] <= 0.6 * roi_h * 6 and b[3] > 1]

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

        mask = self.green_mask(frame)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detail["candidates"] = len(cnts)

        best = None
        for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:5]:
            area = cv2.contourArea(c)
            x, y, bw, bh = cv2.boundingRect(c)
            aspect = bw / max(1, bh)
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

            if ok and best is None:
                best = (board[0], board[1], board[2], board[3], info)
            elif not ok and not detail["reason"]:
                detail["reason"] = reason
                detail.update(info)

        if best is not None:
            x, y, bw, bh, info = best
            cx, cy = x + bw / 2, y + bh / 2
            out.x = float((cx - w / 2) / (w / 2))
            out.y = float((cy - h / 2) / (h / 2))
            out.z = 1.0
            detail.update({"identified": True, "reason": "", **info,
                           "bearing": round(out.x, 3)})
        elif detail["candidates"]:
            # Green things present, none of them the banner. Distinguish this
            # from "nothing green in frame": they are very different for an
            # operator watching the panel.
            out.z = 0.5

        self.pub.publish(out)
        self.pub_detail.publish(String(data=json.dumps(detail)))
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
