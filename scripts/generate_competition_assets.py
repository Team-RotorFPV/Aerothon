#!/usr/bin/env python3
"""Generate Mission 2 QR markers, banner, and restricted-zone signage."""

from pathlib import Path
import json

import qrcode
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "src" / "aerothon_sim" / "sim_gazebo" / "materials"
PAYLOADS = {
    "qr_start.png": "AEROTHON2026:M2:TARGET_A",
    "qr_target_a.png": "AEROTHON2026:M2:TARGET_A",
    "qr_target_b.png": "AEROTHON2026:M2:TARGET_B",
    "qr_target_c.png": "AEROTHON2026:M2:TARGET_C",
    "qr_target_d.png": "AEROTHON2026:M2:TARGET_D",
    "qr_target_e.png": "AEROTHON2026:M2:TARGET_E",
}


def make_qr(payload: str, path: Path) -> list[list[bool]]:
    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=24,
        border=4,
    )
    qr.add_data(payload)
    qr.make(fit=True)
    qr.make_image(fill_color="black", back_color="white").convert("RGB").save(path)
    return qr.get_matrix()


def font(size: int):
    for candidate in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    ):
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


def make_banner(path: Path) -> None:
    image = Image.new("RGB", (1600, 360), "#069447")
    draw = ImageDraw.Draw(image)
    draw.rectangle((20, 20, 1580, 340), outline="white", width=14)
    title = "AeroTHON 2026"
    subtitle = "AUTONOMOUS RAPID DELIVERY"
    title_font, sub_font = font(150), font(52)
    title_box = draw.textbbox((0, 0), title, font=title_font)
    sub_box = draw.textbbox((0, 0), subtitle, font=sub_font)
    draw.text(((1600 - (title_box[2] - title_box[0])) / 2, 60), title,
              font=title_font, fill="white")
    draw.text(((1600 - (sub_box[2] - sub_box[0])) / 2, 250), subtitle,
              font=sub_font, fill="#dff7e9")
    image.save(path)


def make_red_zone(path: Path) -> None:
    image = Image.new("RGB", (1200, 700), "#ed111c")
    draw = ImageDraw.Draw(image)
    draw.rectangle((20, 20, 1180, 680), outline="white", width=20)
    title = "RESTRICTED\nRED ZONE"
    title_font = font(150)
    box = draw.multiline_textbbox((0, 0), title, font=title_font, spacing=20,
                                  align="center")
    x = (1200 - (box[2] - box[0])) / 2
    y = (700 - (box[3] - box[1])) / 2
    draw.multiline_text((x, y), title, font=title_font, spacing=20,
                        align="center", fill="white")
    image.save(path)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    matrices = {}
    for filename, payload in PAYLOADS.items():
        matrices[filename] = make_qr(payload, OUT / filename)
    make_banner(OUT / "banner.png")
    make_red_zone(OUT / "red_zone.png")
    (OUT / "qr_payloads.txt").write_text(
        "\n".join(f"{name}: {payload}" for name, payload in PAYLOADS.items()) + "\n",
        encoding="utf-8",
    )
    # Gazebo's separate GUI process can fail to synchronize PBR texture
    # materials. Keep the exact QR module matrices so materialize_world.py can
    # construct scannable markers from plain box geometry with no textures.
    (OUT / "qr_matrices.json").write_text(
        json.dumps(matrices, separators=(",", ":")), encoding="utf-8")
    print(f"Generated {len(PAYLOADS)} QR images, banner, and red-zone sign in {OUT}")


if __name__ == "__main__":
    main()
