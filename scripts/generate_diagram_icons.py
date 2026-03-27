#!/usr/bin/env python3
"""Generate PNG hardware icons used by the WARD tech stack diagram."""

from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
ICON_DIR = ROOT / "assets" / "diagram-icons"
SIZE = 256
PADDING = 20


def canvas(color: str) -> tuple[Image.Image, ImageDraw.ImageDraw]:
    image = Image.new("RGBA", (SIZE, SIZE), color)
    return image, ImageDraw.Draw(image)


def rounded_panel(draw: ImageDraw.ImageDraw, fill: str) -> None:
    draw.rounded_rectangle((0, 0, SIZE - 1, SIZE - 1), radius=48, fill=fill)


def save(image: Image.Image, name: str) -> None:
    ICON_DIR.mkdir(parents=True, exist_ok=True)
    image.save(ICON_DIR / name, format="PNG")


def draw_object_scene() -> None:
    image, draw = canvas("#f4efe8")
    rounded_panel(draw, "#f4efe8")
    draw.ellipse((48, 184, 208, 224), fill="#d6cec5")
    draw.polygon([(80, 92), (136, 66), (178, 92), (122, 120)], fill="#d7a56d")
    draw.polygon([(80, 92), (80, 150), (122, 176), (122, 120)], fill="#b87b47")
    draw.polygon([(122, 120), (122, 176), (178, 150), (178, 92)], fill="#8f5d35")
    draw.arc((158, 54, 226, 122), start=220, end=320, fill="#4a90a4", width=12)
    draw.arc((150, 34, 242, 142), start=220, end=320, fill="#83bcc9", width=12)
    save(image, "object_scene.png")


def draw_tof_sensor() -> None:
    image, draw = canvas("#e9f3f8")
    rounded_panel(draw, "#e9f3f8")
    draw.rounded_rectangle((40, 52, 216, 204), radius=24, fill="#1f6f78")
    draw.rounded_rectangle((64, 76, 108, 120), radius=10, fill="#d7f0f3")
    draw.rounded_rectangle((148, 76, 192, 120), radius=10, fill="#d7f0f3")
    draw.ellipse((72, 84, 100, 112), fill="#4ab0c1")
    draw.ellipse((156, 84, 184, 112), fill="#4ab0c1")
    draw.line((128, 120, 128, 154), fill="#c7ebef", width=10)
    draw.arc((96, 144, 160, 192), start=205, end=335, fill="#ffd16a", width=8)
    draw.arc((76, 158, 180, 228), start=205, end=335, fill="#ffd16a", width=8)
    save(image, "tof_sensor.png")


def draw_nano_rp2040() -> None:
    image, draw = canvas("#edf3ff")
    rounded_panel(draw, "#edf3ff")
    draw.rounded_rectangle((74, 28, 182, 228), radius=20, fill="#2c7be5")
    draw.rounded_rectangle((98, 52, 158, 88), radius=8, fill="#dce9ff")
    draw.rounded_rectangle((96, 120, 160, 166), radius=8, fill="#133b74")
    for x in (106, 124, 142):
        draw.rectangle((x, 130, x + 10, 140), fill="#9ec4ff")
    for y in (48, 74, 100, 126, 152, 178, 204):
        draw.rounded_rectangle((48, y, 62, y + 10), radius=3, fill="#7fb0ff")
        draw.rounded_rectangle((194, y, 208, y + 10), radius=3, fill="#7fb0ff")
    save(image, "nano_rp2040.png")


def draw_drv8833() -> None:
    image, draw = canvas("#f4edf8")
    rounded_panel(draw, "#f4edf8")
    draw.rounded_rectangle((34, 58, 222, 198), radius=22, fill="#6d4aa2")
    draw.rounded_rectangle((98, 88, 158, 130), radius=8, fill="#24153a")
    for y in (72, 98, 124, 150):
        draw.rounded_rectangle((46, y, 58, y + 18), radius=3, fill="#c7b0eb")
        draw.rounded_rectangle((198, y, 210, y + 18), radius=3, fill="#c7b0eb")
    draw.line((68, 174, 188, 174), fill="#f2dd6f", width=10)
    for x in (82, 128, 174):
        draw.line((x, 160, x, 188), fill="#f2dd6f", width=8)
    save(image, "drv8833.png")


def draw_stepper_yaw() -> None:
    image, draw = canvas("#eef6ef")
    rounded_panel(draw, "#eef6ef")
    draw.rounded_rectangle((56, 68, 148, 160), radius=18, fill="#517d58")
    draw.ellipse((84, 96, 120, 132), fill="#dceadf")
    draw.rounded_rectangle((148, 104, 184, 124), radius=6, fill="#9ab59f")
    draw.arc((96, 88, 212, 204), start=260, end=20, fill="#2f9e44", width=12)
    draw.line((196, 104, 204, 136), fill="#2f9e44", width=10)
    draw.line((204, 136, 172, 128), fill="#2f9e44", width=10)
    save(image, "stepper_yaw.png")


def draw_stepper_pitch() -> None:
    image, draw = canvas("#eef2f7")
    rounded_panel(draw, "#eef2f7")
    draw.rounded_rectangle((84, 84, 176, 176), radius=18, fill="#596b86")
    draw.ellipse((112, 112, 148, 148), fill="#dce6f4")
    draw.rounded_rectangle((120, 50, 140, 84), radius=6, fill="#9fb2cc")
    draw.arc((70, 90, 190, 210), start=330, end=90, fill="#4c6ef5", width=12)
    draw.line((86, 188, 116, 194), fill="#4c6ef5", width=10)
    draw.line((86, 188, 106, 164), fill="#4c6ef5", width=10)
    save(image, "stepper_pitch.png")


if __name__ == "__main__":
    draw_object_scene()
    draw_tof_sensor()
    draw_nano_rp2040()
    draw_drv8833()
    draw_stepper_yaw()
    draw_stepper_pitch()
    print(f"Generated PNG icons in {ICON_DIR}")
