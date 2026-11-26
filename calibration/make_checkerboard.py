"""Generate a printable checkerboard (letter size, landscape) for Zhang calibration.
10 x 7 squares -> 9 x 6 inner corners, 1 inch (25.4 mm) squares at 300 DPI."""
from PIL import Image, ImageDraw, ImageFont

DPI = 300
PAGE_W, PAGE_H = int(11 * DPI), int(8.5 * DPI)  # letter, landscape
SQ = DPI                                        # 1 inch = 25.4 mm per square
COLS, ROWS = 10, 7                              # squares -> 9 x 6 inner corners

img = Image.new("L", (PAGE_W, PAGE_H), 255)
d = ImageDraw.Draw(img)
x0 = (PAGE_W - COLS * SQ) // 2
y0 = (PAGE_H - ROWS * SQ) // 2 - 40
for r in range(ROWS):
    for c in range(COLS):
        if (r + c) % 2 == 0:
            d.rectangle([x0 + c * SQ, y0 + r * SQ, x0 + (c + 1) * SQ - 1, y0 + (r + 1) * SQ - 1], fill=0)

# 100 mm scale bar to verify the print was not rescaled
mm = DPI / 25.4
by = y0 + ROWS * SQ + 60
d.rectangle([x0, by, x0 + round(100 * mm), by + 12], fill=0)
try:
    font = ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 36)
except OSError:
    font = ImageFont.load_default()
d.text((x0 + round(100 * mm) + 30, by - 14),
       "<- must measure 100 mm.  Square = 25.4 mm, 9x6 inner corners. Print at 100% / Actual size.",
       fill=0, font=font)

img.save("checkerboard_9x6_25.4mm_letter.png", dpi=(DPI, DPI))
img.convert("RGB").save("checkerboard_9x6_25.4mm_letter.pdf", resolution=DPI)
print("saved", img.size)
