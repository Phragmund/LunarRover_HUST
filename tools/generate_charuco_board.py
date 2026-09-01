#!/usr/bin/env python3
"""Generate printable ChArUco calibration boards for the arm-mounted camera."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


DPI = 300
MM_PER_INCH = 25.4
A4_MM = (210.0, 297.0)


def mm_to_px(mm: float, dpi: int = DPI) -> int:
    return int(round(mm / MM_PER_INCH * dpi))


def make_charuco(
    squares_x: int,
    squares_y: int,
    square_mm: float,
    marker_mm: float,
    dictionary_name: str,
) -> np.ndarray:
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.Dictionary_get(dictionary_id)
    board = cv2.aruco.CharucoBoard_create(
        squares_x,
        squares_y,
        square_mm / 1000.0,
        marker_mm / 1000.0,
        dictionary,
    )
    board_px = (mm_to_px(squares_x * square_mm), mm_to_px(squares_y * square_mm))
    return board.draw(board_px, marginSize=0, borderBits=1)


def load_font(size: int) -> ImageFont.ImageFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        if Path(path).exists():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def a4_canvas() -> Image.Image:
    return Image.new("RGB", (mm_to_px(A4_MM[0]), mm_to_px(A4_MM[1])), "white")


def draw_page_guides(
    image: Image.Image,
    title: str,
    subtitle: str,
    crop_box: tuple[int, int, int, int] | None = None,
) -> None:
    draw = ImageDraw.Draw(image)
    title_font = load_font(34)
    small_font = load_font(22)
    draw.text((mm_to_px(8), mm_to_px(5)), title, fill=(0, 0, 0), font=title_font)
    draw.text((mm_to_px(8), mm_to_px(15)), subtitle, fill=(0, 0, 0), font=small_font)

    margin = mm_to_px(5)
    w, h = image.size
    line = (120, 120, 120)
    draw.line((margin, margin, margin + mm_to_px(8), margin), fill=line, width=2)
    draw.line((margin, margin, margin, margin + mm_to_px(8)), fill=line, width=2)
    draw.line((w - margin, margin, w - margin - mm_to_px(8), margin), fill=line, width=2)
    draw.line((w - margin, margin, w - margin, margin + mm_to_px(8)), fill=line, width=2)
    draw.line((margin, h - margin, margin + mm_to_px(8), h - margin), fill=line, width=2)
    draw.line((margin, h - margin, margin, h - margin - mm_to_px(8)), fill=line, width=2)
    draw.line((w - margin, h - margin, w - margin - mm_to_px(8), h - margin), fill=line, width=2)
    draw.line((w - margin, h - margin, w - margin, h - margin - mm_to_px(8)), fill=line, width=2)

    if crop_box:
        draw.rectangle(crop_box, outline=(180, 0, 0), width=3)


def save_pdf_and_png(image: Image.Image, out_base: Path) -> None:
    image.save(out_base.with_suffix(".png"), dpi=(DPI, DPI))
    image.save(out_base.with_suffix(".pdf"), resolution=DPI)


def generate_single_a4(out_dir: Path, dictionary_name: str) -> None:
    squares_x, squares_y = 7, 10
    square_mm, marker_mm = 25.0, 18.0
    board = make_charuco(squares_x, squares_y, square_mm, marker_mm, dictionary_name)
    board_img = Image.fromarray(board).convert("RGB")
    canvas = a4_canvas()
    x = (canvas.width - board_img.width) // 2
    y = mm_to_px(30)
    canvas.paste(board_img, (x, y))
    draw_page_guides(
        canvas,
        "ChArUco A4 single board",
        f"{squares_x}x{squares_y}, square={square_mm:.1f}mm, marker={marker_mm:.1f}mm, {dictionary_name}, print 100%",
        (x, y, x + board_img.width, y + board_img.height),
    )
    save_pdf_and_png(canvas, out_dir / "charuco_a4_7x10_25mm")


def generate_large_tiled(out_dir: Path, dictionary_name: str) -> None:
    squares_x, squares_y = 10, 14
    square_mm, marker_mm = 25.0, 18.0
    board = make_charuco(squares_x, squares_y, square_mm, marker_mm, dictionary_name)
    board_img = Image.fromarray(board).convert("RGB")

    # Two A4 sheets side by side and two sheets vertically. The usable board
    # region is centered on this larger canvas, then each A4 page is exported.
    page_w, page_h = mm_to_px(A4_MM[0]), mm_to_px(A4_MM[1])
    full_w, full_h = page_w * 2, page_h * 2
    full = Image.new("RGB", (full_w, full_h), "white")
    board_x = (full_w - board_img.width) // 2
    board_y = (full_h - board_img.height) // 2
    full.paste(board_img, (board_x, board_y))

    for row in range(2):
        for col in range(2):
            left = col * page_w
            top = row * page_h
            page = full.crop((left, top, left + page_w, top + page_h))
            crop_box = (
                max(0, board_x - left),
                max(0, board_y - top),
                min(page_w, board_x + board_img.width - left),
                min(page_h, board_y + board_img.height - top),
            )
            draw_page_guides(
                page,
                f"ChArUco tiled board {row + 1},{col + 1}",
                f"{squares_x}x{squares_y}, square={square_mm:.1f}mm, marker={marker_mm:.1f}mm, print 100%",
                crop_box,
            )
            save_pdf_and_png(page, out_dir / f"charuco_tiled_10x14_25mm_r{row + 1}_c{col + 1}")


def write_readme(out_dir: Path, dictionary_name: str) -> None:
    text = f"""# ChArUco Calibration Boards

Generated at 300 DPI for printing at 100% scale.

## Single A4 board

- File: `charuco_a4_7x10_25mm.pdf`
- Board: 7 x 10 squares
- Square length: 25.0 mm
- Marker length: 18.0 mm
- Dictionary: `{dictionary_name}`

## 2x2 tiled board

- Files: `charuco_tiled_10x14_25mm_r*_c*.pdf`
- Board: 10 x 14 squares
- Square length: 25.0 mm
- Marker length: 18.0 mm
- Dictionary: `{dictionary_name}`
- Print every page at 100% scale, trim along the red board boundary, and mount
  flat on a rigid board. Do not use printer page fitting or scaling.

For hand-eye calibration, the detector parameters must use the same square
length, marker length, and dictionary.
"""
    (out_dir / "README.md").write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        default="generated/calibration/charuco",
        help="Output directory.",
    )
    parser.add_argument(
        "--dictionary",
        default="DICT_5X5_1000",
        help="OpenCV ArUco dictionary constant name.",
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    generate_single_a4(out_dir, args.dictionary)
    generate_large_tiled(out_dir, args.dictionary)
    write_readme(out_dir, args.dictionary)
    print(f"Wrote ChArUco printable boards to {out_dir}")


if __name__ == "__main__":
    main()
