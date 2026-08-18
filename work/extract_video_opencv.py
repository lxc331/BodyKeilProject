from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def write_image(path: Path, image: np.ndarray) -> None:
    suffix = path.suffix.lower()
    extension = ".jpg" if suffix in {".jpg", ".jpeg"} else suffix
    ok, encoded = cv2.imencode(extension, image)
    if not ok:
        raise RuntimeError(f"Could not encode {path}")
    path.write_bytes(encoded.tobytes())


def stamp(frame: np.ndarray, seconds: float) -> np.ndarray:
    image = frame.copy()
    minutes = int(seconds // 60)
    remainder = seconds - minutes * 60
    label = f"{minutes:02d}:{remainder:05.2f}"
    cv2.rectangle(image, (0, 0), (180, 38), (0, 0, 0), -1)
    cv2.putText(
        image,
        label,
        (10, 27),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.78,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--step", type=float, default=20.0)
    parser.add_argument("--times", type=float, nargs="*")
    parser.add_argument("--columns", type=int, default=2)
    parser.add_argument("--rows", type=int, default=3)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open {args.video}")

    fps = capture.get(cv2.CAP_PROP_FPS)
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = frame_count / fps if fps else 0.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if args.times:
        times = args.times
    else:
        times = []
        t = 0.0
        while t < duration:
            times.append(t)
            t += args.step
        if duration and (not times or duration - times[-1] > 1.0):
            times.append(max(0.0, duration - 0.2))

    frames: list[np.ndarray] = []
    written: list[dict[str, float | str]] = []
    for requested_time in times:
        safe_time = min(max(requested_time, 0.0), max(duration - 0.05, 0.0))
        capture.set(cv2.CAP_PROP_POS_MSEC, safe_time * 1000.0)
        ok, frame = capture.read()
        if not ok:
            continue
        actual_time = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        shown_time = actual_time if actual_time > 0.0 else safe_time
        frame = stamp(frame, shown_time)
        filename = f"frame-{int(round(safe_time * 1000)):07d}.png"
        write_image(args.output / filename, frame)
        frames.append(frame)
        written.append(
            {
                "requested_seconds": safe_time,
                "reported_seconds": actual_time,
                "file": filename,
            }
        )

    page_size = args.columns * args.rows
    for page_start in range(0, len(frames), page_size):
        page_frames = frames[page_start : page_start + page_size]
        while len(page_frames) < page_size:
            page_frames.append(np.zeros((height, width, 3), dtype=np.uint8))
        rows = []
        for row_start in range(0, page_size, args.columns):
            rows.append(cv2.hconcat(page_frames[row_start : row_start + args.columns]))
        contact_sheet = cv2.vconcat(rows)
        page_number = page_start // page_size + 1
        write_image(args.output / f"contact-{page_number:02d}.jpg", contact_sheet)

    capture.release()
    print(
        json.dumps(
            {
                "video": str(args.video),
                "duration_seconds": duration,
                "fps": fps,
                "frame_count": frame_count,
                "width": width,
                "height": height,
                "written": written,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
