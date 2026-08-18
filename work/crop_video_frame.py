from pathlib import Path
import sys

import cv2
import numpy as np


source = Path(sys.argv[1])
output = Path(sys.argv[2])
x, y, width, height = (int(value) for value in sys.argv[3:7])
scale = int(sys.argv[7])

image = cv2.imdecode(np.frombuffer(source.read_bytes(), dtype="uint8"), cv2.IMREAD_COLOR)
crop = image[y : y + height, x : x + width]
crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
ok, encoded = cv2.imencode(output.suffix, crop)
if not ok:
    raise RuntimeError(f"Could not encode {output}")
output.write_bytes(encoded.tobytes())
