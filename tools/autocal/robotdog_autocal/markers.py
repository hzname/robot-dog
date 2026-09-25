"""Printable ArUco marker sheet (A4, 300 dpi) with cut guides and centre marks."""

import numpy as np

from . import geometry as geo
from .vision import cv2, marker_image, require_cv2

LABELS = {
    0: 'LF thigh axis', 1: 'LF knee axis', 2: 'LF foot', 3: 'LR thigh axis', 4: 'LR knee axis', 5: 'LR foot',
    10: 'RF thigh axis', 11: 'RF knee axis', 12: 'RF foot', 13: 'RR thigh axis', 14: 'RR knee axis',
    15: 'RR foot', 20: 'LF hip axis (front)', 21: 'LF foot (front)', 22: 'RF hip axis (front)',
    23: 'RF foot (front)', 24: 'LR hip axis (rear)', 25: 'LR foot (rear)', 26: 'RR hip axis (rear)',
    27: 'RR foot (rear)',
}
DPI = 300
A4_MM = (210, 297)


def sheet(marker_mm=25.0, ids=None):
    """Grayscale image of an A4 page with all calibration markers."""
    require_cv2()
    ids = ids or sorted(LABELS)
    px_per_mm = DPI / 25.4
    W, H = int(A4_MM[0] * px_per_mm), int(A4_MM[1] * px_per_mm)
    page = np.full((H, W), 255, np.uint8)
    m = int(round(marker_mm * px_per_mm))
    cell_w, cell_h = m + int(18 * px_per_mm), m + int(16 * px_per_mm)
    cols = max(1, (W - int(20 * px_per_mm)) // cell_w)
    x0, y0 = int(10 * px_per_mm), int(22 * px_per_mm)
    cv2.putText(page, f'RobotDog 2.0 calibration markers - black square = {marker_mm:g} mm. '
                'Check with a ruler after printing (scale 100%).', (x0, int(12 * px_per_mm)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2, cv2.LINE_AA)
    for k, i in enumerate(ids):
        r, c = divmod(k, cols)
        x = x0 + c * cell_w + int(4 * px_per_mm)
        y = y0 + r * cell_h
        if y + cell_h > H:
            break
        page[y:y + m, x:x + m] = cv2.resize(marker_image(i, m), (m, m), interpolation=cv2.INTER_NEAREST)
        # centre ticks outside the white quiet zone: put these on the joint axis
        cx, cy, t = x + m // 2, y + m // 2, int(3 * px_per_mm)
        q = int(2.5 * px_per_mm)
        for (a, b) in (((cx, y - q - t), (cx, y - q)), ((cx, y + m + q), (cx, y + m + q + t)),
                       ((x - q - t, cy), (x - q, cy)), ((x + m + q, cy), (x + m + q + t, cy))):
            cv2.line(page, a, b, 0, 2)
        cv2.rectangle(page, (x - q - t, y - q - t), (x + m + q + t, y + m + q + t), 190, 1)
        cv2.putText(page, f'{i}: {LABELS[i]}', (x - q - t, y + m + q + t + int(4.5 * px_per_mm)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, 0, 2, cv2.LINE_AA)
    return page


def views_help():
    lines = []
    for view, v in geo.VIEWS.items():
        ids = [i for ids in v['legs'].values() for i in ids]
        lines.append(f'  {view:5s}: ' + ', '.join(f'{i} {LABELS[i]}' for i in ids))
    return '\n'.join(lines)
