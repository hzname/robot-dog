"""ArUco marker detection and 3D marker positions from one camera."""

import json
import math

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - reported by the CLI
    cv2 = None

DICTIONARY = 'DICT_4X4_50'


def require_cv2():
    if cv2 is None or not hasattr(cv2, 'aruco'):
        raise RuntimeError('OpenCV with ArUco is missing: pip install opencv-contrib-python')


def dictionary():
    require_cv2()
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))


def marker_image(marker_id, pixels):
    d = dictionary()
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(d, marker_id, pixels)
    return cv2.aruco.drawMarker(d, marker_id, pixels)  # OpenCV < 4.7


class Intrinsics:
    def __init__(self, K, dist=None):
        self.K = np.asarray(K, dtype=float)
        self.dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=float)

    @classmethod
    def approximate(cls, width, height, hfov_deg=65.0):
        """Good enough when no chessboard calibration is available."""
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]])

    @classmethod
    def load(cls, path):
        with open(path) as fh:
            d = json.load(fh)
        return cls(d['K'], d.get('dist'))

    def save(self, path, rms=None):
        with open(path, 'w') as fh:
            json.dump({'K': self.K.tolist(), 'dist': self.dist.tolist(), 'rms_px': rms}, fh, indent=2)


class MarkerTracker:
    def __init__(self, intrinsics: Intrinsics, marker_mm: float):
        require_cv2()
        self.intr = intrinsics
        self.half = marker_mm / 2.0
        d = dictionary()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            det = cv2.aruco.ArucoDetector(d, params)
            self._detect = lambda img: det.detectMarkers(img)[:2]
        else:  # OpenCV < 4.7
            params = cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._detect = lambda img: cv2.aruco.detectMarkers(img, d, parameters=params)[:2]
        h = self.half
        self._obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)

    def detect(self, image):
        """{id: (xyz_mm, corners_px)} for every marker in the image."""
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        out = {}
        if ids is None:
            return out
        for c, i in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2).astype(np.float32)
            ok, _rvec, tvec = cv2.solvePnP(self._obj, pts, self.intr.K, self.intr.dist,
                                           flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                out[int(i)] = (tvec.reshape(3), pts)
        return out

    def positions(self, frames, min_fraction=0.5):
        """Median position [mm] of each marker over several frames."""
        seen = {}
        for img in frames:
            for i, (xyz, _) in self.detect(img).items():
                seen.setdefault(i, []).append(xyz)
        need = max(1, int(math.ceil(min_fraction * len(frames))))
        return {i: np.median(np.array(v), axis=0) for i, v in seen.items() if len(v) >= need}


class Camera:
    """cv2.VideoCapture wrapper: index (0, 1...) or URL (phone IP-camera)."""

    def __init__(self, source, width=1280, height=720):
        require_cv2()
        src = int(source) if str(source).isdigit() else source
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f'cannot open camera {source!r}')
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def frames(self, n):
        out = []
        for _ in range(n * 3):
            ok, img = self.cap.read()
            if ok:
                out.append(img)
            if len(out) >= n:
                break
        if not out:
            raise RuntimeError('camera returns no frames')
        return out

    def size(self):
        img = self.frames(1)[0]
        return img.shape[1], img.shape[0]


def calibrate_from_chessboard(images, cols=9, rows=6, square_mm=25.0):
    """Intrinsics from chessboard photos (inner corners cols x rows)."""
    require_cv2()
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    obj_pts, img_pts, size = [], [], None
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        size = gray.shape[::-1]
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if ok:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
            obj_pts.append(objp)
            img_pts.append(corners)
    if len(obj_pts) < 5:
        raise RuntimeError(f'chessboard found in {len(obj_pts)} photos, need at least 5')
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    return Intrinsics(K, dist.reshape(-1)), float(rms)
