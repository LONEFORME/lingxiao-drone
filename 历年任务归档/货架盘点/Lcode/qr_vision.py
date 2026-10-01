"""二维码解码、多帧确认与可开关的真实飞行图片留档。"""

import json
import math
import os
import threading
import time
from collections import Counter, deque
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import numpy as np

from Lcode.Logger import logger

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
except Exception:
    # ZBar is an optional fallback.  OpenCV remains usable if the native ZBar
    # library is unavailable on a flight board.
    pyzbar_decode = None


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class QRDetection:
    number: Optional[int]
    content: str
    corners: Tuple[Point2D, Point2D, Point2D, Point2D]

    @property
    def center(self) -> Point2D:
        return (
            sum(point[0] for point in self.corners) / 4.0,
            sum(point[1] for point in self.corners) / 4.0,
        )


class QRMapping:
    def __init__(self, mapping_file):
        self.path = Path(mapping_file)
        self.number_to_content = {}
        self.content_to_number = {}
        self.load()

    def load(self):
        if not self.path.exists():
            raise FileNotFoundError(f"二维码映射不存在: {self.path}")
        for line_number, line in enumerate(
            self.path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            if not line.strip():
                continue
            try:
                number_text, content = line.split("\t", 1)
                number = int(number_text)
            except ValueError as exc:
                raise ValueError(f"二维码映射第{line_number}行无效") from exc
            content = content.strip()
            if not 1 <= number <= 24 or not content:
                raise ValueError(f"二维码映射第{line_number}行超出范围")
            if number in self.number_to_content or content in self.content_to_number:
                raise ValueError("二维码映射包含重复编号或内容")
            self.number_to_content[number] = content
            self.content_to_number[content] = number
        if set(self.number_to_content) != set(range(1, 25)):
            raise ValueError("二维码映射必须完整包含1~24")


class QRDecoder:
    def __init__(
        self,
        mapping: QRMapping,
        detection_width=1280,
        detector=None,
        geometry_roi_width=1000,
        geometry_roi_height=600,
        geometry_upscale_scales=(2.0, 3.0, 4.0),
        decode_profile="variants",
    ):
        if cv2 is None and detector is None:
            raise RuntimeError("二维码识别需要OpenCV")
        self.mapping = mapping
        self.detection_width = int(detection_width)
        self.detector = detector or cv2.QRCodeDetector()
        # Visual servoing only needs the QR quadrilateral.  A narrow ROI around
        # the optical axis is substantially cheaper than upscaling the full
        # 1280x720 frame, and excludes most adjacent shelf labels.
        self.geometry_roi_width = max(160, int(geometry_roi_width))
        self.geometry_roi_height = max(160, int(geometry_roi_height))
        self.geometry_upscale_scales = tuple(
            float(scale)
            for scale in geometry_upscale_scales
            if float(scale) > 1.0
        )
        self.decode_profile = str(decode_profile).strip().lower()
        if self.decode_profile not in {"raw", "variants"}:
            raise ValueError("decode_profile只能是raw/variants")

    @staticmethod
    def _points_array(points):
        if points is None:
            return None
        try:
            array = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        except (TypeError, ValueError):
            return None
        return array if len(array) == 4 else None

    @staticmethod
    def _ordered_points(points):
        """Return QR corners in top-left, top-right, bottom-right, bottom-left order."""
        points = np.asarray(points, dtype=np.float32).reshape(4, 2)
        sums = points.sum(axis=1)
        diffs = np.diff(points, axis=1).reshape(-1)
        return np.array(
            [
                points[np.argmin(sums)],
                points[np.argmin(diffs)],
                points[np.argmax(sums)],
                points[np.argmax(diffs)],
            ],
            dtype=np.float32,
        )

    def _detection_frame(self, frame):
        """Keep the QR pixels large enough for decoding during flight."""
        height, width = frame.shape[:2]
        if self.detection_width <= 0 or width <= self.detection_width:
            return frame, 1.0
        scale = self.detection_width / width
        return (
            cv2.resize(
                frame,
                (self.detection_width, max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            ),
            scale,
        )

    def _decode_pyzbar_image(self, image, target_point=None):
        """Decode one image with ZBar, optionally selecting nearest target.

        pyzbar may return several shelf codes in an implementation-dependent
        order.  The first result is not necessarily the current cargo slot, so
        target-aware calls must select by polygon center rather than list order.
        """
        if pyzbar_decode is not None:
            try:
                results = pyzbar_decode(image)
            except Exception:
                results = ()
            decoded = []
            for result in results:
                raw_data = getattr(result, "data", b"")
                if isinstance(raw_data, bytes):
                    fallback_content = raw_data.decode("utf-8", errors="replace").strip()
                else:
                    fallback_content = str(raw_data or "").strip()
                if not fallback_content:
                    continue
                polygon = self._points_array(getattr(result, "polygon", None))
                if polygon is None:
                    rect = getattr(result, "rect", None)
                    if rect is not None:
                        x, y, width, height = (
                            float(rect.left),
                            float(rect.top),
                            float(rect.width),
                            float(rect.height),
                        )
                        polygon = np.asarray(
                            [
                                (x, y),
                                (x + width, y),
                                (x + width, y + height),
                                (x, y + height),
                            ],
                            dtype=np.float32,
                        )
                decoded.append((fallback_content, polygon))
            if decoded:
                if target_point is not None:
                    tx, ty = target_point
                    with_geometry = [item for item in decoded if item[1] is not None]
                    if with_geometry:
                        return min(
                            with_geometry,
                            key=lambda item: math.hypot(
                                float(np.asarray(item[1])[:, 0].mean()) - tx,
                                float(np.asarray(item[1])[:, 1].mean()) - ty,
                            ),
                        )
                return decoded[0]
        return "", None

    def _decode_image(self, image):
        """Decode one image, also trying an already-localized QR polygon."""
        # Use ZBar/pyzbar first for content decoding when it is available.
        # OpenCV remains the fallback because it also supplies geometry for
        # visual servoing on boards without the native ZBar library.
        content, polygon = self._decode_pyzbar_image(image)
        if content:
            return content, polygon

        content, points, _ = self.detector.detectAndDecode(image)
        content = str(content or "").strip()
        point_array = self._points_array(points)
        if not content and point_array is not None and hasattr(self.detector, "decode"):
            try:
                decoded, _ = self.detector.decode(
                    image, point_array.reshape(1, 4, 2)
                )
                content = str(decoded or "").strip()
            except Exception:
                pass
        return content, point_array

    def _decode_warped(self, image, points):
        """Perspective-normalize a localized code before retrying decode."""
        if cv2 is None:
            return ""
        ordered = self._ordered_points(points)
        side = max(
            256,
            min(
                640,
                int(
                    round(
                        max(
                            np.linalg.norm(ordered[1] - ordered[0]),
                            np.linalg.norm(ordered[2] - ordered[3]),
                            np.linalg.norm(ordered[2] - ordered[1]),
                            np.linalg.norm(ordered[3] - ordered[0]),
                        )
                        * 2.0
                    )
                ),
            ),
        )
        destination = np.array(
            [[0, 0], [side - 1, 0], [side - 1, side - 1], [0, side - 1]],
            dtype=np.float32,
        )
        transform = cv2.getPerspectiveTransform(ordered, destination)
        warped = cv2.warpPerspective(
            image,
            transform,
            (side, side),
            flags=cv2.INTER_CUBIC,
            borderMode=cv2.BORDER_REPLICATE,
        )
        border = max(16, side // 8)
        warped = cv2.copyMakeBorder(
            warped,
            border,
            border,
            border,
            border,
            cv2.BORDER_CONSTANT,
            value=(255, 255, 255),
        )
        gray = (
            warped
            if len(warped.shape) == 2
            else cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        )
        for variant in self._content_variants(warped):
            try:
                content, _, _ = self.detector.detectAndDecode(variant)
            except Exception:
                continue
            content = str(content or "").strip()
            if content:
                return content
        return ""

    @staticmethod
    def _variants(image):
        yield image
        if cv2 is not None and len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            yield gray
            yield cv2.createCLAHE(2.0, (8, 8)).apply(gray)

    @staticmethod
    def _content_variants(image):
        """Return bounded preprocessing variants for content decoding only."""
        yield image
        if cv2 is None:
            return
        gray = image if len(image.shape) == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        yield gray
        yield cv2.createCLAHE(2.0, (8, 8)).apply(gray)

        # Preserve finder patterns while restoring contrast around the smaller
        # data modules that are weakened by flight motion blur.
        blurred = cv2.GaussianBlur(gray, (0, 0), 1.0)
        yield cv2.addWeighted(gray, 1.6, blurred, -0.6, 0)
        yield cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )[1]
        yield cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            5,
        )

    def _candidate(self, content, points, scale=1.0, offset=(0, 0)):
        point_array = self._points_array(points)
        if point_array is None:
            return None
        ox, oy = offset
        raw_points = tuple(
            (float(x / scale + ox), float(y / scale + oy))
            for x, y in point_array
        )
        clean_content = str(content or "").strip()
        return QRDetection(
            self.mapping.content_to_number.get(clean_content),
            clean_content,
            raw_points,
        )

    @staticmethod
    def _valid_geometry_points(points, min_area=1200.0):
        """Reject detector artefacts such as duplicated/collinear corners."""
        point_array = QRDecoder._points_array(points)
        if point_array is None:
            return None
        ordered = QRDecoder._ordered_points(point_array)
        side_lengths = [
            float(np.linalg.norm(ordered[(index + 1) % 4] - ordered[index]))
            for index in range(4)
        ]
        if min(side_lengths) < 12.0:
            return None
        area = abs(float(cv2.contourArea(ordered))) if cv2 is not None else 0.0
        if area < float(min_area):
            return None
        if max(side_lengths) / min(side_lengths) > 8.0:
            return None
        if cv2 is not None and not cv2.isContourConvex(ordered):
            return None
        return ordered

    def _geometry_candidate(self, points, offset=(0, 0), scale=1.0):
        valid = self._valid_geometry_points(points)
        if valid is None:
            return None
        return self._candidate("", valid, scale, offset)

    def _geometry_candidates_from_image(self, image, offset=(0, 0), scale=1.0):
        """Find geometry in one image and map it back to full-frame pixels."""
        candidates = []
        for variant in self._variants(image):
            if hasattr(self.detector, "detect"):
                try:
                    detected, points = self.detector.detect(variant)
                except Exception:
                    detected, points = False, None
                if detected:
                    candidate = self._geometry_candidate(points, offset, scale)
                    if candidate is not None:
                        candidates.append(candidate)

            # Some older OpenCV builds expose only detectAndDecode. Geometry is
            # still useful when content is empty.
            if not candidates:
                try:
                    _, points, _ = self.detector.detectAndDecode(variant)
                except Exception:
                    points = None
                candidate = self._geometry_candidate(points, offset, scale)
                if candidate is not None:
                    candidates.append(candidate)

            if candidates:
                break
        return candidates

    def _fast_geometry_search(self, frame, target_point=None):
        """Find a plausible QR quadrilateral without a full-frame upscale."""
        height, width = frame.shape[:2]
        roi_width = min(width, self.geometry_roi_width)
        roi_height = min(height, self.geometry_roi_height)
        # Keep the optical axis in the middle of the ROI. The previous
        # downward bias admitted more guard-rail pixels and could hide the
        # upper QR behind a cluttered lower edge.
        roi_center_x = width * 0.50
        roi_center_y = height * 0.50
        ox = max(0, min(width - roi_width, round(roi_center_x - roi_width / 2)))
        oy = max(0, min(height - roi_height, round(roi_center_y - roi_height / 2)))
        roi = frame[oy : oy + roi_height, ox : ox + roi_width]
        candidates = self._geometry_candidates_from_image(roi, (ox, oy))

        # The airborne QR is only about 120-140 px wide in the 1280x720
        # camera frame. Native-resolution detection is intermittent even when
        # the code remains visibly centered. Upscale only this small ROI so
        # the board does not pay the cost of a full-frame 2x/3x pass.
        if not candidates and cv2 is not None:
            for scale in self.geometry_upscale_scales:
                scaled_roi = cv2.resize(
                    roi,
                    (round(roi_width * scale), round(roi_height * scale)),
                    interpolation=cv2.INTER_CUBIC,
                )
                candidates = self._geometry_candidates_from_image(
                    scaled_roi, (ox, oy), scale
                )
                if candidates:
                    break

        if not candidates:
            # One native-resolution full-frame attempt is a cheap fallback. Do
            # not return to the old 2x full-frame detectMulti path here: that
            # path caused the board-side servo timeout after a single frame.
            if hasattr(self.detector, "detect"):
                try:
                    detected, points = self.detector.detect(frame)
                except Exception:
                    detected, points = False, None
                if detected:
                    candidate = self._geometry_candidate(points)
                    if candidate is not None:
                        candidates.append(candidate)

        return self._select_candidate(candidates, frame.shape, target_point)

    @staticmethod
    def _select_candidate(candidates, frame_shape, target_point=None):
        if not candidates:
            return None
        height, width = frame_shape[:2]
        center = target_point or (width / 2.0, height / 2.0)
        return min(
            candidates,
            key=lambda item: math.hypot(
                item.center[0] - center[0], item.center[1] - center[1]
            ),
        )

    def _target_roi(self, frame, target_point=None):
        """Return a bounded ROI around the laser/optical target."""
        height, width = frame.shape[:2]
        roi_width = min(width, max(320, self.geometry_roi_width))
        roi_height = min(height, max(320, self.geometry_roi_height))
        if target_point is None:
            center_x, center_y = width / 2.0, height / 2.0
        else:
            center_x, center_y = target_point
        ox = max(0, min(width - roi_width, round(center_x - roi_width / 2.0)))
        oy = max(0, min(height - roi_height, round(center_y - roi_height / 2.0)))
        return frame[oy : oy + roi_height, ox : ox + roi_width], (ox, oy)

    def _decode_target_roi(self, frame, target_point):
        """Fast pyzbar-only decode for the airborne target region.

        The full geometry search is useful for visual servoing but is too
        expensive to run once per airborne frame on the board.  QR content is
        decoded here from a bounded ROI, with only a few cheap image variants;
        the returned polygon is mapped back to full-frame coordinates.
        """
        roi, offset = self._target_roi(frame, target_point)
        variants = [(roi, 1.0)]
        if self.decode_profile == "variants" and cv2 is not None:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            variants.extend(((gray, 1.0), (cv2.createCLAHE(2.0, (8, 8)).apply(gray), 1.0)))
            enlarged = cv2.resize(
                roi,
                (round(roi.shape[1] * 2.0), round(roi.shape[0] * 2.0)),
                interpolation=cv2.INTER_CUBIC,
            )
            variants.append((enlarged, 2.0))

            # Airborne frames can contain enough motion/illumination
            # variation that the QR is visible to a phone but the raw UVC
            # image is not decodable by ZBar.  Keep the fast ROI path, but
            # add the cheap adaptive-threshold variants that recover the
            # small modules without returning to the expensive full-frame
            # multi-pass search.
            for scale in (1.0, 2.0, 3.0):
                if scale == 1.0:
                    scaled_gray = gray
                else:
                    scaled_gray = cv2.resize(
                        gray,
                        (round(roi.shape[1] * scale), round(roi.shape[0] * scale)),
                        interpolation=cv2.INTER_CUBIC,
                    )
                variants.append(
                    (
                        cv2.adaptiveThreshold(
                            scaled_gray,
                            255,
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                            cv2.THRESH_BINARY,
                            31,
                            5,
                        ),
                        scale,
                    )
                )
        for image, scale in variants:
            local_target = (
                (float(target_point[0]) - offset[0]) * scale,
                (float(target_point[1]) - offset[1]) * scale,
            )
            content, points = self._decode_pyzbar_image(
                image,
                target_point=local_target,
            )
            if not content:
                continue
            candidate = self._candidate(content, points, scale, offset)
            if candidate is not None and candidate.number is not None:
                return candidate

        return None

    def _decode_search(self, frame, target_point=None):
        """Search full resolution first, then overlapping tiles if needed."""
        detection_frame, scale = self._detection_frame(frame)
        candidates = []

        for variant in self._variants(detection_frame):
            content, points = self._decode_image(variant)
            if points is not None:
                if not content:
                    content = self._decode_warped(variant, points)
                candidate = self._candidate(content, points, scale)
                if candidate is not None:
                    candidates.append(candidate)

            if hasattr(self.detector, "detectAndDecodeMulti"):
                try:
                    ok, contents, multi_points, _ = self.detector.detectAndDecodeMulti(
                        variant
                    )
                except Exception:
                    ok, contents, multi_points = False, [], None
                if ok and multi_points is not None:
                    for index, multi_point in enumerate(
                        np.asarray(multi_points).reshape(-1, 4, 2)
                    ):
                        content = contents[index] if index < len(contents) else ""
                        candidate = self._candidate(content, multi_point, scale)
                        if candidate is not None:
                            candidates.append(candidate)

        known = [item for item in candidates if item.number is not None]
        if known:
            return self._select_candidate(known, frame.shape, target_point)

        # A sliding-window pass gives each QR code its own quiet zone when the
        # frame contains two or more codes, a rail, or a cable.  Large tiles
        # still contain too much background for the OpenCV detector; half-
        # overlapping square windows are deliberate here.
        height, width = frame.shape[:2]
        window_size = min(
            min(width, height),
            max(260, min(420, int(min(width, height) * 0.5))),
        )
        stride = max(1, window_size // 2)
        x_starts = list(range(0, max(1, width - window_size + 1), stride))
        y_starts = list(range(0, max(1, height - window_size + 1), stride))
        if not x_starts or x_starts[-1] != max(0, width - window_size):
            x_starts.append(max(0, width - window_size))
        if not y_starts or y_starts[-1] != max(0, height - window_size):
            y_starts.append(max(0, height - window_size))
        for oy in dict.fromkeys(y_starts):
            for ox in dict.fromkeys(x_starts):
                tile = frame[oy : oy + window_size, ox : ox + window_size]
                content, points = self._decode_image(tile)
                if points is not None and not content:
                    content = self._decode_warped(tile, points)
                candidate = self._candidate(content, points, 1.0, (ox, oy))
                if candidate is not None:
                    candidates.append(candidate)

        known = [item for item in candidates if item.number is not None]
        return self._select_candidate(known or candidates, frame.shape, target_point)

    def detect_geometry(self, frame, decode_content=True) -> Optional[QRDetection]:
        """Return the selected QR geometry even when decoding is empty.

        OpenCV can localize a QR quadrilateral before it can decode its text.
        The geometry is safe to use for a bounded centering phase, but it is
        never accepted by QRConsensus unless ``number`` is known.
        """
        if not decode_content:
            return self._fast_geometry_search(frame)

        height, width = frame.shape[:2]
        scale = 1.0
        detection_frame = frame
        if self.detection_width > 0 and width > self.detection_width:
            scale = self.detection_width / width
            detection_frame = cv2.resize(
                frame,
                (self.detection_width, max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        content, points, _ = self.detector.detectAndDecode(detection_frame)
        if points is None and hasattr(self.detector, "detect"):
            # A first decode attempt can be slow or return no text/points even
            # though OpenCV can already localize the QR quadrilateral.
            detected, detected_points = self.detector.detect(detection_frame)
            if detected:
                points = detected_points
        if points is None and hasattr(self.detector, "detectAndDecodeMulti"):
            # The real flight frames contain several small QR codes and a
            # guard rail.  Upscaling before the multi-code geometry pass is
            # materially more reliable than running the detector on the
            # 800-pixel decode frame.  Geometry selection is nearest-to-
            # image-center only; content acceptance remains unchanged.
            geometry_scale = 2.0
            geometry_frame = cv2.resize(
                frame,
                (round(width * geometry_scale), round(height * geometry_scale)),
                interpolation=cv2.INTER_CUBIC,
            )
            multi_ok, multi_content, multi_points, _ = self.detector.detectAndDecodeMulti(
                geometry_frame
            )
            if multi_ok and multi_points is not None and len(multi_points):
                image_center = (width * geometry_scale / 2.0, height * geometry_scale / 2.0)
                candidates = multi_points.reshape(-1, 4, 2)
                selected_index = min(
                    range(len(candidates)),
                    key=lambda index: math.hypot(
                        float(candidates[index, :, 0].mean()) - image_center[0],
                        float(candidates[index, :, 1].mean()) - image_center[1],
                    ),
                )
                points = candidates[selected_index]
                content = (
                    str(multi_content[selected_index]).strip()
                    if multi_content is not None and selected_index < len(multi_content)
                    else ""
                )
                scale = geometry_scale
        if points is None:
            return None
        raw_points = points.reshape(-1, 2)
        if len(raw_points) != 4:
            return None
        corners = tuple((float(x / scale), float(y / scale)) for x, y in raw_points)
        clean_content = content.strip()
        number = self.mapping.content_to_number.get(clean_content)
        return QRDetection(number, clean_content, corners)

    def detect(self, frame, target_point=None) -> Optional[QRDetection]:
        if target_point is not None:
            # Airborne VERIFY_QR is latency-bounded.  Only use the pyzbar ROI
            # variants here; OpenCV geometry/localized fallback can take many
            # seconds on the board and belongs to offline target_point=None use.
            return self._decode_target_roi(frame, target_point)

        # target_point is None: full search without airborne latency constraint.
        # First localize the QR near the optical axis. The same localized
        # polygon is then used to decode a padded, enlarged crop; this keeps
        # adjacent shelf codes and the guard rail out of the decoder input.
        geometry = self._fast_geometry_search(frame, target_point)
        if geometry is not None:
            content = self._decode_localized(frame, geometry.corners)
            if content:
                return QRDetection(
                    self.mapping.content_to_number.get(content),
                    content,
                    geometry.corners,
                )
            # A localized-but-undecoded target is safer to report as no
            # content than to fall back to a different QR elsewhere in the
            # shelf. The consensus layer will use the next airborne frame.
            return None

        detection = self._decode_search(frame, target_point)
        if detection is None or detection.number is None:
            return None
        return detection

    def _decode_localized(self, frame, corners):
        """Decode a padded, enlarged crop around one selected QR polygon."""
        points = np.asarray(corners, dtype=np.float32)
        x_min, y_min = points.min(axis=0)
        x_max, y_max = points.max(axis=0)
        width = max(1.0, x_max - x_min)
        height = max(1.0, y_max - y_min)
        margin_x = max(24.0, width * 0.35)
        margin_y = max(24.0, height * 0.35)
        left = max(0, int(math.floor(x_min - margin_x)))
        top = max(0, int(math.floor(y_min - margin_y)))
        right = min(frame.shape[1], int(math.ceil(x_max + margin_x)))
        bottom = min(frame.shape[0], int(math.ceil(y_max + margin_y)))
        crop = frame[top:bottom, left:right]
        if crop.size == 0:
            return ""

        local_points = np.asarray(
            [((x - left), (y - top)) for x, y in corners],
            dtype=np.float32,
        )
        for scale in (2.0, 3.0):
            if cv2 is None:
                variants = (crop,)
                enlarged = crop
            else:
                enlarged = cv2.resize(
                    crop,
                    (round(crop.shape[1] * scale), round(crop.shape[0] * scale)),
                    interpolation=cv2.INTER_LANCZOS4,
                )
                variants = self._content_variants(enlarged)
            for variant in variants:
                content, _ = self._decode_image(variant)
                content = str(content or "").strip()
                if content in self.mapping.content_to_number:
                    return content

            # Use the already selected quadrilateral for the perspective retry;
            # do not let an enhancement variant switch to another shelf QR.
            warped_content = self._decode_warped(
                enlarged,
                local_points * scale if cv2 is not None else local_points,
            )
            if warped_content in self.mapping.content_to_number:
                return warped_content
        return ""


def _distance_to_line(point: Point2D, a: Point2D, b: Point2D) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length == 0:
        return 0.0
    return abs(dy * point[0] - dx * point[1] + b[0] * a[1] - b[1] * a[0]) / length


def point_inside_qr(corners: Sequence[Point2D], point: Point2D, margin_px=0.0) -> bool:
    if len(corners) != 4:
        return False
    signs = []
    for index, a in enumerate(corners):
        b = corners[(index + 1) % len(corners)]
        cross = (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])
        signs.append(cross)
        if _distance_to_line(point, a, b) < margin_px:
            return False
    return all(value >= 0 for value in signs) or all(value <= 0 for value in signs)


@dataclass(frozen=True)
class QRConsensusConfig:
    window_size: int = 2
    required_count: int = 1
    laser_margin_px: float = 12.0
    require_laser_inside: bool = False

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        defaults = cls()  # 用类默认值作为回退
        enabled = env.get("DRONE_QR_REQUIRE_LASER_INSIDE").strip().lower() if "DRONE_QR_REQUIRE_LASER_INSIDE" in env else ("1" if defaults.require_laser_inside else "0")
        if enabled not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_QR_REQUIRE_LASER_INSIDE must be 0/1/false/true")
        return cls(
            window_size=int(env.get("DRONE_QR_CONSENSUS_WINDOW", str(defaults.window_size))),
            required_count=int(env.get("DRONE_QR_REQUIRED_COUNT", str(defaults.required_count))),
            laser_margin_px=float(env.get("DRONE_QR_LASER_MARGIN_PX", str(defaults.laser_margin_px))),
            require_laser_inside=enabled in {"1", "true"},
        )

    def __post_init__(self):
        if not 1 <= self.required_count <= self.window_size <= 30:
            raise ValueError("二维码多帧确认参数无效")
        if self.laser_margin_px < 0:
            raise ValueError("激光像素安全边距不能为负")


class QRConsensus:
    def __init__(self, config: QRConsensusConfig = None):
        self.config = config or QRConsensusConfig()
        self._window = deque(maxlen=self.config.window_size)

    def reset(self):
        self._window.clear()

    def update(self, detection: Optional[QRDetection], laser_aim_px: Point2D):
        valid = (
            detection
            if detection is not None
            and detection.number is not None
            and (
                not self.config.require_laser_inside
                or point_inside_qr(
                    detection.corners, laser_aim_px, self.config.laser_margin_px
                )
            )
            else None
        )
        self._window.append(valid)
        counts = Counter(item.number for item in self._window if item is not None)
        if not counts:
            return None
        number, count = counts.most_common(1)[0]
        if count < self.config.required_count:
            return None
        return next(item for item in reversed(self._window) if item and item.number == number)


@dataclass(frozen=True)
class VisionDebugConfig:
    enabled: bool = False
    transit_interval_s: float = 1.0
    transit_interval_m: float = 0.25
    scan_interval_s: float = 0.5

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        enabled = env.get("DRONE_VISION_DEBUG_CAPTURE", "0").strip().lower()
        if enabled not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_VISION_DEBUG_CAPTURE只能是0/1/false/true")
        return cls(
            enabled=enabled in {"1", "true"},
            transit_interval_s=float(env.get("DRONE_VISION_CAPTURE_INTERVAL_S", "1.0")),
            transit_interval_m=float(env.get("DRONE_VISION_CAPTURE_INTERVAL_M", "0.25")),
            scan_interval_s=float(env.get("DRONE_VISION_SCAN_CAPTURE_INTERVAL_S", "0.5")),
        )


class VisionDebugCapture:
    def __init__(self, directory, config: VisionDebugConfig = None, image_writer=None):
        # 每次启动自动创建时间戳子目录，避免多次飞行图片混在一起
        base = Path(directory)
        session_tag = time.strftime("%Y%m%d_%H%M%S")
        self.directory = base / session_tag
        self.config = config or VisionDebugConfig.from_env()
        self._image_writer = image_writer or (cv2.imwrite if cv2 is not None else None)
        self._last_time = None
        self._last_position = None
        self._last_scan_time = None
        self._counter = 0
        self._lock = threading.Lock()

    def _save(self, frame, prefix, metadata) -> Optional[Path]:
        if not self.config.enabled or self._image_writer is None:
            return None
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            with self._lock:
                self._counter += 1
                slot = metadata.get("slot_label", "") if metadata else ""
                slot = slot.replace("/", "_").replace(" ", "")
                stem = f"{slot}_{int(time.time() * 1000)}_{self._counter:05d}_{prefix}" if slot else f"{int(time.time() * 1000)}_{self._counter:05d}_{prefix}"
            image_path = self.directory / f"{stem}.jpg"
            if not self._image_writer(str(image_path), frame):
                raise RuntimeError("图像编码失败")
            metadata_path = self.directory / f"{stem}.json"
            metadata_path.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            return image_path
        except Exception as exc:
            logger.error(f"视觉调试图片保存失败，飞行继续: {exc}")
            return None

    def capture_fixed(self, frame, metadata) -> Optional[Path]:
        return self._save(frame, "fixed", metadata)

    def capture_scan(self, frame, metadata, now=None) -> Optional[Path]:
        """Save only frames processed while the airborne vision loop is active."""
        if not self.config.enabled:
            return None
        now = time.monotonic() if now is None else float(now)
        if (
            self._last_scan_time is not None
            and now - self._last_scan_time < self.config.scan_interval_s
        ):
            return None
        path = self._save(frame, "scan", metadata)
        if path is not None:
            self._last_scan_time = now
        return path

    def maybe_capture_transit(self, frame, position_xy, metadata, now=None) -> Optional[Path]:
        if not self.config.enabled:
            return None
        now = time.monotonic() if now is None else float(now)
        position = (float(position_xy[0]), float(position_xy[1]))
        time_due = self._last_time is None or now - self._last_time >= self.config.transit_interval_s
        distance_due = (
            self._last_position is None
            or math.hypot(position[0] - self._last_position[0], position[1] - self._last_position[1])
            >= self.config.transit_interval_m
        )
        if not (time_due or distance_due):
            return None
        path = self._save(frame, "transit", metadata)
        if path is not None:
            self._last_time = now
            self._last_position = position
        return path
