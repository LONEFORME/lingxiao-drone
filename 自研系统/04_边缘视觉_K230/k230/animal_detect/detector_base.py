"""
K230 动物检测基类 — 提取自 animal_detect_visual 和 animal_detect_yolov8n
提供公共的检测后处理逻辑，子类覆写特定行为（draw_result / get_frame_data）
"""

from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import nncase_runtime as nn
import ulab.numpy as np


class AnimalDetectBase(AIBase):
    """YOLOv8 动物检测基类：预处理 + 后处理 + 统计"""

    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                 confidence_threshold=0.5, nms_threshold=0.2,
                 rgb888p_size=[640, 480], display_size=[800, 480], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        self.rgb888p_size = [rgb888p_size[0], rgb888p_size[1]]
        self.display_size = [display_size[0], display_size[1]]
        self.debug_mode = debug_mode
        self.color_four = get_colors(len(self.labels))
        self.scale = 1.0
        # Ai2d 实例，用于实现模型预处理
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(
            nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
            np.uint8, np.uint8
        )

    @staticmethod
    def _parse_dets(dets):
        """单次遍历检测结果，返回 (counts, max_conf)"""
        counts = {}
        max_conf = {}
        if dets:
            for i in range(len(dets[0])):
                label_id = dets[1][i]
                score = float(dets[2][i])
                counts[label_id] = counts.get(label_id, 0) + 1
                if label_id not in max_conf or score > max_conf[label_id]:
                    max_conf[label_id] = score
        return counts, max_conf

    @staticmethod
    def _resolve_best(counts, max_conf):
        """多种类冲突时取最高置信度的类，返回 (best_id, show_counts)"""
        if not counts:
            return None, {}
        if len(counts) > 1:
            best_id = max(counts, key=lambda k: max_conf.get(k, 0))
            return best_id, {best_id: sum(counts.values())}
        best_id = next(iter(counts))
        return best_id, dict(counts)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size
            )
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build(
                [1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                [1, 3, self.model_input_size[1], self.model_input_size[0]],
            )

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            new_result = results[0][0].transpose()
            det_res = aidemo.yolov8_det_postprocess(
                new_result.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(self.labels),
                self.confidence_threshold,
                self.nms_threshold,
                self.max_boxes_num,
            )
            return det_res

    # === 子类需实现的抽象方法 ===
    def get_frame_data(self, dets):
        """active 模式下提取单帧检测数据（覆盖裁剪过滤等自定义逻辑）"""
        raise NotImplementedError

    def get_uart_data(self, dets):
        """获取 UART 发送用的统计数据（默认使用 _resolve_best）"""
        _, show_counts = self._resolve_best(*self._parse_dets(dets))
        return show_counts
