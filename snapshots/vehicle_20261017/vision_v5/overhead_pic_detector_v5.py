#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
针对低算力工控机 (Intel Core i5-3337U 2核4线程, CPU已占用70%+) 的超低CPU占用版识别程序 (v5)
全场景 4 大类 x 2 状态 (共 8 类) 巡检贴图识别与判别:
1. 火源 (FIRE):
   - FIRE_NORMAL (正常火源: 普通小火苗/蜡烛/打火机/燃气蓝火/受控小篝火)
   - FIRE_ABNORMAL (异常火源: 明显大火/工业喷火爆炸/大面积燃烧/浓烟火災)
2. 水源 (WATER):
   - WATER_NORMAL (正常水源: 已确认水体，且呈现清澈水体的可见特征)
   - WATER_ABNORMAL (异常水源: 已确认水体，且呈现浑浊水体的可见特征)
   - 水体或清浊证据不足时输出UNKNOWN，不按水面大小判断异常
3. 行人 (PEDESTRIAN):
   - PEDESTRIAN_NORMAL (正常行人: 直立行人，前景人体团块主轴竖直、高宽比 h/w >= 1.22)
   - PEDESTRIAN_ABNORMAL (异常行人: 倒下行人，常在道路上，前景人体团块呈横向/斜卧姿态 h/w < 1.20)
4. 道路 (ROAD):
   - ROAD_NORMAL (正常道路: 空旷平整路面，含纯路面或正常白色/黄色车道线)
   - ROAD_ABNORMAL (异常道路: 地面存在暗色分叉裂纹/龟裂网格，通过形态学黑帽变换 MORPH_BLACKHAT 提取)
"""

from __future__ import print_function, division
import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"

import sys
import time
import argparse
import random
import json
import cv2
import numpy as np

if hasattr(cv2, 'setNumThreads'):
    cv2.setNumThreads(1)


# ================= OpenCV 2.4 / 3.x / 4.x 兼容辅助函数 =================
def find_contours_compat(binary_img, mode, method):
    res = cv2.findContours(binary_img.copy(), mode, method)
    if len(res) == 2:
        return res[0], res[1]
    return res[1], res[2]


def box_points_compat(rect):
    if hasattr(cv2, 'boxPoints'):
        pts = cv2.boxPoints(rect)
    else:
        pts = cv2.cv.BoxPoints(rect)
    return np.int32(pts)


def set_cam_props_compat(cap, width, height, exposure=None, fps=None):
    prop_w = getattr(cv2, 'CAP_PROP_FRAME_WIDTH', 3)
    prop_h = getattr(cv2, 'CAP_PROP_FRAME_HEIGHT', 4)
    prop_buf = getattr(cv2, 'CAP_PROP_BUFFERSIZE', 38)
    cap.set(prop_w, width)
    cap.set(prop_h, height)
    if fps is not None:cap.set(getattr(cv2, 'CAP_PROP_FPS', 5), float(fps))
    cap.set(prop_buf, 1)
    if exposure is not None:
        prop_ae = getattr(cv2, 'CAP_PROP_AUTO_EXPOSURE', 21)
        prop_exp = getattr(cv2, 'CAP_PROP_EXPOSURE', 15)
        cap.set(prop_ae, 1)
        cap.set(prop_exp, float(exposure))


def order_quad_points(pts, prev_poly=None):
    """
    将任意凸四边形的 4 个顶点严格按 [左上, 右上, 右下, 左下] 顺时针排序。
    支持传入 prev_poly 锚定上一帧角点顺序，消除 45° 临界角附近的 90° 跳变。
    """
    pts = np.array(pts, dtype=np.float32).reshape(4, 2)
    cx = float(np.mean(pts[:, 0]))
    cy = float(np.mean(pts[:, 1]))

    angles = np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx)
    cw_idx = np.argsort(angles)
    pts_cw = pts[cw_idx]

    signed_area = 0.0
    for i in range(4):
        x1, y1 = pts_cw[i]
        x2, y2 = pts_cw[(i + 1) % 4]
        signed_area += (x1 * y2 - x2 * y1)
    if signed_area < 0:
        pts_cw = pts_cw[::-1]

    if prev_poly is not None:
        prev = np.array(prev_poly, dtype=np.float32).reshape(4, 2)
        best_shift = 0
        min_dist_sq = float('inf')
        for shift in range(4):
            rolled = np.roll(pts_cw, -shift, axis=0)
            dist_sq = float(np.sum((rolled - prev) ** 2))
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                best_shift = shift
        return np.roll(pts_cw, -best_shift, axis=0).astype(np.float32)

    scores = pts_cw[:, 0] + pts_cw[:, 1] + 1e-3 * pts_cw[:, 1]
    start_idx = int(np.argmin(scores))
    ordered = np.roll(pts_cw, -start_idx, axis=0)
    return ordered.astype(np.float32)


def fit_quad_to_hull(hull):
    """从凸包轮廓拟合最佳 4 顶点四边形并支持三定一缺角几何补全"""
    peri = cv2.arcLength(hull, True)
    if peri <= 1.0:
        return None, 0.0

    hull_area = max(float(cv2.contourArea(hull)), 1.0)
    best_quad = None
    best_fit = 0.0

    for eps_factor in (0.018, 0.025, 0.035, 0.045, 0.06, 0.08, 0.10):
        approx = cv2.approxPolyDP(hull, eps_factor * peri, True)
        if len(approx) == 4:
            q_area = float(cv2.contourArea(approx))
            fit = min(hull_area, q_area) / max(hull_area, q_area)
            if fit > best_fit:
                best_fit = fit
                best_quad = approx.reshape(4, 2)
            break
        elif len(approx) < 4:
            break

    if best_quad is not None:
        ordered_q = order_quad_points(best_quad)
        min_rect = cv2.minAreaRect(hull)
        rect_pts = order_quad_points(box_points_compat(min_rect))
        char_len = max(float(min_rect[1][0]), float(min_rect[1][1]), 1.0)
        dists = np.linalg.norm(ordered_q - rect_pts, axis=1)
        close_mask = dists < (0.14 * char_len)
        if int(np.sum(close_mask)) == 3:
            bad_idx = int(np.argmin(close_mask))
            p_prev = ordered_q[(bad_idx - 1) % 4]
            p_next = ordered_q[(bad_idx + 1) % 4]
            p_opp = ordered_q[(bad_idx + 2) % 4]
            p_para = p_prev + p_next - p_opp
            ordered_q[bad_idx] = 0.55 * p_para + 0.45 * rect_pts[bad_idx]
        return ordered_q, best_fit

    min_rect = cv2.minAreaRect(hull)
    rect_pts = box_points_compat(min_rect)
    rect_area = max(float(min_rect[1][0] * min_rect[1][1]), 1.0)
    fallback_fit = (hull_area / rect_area) * 0.88
    return order_quad_points(rect_pts), fallback_fit


def compute_bbox_iou(box1, box2):
    """计算两个正外接矩形 (x, y, w, h) 的交并比 IoU"""
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2
    ix1 = max(x1, x2)
    iy1 = max(y1, y2)
    ix2 = min(x1 + w1, x2 + w2)
    iy2 = min(y1 + h1, y2 + h2)
    iw = max(0.0, float(ix2 - ix1))
    ih = max(0.0, float(iy2 - iy1))
    inter = iw * ih
    area1 = max(float(w1 * h1), 1.0)
    area2 = max(float(w2 * h2), 1.0)
    union = area1 + area2 - inter
    return inter / max(union, 1.0)


# ================= 全场景 8 类巡检内容判别器 (火源 / 水源 / 行人 / 道路) =================
class MultiSceneClassifier(object):
    """
    通用多维物理与几何特征提取分类器 (融合闭集 4 大类 x 2 状态先验优化与单次梯度共享)
    针对透视拉正后的 160x160 贴图，单次计算全图共享特征图并在 ~1.5ms 内完成 8 类场景判别:
    1. 闭集先验优化 ("只有火源、水源、行人、道路四种情况"):
       - 单次预计算全图 Sobel 梯度场 (gx_full, gy_full) 与色彩通道，供水源/行人/道路共享，消除 75% 重复卷积开销
       - 水源先用水面/喷流结构确认类别，再用水体区域色彩与细节判断清澈/浑浊
       - 行人内部融合【显著性加权协方差比 Sy/Sx】+【纵向归一化跨度 std_y/ih】+【单列高度/单行宽度比 CR】三项几何不变量
       - 道路内部利用顶帽变换 (MORPH_TOPHAT) 抑制高亮白/黄标线边缘伪影，再用黑帽变换 (MORPH_BLACKHAT) 纯净提取暗色裂缝
    """
    def __init__(self, config=None):
        self.config = dict(config or {})
        defaults = {
            'fire_dim_min_value': 60,
            'fire_dim_min_ratio': 0.0004,
            'fire_dim_max_ratio': 0.12,
            'fire_dim_min_local_contrast': 12,
            'road_min_surface_ratio': 0.55,
            'road_max_saturation': 70,
            'road_min_texture_ratio': 0.08,
            'water_min_area_ratio': 0.20,
            'water_min_span_ratio': 0.55,
            'water_min_ripple_ratio': 0.015,
            'water_turbid_color_ratio': 0.35,
            'water_clear_max_muddy_ratio': 0.18,
            'water_clear_min_detail_ratio': 0.06
        }
        for key, value in defaults.items():
            self.config.setdefault(key, value)
        for key in defaults:
            value = self.config[key]
            upper = 255.0 if key in ('road_max_saturation', 'fire_dim_min_value', 'fire_dim_min_local_contrast') else 1.0
            if not isinstance(value, (int, float)) or not np.isfinite(value) or not 0 <= value <= upper:
                raise ValueError('Invalid classifier parameter: ' + key)
        if self.config['water_clear_max_muddy_ratio'] >= self.config['water_turbid_color_ratio']:
            raise ValueError('Clear-water threshold must be below turbid-water threshold')
        if self.config['fire_dim_min_ratio'] >= self.config['fire_dim_max_ratio']:
            raise ValueError('Minimum dim-fire ratio must be below maximum')
        self.call_count = 0
        self.k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.k7 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self.k9 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        self.k15 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        self.k_blackhat = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        self.k_horiz_tophat = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 1))
        self._saliency_cache = {}

    def _get_saliency_weights(self, ih, iw):
        key = (ih, iw)
        if key not in self._saliency_cache:
            yy, xx = np.mgrid[0:ih, 0:iw]
            w = np.exp(-0.5 * (((xx - 0.50 * iw) / (0.30 * iw)) ** 2 + ((yy - 0.55 * ih) / (0.32 * ih)) ** 2))
            self._saliency_cache[key] = (yy.astype(np.float32), xx.astype(np.float32), w.astype(np.float32), float(np.sum(w)))
        return self._saliency_cache[key]

    @staticmethod
    def rectify_patch(frame, poly_pts, out_size=160):
        src = np.array(poly_pts, dtype=np.float32).reshape(4, 2)
        center = np.mean(src, axis=0)
        src_inner = center + 0.94 * (src - center)
        dst = np.array([
            [0, 0],
            [out_size - 1, 0],
            [out_size - 1, out_size - 1],
            [0, out_size - 1]
        ], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src_inner.astype(np.float32), dst)
        return cv2.warpPerspective(frame, M, (out_size, out_size))

    def _extract_fire_features(self, ih, iw, total_px, b_ch, g_ch, r_ch, h_ch, s_ch, v_ch):
        active_roy = (
            ((h_ch <= 28) | (h_ch >= 166)) &
            (s_ch >= 80) & (v_ch >= 180) &
            (r_ch >= 205) & (r_ch - g_ch >= 18) & (g_ch - b_ch >= 10)
        )
        amber_fire = (
            (h_ch <= 21) & (s_ch >= 125) & (v_ch >= 105) &
            (r_ch >= 135) & (r_ch - g_ch >= 35) & (g_ch - b_ch >= 20) & (r_ch - b_ch >= 95)
        )
        white_hot = (v_ch >= 240) & (r_ch >= 235) & (g_ch >= 195) & (r_ch >= b_ch)
        roy_dilated = cv2.dilate(np.uint8(active_roy) * 255, self.k7)
        raw_flame = np.uint8(active_roy | (white_hot & (roy_dilated > 0))) * 255
        raw_flame = cv2.morphologyEx(raw_flame, cv2.MORPH_CLOSE, self.k3)

        cnts, _ = find_contours_compat(raw_flame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        flame_mask = np.zeros_like(raw_flame)
        sig_cnts = []
        if cnts:
            max_c_area = max([cv2.contourArea(c) for c in cnts])
            min_keep_area = max(30.0, max_c_area * 0.22)
            for c in cnts:
                if cv2.contourArea(c) >= min_keep_area:
                    sig_cnts.append(c)
                    cv2.drawContours(flame_mask, [c], -1, 255, -1)

        # Printed fire may be far dimmer than a real self-luminous flame.
        # Require small warm strokes and local brightness contrast; lowering
        # the bright-fire threshold globally would also accept brown surfaces.
        dim_warm = (((h_ch <= 32) | (h_ch >= 170)) & (s_ch >= 60) &
                    (v_ch >= self.config['fire_dim_min_value']) &
                    (r_ch-g_ch >= 10) & (r_ch-b_ch >= 18) & (g_ch-b_ch >= -6))
        dim_candidates = np.uint8(dim_warm)*255
        dim_mask = np.zeros_like(dim_candidates)
        dim_ratio = float(np.mean(dim_warm))
        dim_contrasts = []
        if self.config['fire_dim_min_ratio'] <= dim_ratio <= self.config['fire_dim_max_ratio']:
            dim_cnts, _ = find_contours_compat(dim_candidates, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in dim_cnts:
                x, y, width, height = cv2.boundingRect(contour)
                if height < 3 or float(height)/max(width, 1) < 1.15:
                    continue
                component = np.zeros_like(dim_candidates)
                cv2.drawContours(component, [contour], -1, 255, -1)
                count = cv2.countNonZero(component)
                if count < 6:
                    continue
                ring = (cv2.dilate(component, self.k9) > 0) & (dim_candidates == 0)
                if np.sum(ring) < 8:
                    continue
                contrast = float(np.percentile(v_ch[component > 0], 85)-np.median(v_ch[ring]))
                if contrast < self.config['fire_dim_min_local_contrast']:
                    continue
                dim_contrasts.append(contrast)
                dim_mask = cv2.bitwise_or(dim_mask, component)
                sig_cnts.append(contour)
            flame_mask = cv2.bitwise_or(flame_mask, dim_mask)
        dim_flame_ratio = float(cv2.countNonZero(dim_mask))/total_px
        dim_fire = dim_flame_ratio >= self.config['fire_dim_min_ratio']
        active_flame_ratio = float(cv2.countNonZero(flame_mask)) / total_px
        amber_ratio = float(np.mean(amber_fire))

        dark_px = (v_ch <= 45)
        near_dark = cv2.dilate(np.uint8(dark_px) * 255, self.k15) > 0
        amber_near_dark_ratio = float(np.mean(amber_fire & near_dark))

        lum_core = (r_ch >= 225) & (g_ch >= 150) & (v_ch >= 225) & (r_ch - b_ch >= 30)
        lum_core_ratio = float(np.mean(lum_core & (roy_dilated > 0)))

        corners_v = np.mean([v_ch[:15, :15], v_ch[:15, -15:], v_ch[-15:, :15], v_ch[-15:, -15:]])
        blue_flame_mask = (
            (h_ch >= 92) & (h_ch <= 128) & (s_ch >= 85) & (v_ch >= 175) & (b_ch - r_ch > 40)
        )
        blue_flame_ratio = float(np.mean(blue_flame_mask[20:-20, 20:-20])) if corners_v < 70.0 else 0.0

        fire_glare = (
            (h_ch <= 24) & (s_ch >= 115) & (v_ch >= 85) &
            (r_ch - g_ch > 40) & (flame_mask == 0)
        )
        fire_glare_ratio = float(np.mean(fire_glare))
        total_fire_energy = active_flame_ratio + 0.70 * fire_glare_ratio

        flame_span_x = 0.0
        flame_span_y = 0.0
        if sig_cnts:
            all_pts = np.vstack(sig_cnts)
            _, _, fw, fh = cv2.boundingRect(all_pts)
            flame_span_x = float(fw) / float(iw)
            flame_span_y = float(fh) / float(ih)

        is_fire = (
            (amber_ratio >= 0.005 and (
                (active_flame_ratio >= 0.010 and lum_core_ratio >= 0.0015) or
                (amber_near_dark_ratio >= 0.005 and amber_ratio >= 0.006)
            )) or
            (blue_flame_ratio >= 0.035) or dim_fire
        )

        return {
            'is_fire': is_fire,
            'possible_fire': (not is_fire and self.config['fire_dim_min_ratio'] <= dim_ratio <= self.config['fire_dim_max_ratio']),
            'dim_flame_ratio': dim_flame_ratio,
            'dim_warm_ratio': dim_ratio,
            'dim_local_contrast': max(dim_contrasts) if dim_contrasts else 0.0,
            'active_flame_ratio': active_flame_ratio,
            'amber_ratio': amber_ratio,
            'lum_core_ratio': lum_core_ratio,
            'blue_flame_ratio': blue_flame_ratio,
            'fire_glare_ratio': fire_glare_ratio,
            'total_fire_energy': total_fire_energy,
            'flame_span_x': flame_span_x,
            'flame_span_y': flame_span_y
        }

    def _extract_water_features(self, ih, iw, total_px, gray, gx_full, gy_full, b_u8, b_ch, r_ch, h_ch, s_ch, v_ch):
        y_ground = int(ih * 0.28)
        ground_area = float((ih - y_ground) * iw)

        # 直接切片复用全图单次计算的 gx_full, gy_full
        y_bot_water = int(ih * 0.60)
        gx_b = float(np.mean(gx_full[y_bot_water:, :]))
        gy_b = float(np.mean(gy_full[y_bot_water:, :]))
        bot_s_mean = float(np.mean(s_ch[y_bot_water:, :]))
        bot_s_std = float(np.std(s_ch[y_bot_water:, :]))

        if gx_b < 24.0 and gy_b >= gx_b * 1.05:
            aqua_blue = (h_ch >= 72) & (h_ch <= 135) & (s_ch >= 25) & (v_ch >= 40) & (b_ch - r_ch >= 14)
        else:
            aqua_blue = (h_ch >= 80) & (h_ch <= 132) & (s_ch >= 48) & (v_ch >= 45) & (b_ch - r_ch >= 22)

        aqua_u8 = cv2.morphologyEx(np.uint8(aqua_blue) * 255, cv2.MORPH_CLOSE, self.k5)
        aqua_u8 = cv2.morphologyEx(aqua_u8, cv2.MORPH_OPEN, self.k3)
        ground_aqua_ratio = float(np.mean(aqua_u8[y_ground:, :] > 0))

        cnts_w, _ = find_contours_compat(aqua_u8[y_ground:, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_pond_ratio = 0.0
        pond_span_x = 0.0
        if cnts_w:
            main_pond = max(cnts_w, key=cv2.contourArea)
            max_pond_ratio = float(cv2.contourArea(main_pond)) / ground_area
            _, _, pw, _ = cv2.boundingRect(main_pond)
            pond_span_x = float(pw) / float(iw)

        is_planar_flood = (
            gx_b < 18.0 and
            gy_b >= gx_b * 1.12 and
            bot_s_mean >= 28.0 and
            bot_s_std < 25.0
        )

        vert_tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, self.k_horiz_tophat)
        v_jets_ratio = float(np.mean((vert_tophat[15:130, :] >= 28) & (v_ch[15:130, :] >= 175) & (s_ch[15:130, :] <= 65)))

        gx_m = float(np.mean(gx_full[15:125, :]))
        gy_m = float(np.mean(gy_full[15:125, :]))
        vert_grad_ratio = gx_m / max(gy_m, 1.0)
        bot_white_ratio = float(np.mean((v_ch[int(ih * 0.76):, :] >= 195) & (s_ch[int(ih * 0.76):, :] <= 48)))

        white_foam = np.uint8((v_ch >= 205) & (s_ch <= 60) & (b_u8 >= 195)) * 255
        fountain_spray = cv2.bitwise_and(white_foam, cv2.dilate(aqua_u8, self.k9))
        spray_ratio = float(cv2.countNonZero(fountain_spray)) / total_px

        # 闭集水源内部竞争得分: 漫水得分 flood_score vs 喷泉得分 fountain_score
        flood_score = 0.0
        if max_pond_ratio >= 0.28 and pond_span_x >= 0.60 and spray_ratio < 0.06 and v_jets_ratio < 0.025:
            flood_score = max(flood_score, min(0.99, 0.60 + max_pond_ratio * 0.45))
        if is_planar_flood:
            flood_score = max(flood_score, 0.92)

        fountain_score = 0.0
        if v_jets_ratio >= 0.022 and vert_grad_ratio >= 1.22 and (bot_white_ratio + ground_aqua_ratio) >= 0.08:
            fountain_score = max(fountain_score, min(0.99, 0.65 + (vert_grad_ratio - 1.0) * 0.20))
        if ground_aqua_ratio >= 0.04 and spray_ratio >= 0.015 and max_pond_ratio < 0.45:
            fountain_score = max(fountain_score, min(0.99, 0.65 + spray_ratio * 4.0))

        # Water identity is independent of whether the water is normal or abnormal.
        # Warm brown/yellow surfaces require additional water-like structure.
        muddy = ((h_ch >= 8) & (h_ch <= 38) & (s_ch >= 30) &
                 (s_ch <= 220) & (v_ch >= 45) & (v_ch <= 245) & (r_ch - b_ch >= 12))
        water_mask = np.uint8(aqua_blue | muddy) * 255
        water_mask = cv2.morphologyEx(water_mask, cv2.MORPH_CLOSE, self.k5)
        water_mask = cv2.morphologyEx(water_mask, cv2.MORPH_OPEN, self.k3)
        water_contours, _ = find_contours_compat(water_mask[y_ground:, :], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        surface_mask = np.zeros((ih, iw), dtype=np.uint8)
        surface_ratio, surface_span = 0.0, 0.0
        if water_contours:
            largest = max(water_contours, key=cv2.contourArea)
            surface_ratio = float(cv2.contourArea(largest)) / ground_area
            _, _, sw, _ = cv2.boundingRect(largest)
            surface_span = float(sw) / iw
            cv2.drawContours(surface_mask[y_ground:, :], [largest], -1, 255, -1)
        # The old blue-water detector also covers grey reflection/foam areas.
        old_water = (flood_score >= 0.55 or fountain_score >= 0.55)
        if old_water:
            surface_mask[y_ground:, :] = 255
        sample_mask = (cv2.erode(surface_mask, self.k3) > 0) & (v_ch > 35) & (v_ch < 245)
        sample_count = int(np.sum(sample_mask))
        if sample_count:
            muddy_ratio = float(np.mean(muddy[sample_mask]))
            cool_ratio = float(np.mean(aqua_blue[sample_mask]))
            detail_ratio = float(np.mean((gx_full[sample_mask] + gy_full[sample_mask]) >= 28))
            ripple_ratio = float(np.mean((gy_full[sample_mask] >= 24) &
                                        (gy_full[sample_mask] >= 1.4 * gx_full[sample_mask])))
        else:
            muddy_ratio = cool_ratio = detail_ratio = ripple_ratio = 0.0
        broad_surface = (surface_ratio >= self.config['water_min_area_ratio'] and
                         surface_span >= self.config['water_min_span_ratio'])
        water_structure = (ripple_ratio >= self.config['water_min_ripple_ratio'] or spray_ratio >= 0.015)
        is_water = old_water or (broad_surface and water_structure)
        # Brown colour alone is insufficient: it may be soil, skin or a road.
        ambiguous_water = broad_surface and not is_water
        if is_water and muddy_ratio >= self.config['water_turbid_color_ratio']:
            water_status = 'ABNORMAL'
        elif (is_water and muddy_ratio <= self.config['water_clear_max_muddy_ratio'] and
              (cool_ratio >= 0.20 or detail_ratio >= self.config['water_clear_min_detail_ratio'])):
            water_status = 'NORMAL'
        else:
            water_status = 'UNKNOWN'
        return {
            'is_water': is_water,
            'ambiguous_water': ambiguous_water,
            'water_status': water_status,
            'muddy_color_ratio': muddy_ratio,
            'cool_water_ratio': cool_ratio,
            'water_detail_ratio': detail_ratio,
            'water_ripple_ratio': ripple_ratio,
            'water_surface_ratio': surface_ratio,
            'flood_score': flood_score,
            'fountain_score': fountain_score,
            'max_pond_ratio': max_pond_ratio,
            'pond_span_x': pond_span_x,
            'gx_bot': gx_b,
            'gy_bot': gy_b,
            'v_jets_ratio': v_jets_ratio,
            'vert_grad_ratio': vert_grad_ratio,
            'spray_ratio': spray_ratio
        }

    def _extract_pedestrian_features(self, ih, iw, total_px, inner_bgr, b_ch, g_ch, r_ch, h_ch, s_ch, v_ch):
        lab = cv2.cvtColor(inner_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
        bg_mask = np.zeros((ih, iw), dtype=bool)
        bg_mask[-18:, :] = True
        bg_mask[int(ih * 0.45):, :16] = True
        bg_mask[int(ih * 0.45):, -16:] = True

        bg_l = float(np.median(lab[:, :, 0][bg_mask]))
        bg_a = float(np.median(lab[:, :, 1][bg_mask]))
        bg_b = float(np.median(lab[:, :, 2][bg_mask]))
        bg_v = float(np.median(v_ch[bg_mask]))

        lab_dist = np.sqrt((lab[:, :, 1] - bg_a) ** 2 + (lab[:, :, 2] - bg_b) ** 2)
        veg_mask = (h_ch >= 22) & (h_ch <= 92) & (g_ch >= r_ch - 8) & (g_ch >= b_ch + 4)

        chroma_or_dark = ((lab_dist >= 16.0) | (s_ch >= 50) | (v_ch < min(bg_l - 38.0, 72.0))) & (~veg_mask)
        bright_cloth = (v_ch >= max(bg_v + 42.0, 190.0)) & (s_ch <= 42)
        bright_cloth[:int(ih * 0.22), :] = False
        fg_raw = chroma_or_dark | bright_cloth

        fg_clean = cv2.morphologyEx(np.uint8(fg_raw) * 255, cv2.MORPH_OPEN, self.k5)
        fg_clean = cv2.morphologyEx(fg_clean, cv2.MORPH_CLOSE, self.k9)

        cnts_p, _ = find_contours_compat(fg_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        ground_fg = np.zeros_like(fg_clean)
        for c in cnts_p:
            area = cv2.contourArea(c)
            if area < (total_px * 0.015):
                continue
            px, py, pw, ph = cv2.boundingRect(c)
            if min(pw, ph) < 12:
                continue
            if (py + ph) < int(ih * 0.48):
                continue
            if (px <= 2 and (px + pw) < int(iw * 0.22)) or ((px + pw) >= iw - 2 and px > int(iw * 0.78)):
                continue
            c_mask = np.zeros((ih, iw), dtype=np.uint8)
            cv2.drawContours(c_mask, [c], -1, 255, -1)
            cd_px = float(np.sum(chroma_or_dark & (c_mask > 0)))
            if cd_px < 80.0 and (cd_px / max(area, 1.0)) < 0.15:
                continue
            cv2.drawContours(ground_fg, [c], -1, 255, -1)

        yy, xx, w_saliency, w_total = self._get_saliency_weights(ih, iw)
        fg_bool = (ground_fg > 0)
        salient_fg_score = float(np.sum(w_saliency[fg_bool])) / w_total

        if salient_fg_score < 0.040:
            return {
                'has_pedestrian': False,
                'salient_fg_score': salient_fg_score,
                'std_y': 0.0,
                'std_x': 0.0,
                'vy_vx_ratio': 1.0,
                'col_row_ratio': 1.0,
                'upright_score': 0.0
            }

        weights = w_saliency[fg_bool]
        ys = yy[fg_bool]
        xs = xx[fg_bool]
        w_sum = float(np.sum(weights))
        cy = float(np.sum(weights * ys) / w_sum)
        cx = float(np.sum(weights * xs) / w_sum)
        var_y = float(np.sum(weights * (ys - cy) ** 2) / w_sum)
        var_x = float(np.sum(weights * (xs - cx) ** 2) / w_sum)

        std_y = float(np.sqrt(var_y))
        std_x = float(np.sqrt(var_x))
        vy_vx_ratio = std_y / max(std_x, 1.0)

        # 单列垂直占据度 p85 与单行水平占据度 p85 比值 (对多人并排站立具备强不变性)
        col_occ = np.mean(fg_bool, axis=0)
        row_occ = np.mean(fg_bool, axis=1)
        active_cols = col_occ[col_occ > 0.05]
        active_rows = row_occ[row_occ > 0.05]
        col_p85 = float(np.percentile(active_cols, 85)) if len(active_cols) > 0 else 0.0
        row_p85 = float(np.percentile(active_rows, 85)) if len(active_rows) > 0 else 0.01
        col_row_ratio = col_p85 / max(row_p85, 0.05)

        norm_std_y = std_y / float(ih)
        upright_score = (
            0.40 * min(vy_vx_ratio / 0.72, 1.6) +
            0.35 * min(norm_std_y / 0.135, 1.6) +
            0.25 * min(col_row_ratio / 0.80, 1.6)
        )

        return {
            'has_pedestrian': True,
            'salient_fg_score': salient_fg_score,
            'std_y': std_y,
            'std_x': std_x,
            'vy_vx_ratio': vy_vx_ratio,
            'col_row_ratio': col_row_ratio,
            'upright_score': upright_score
        }

    def _extract_road_features(self, ih, gray, s_ch):
        gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)
        blackhat = cv2.morphologyEx(gray_blur, cv2.MORPH_BLACKHAT, self.k_blackhat)
        # 闭集道路优化: 用顶帽变换 (MORPH_TOPHAT) 检出高亮白/黄标线与箭头，屏蔽其边缘过渡带，从而安全提升细裂纹检出率
        tophat = cv2.morphologyEx(gray_blur, cv2.MORPH_TOPHAT, self.k_blackhat)
        lane_paint_halo = cv2.dilate(np.uint8(tophat >= 28) * 255, self.k5) > 0

        y_road = int(ih * 0.25)
        road_bh = blackhat[y_road:, 10:-10]
        road_g = gray_blur[y_road:, 10:-10]
        road_s = s_ch[y_road:, 10:-10]
        road_halo = lane_paint_halo[y_road:, 10:-10]
        med_road_g = float(np.median(road_g))

        crack_mask = np.uint8(
            (road_bh >= 20) &
            (road_g < med_road_g - 13.0) &
            (road_s < 38) &
            (~road_halo)
        ) * 255
        crack_cnts, _ = find_contours_compat(crack_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        valid_crack_px = 0.0
        max_crack_span = 0.0
        for cc in crack_cnts:
            ca = cv2.contourArea(cc)
            cp = cv2.arcLength(cc, False)
            _, _, cw, ch_c = cv2.boundingRect(cc)
            span = float(max(cw, ch_c))
            if cp >= 16.0 or span >= 11.0:
                valid_crack_px += max(ca, cp * 1.5)
                max_crack_span = max(max_crack_span, span)

        road_area = float(road_bh.shape[0] * road_bh.shape[1])
        crack_density = valid_crack_px / max(road_area, 1.0)

        has_cracks = (crack_density >= 0.016 or max_crack_span >= 25.0)
        surface = ((road_s <= self.config['road_max_saturation']) & (road_g >= 45) & (road_g <= 235))
        surface_ratio = float(np.mean(surface))
        texture = np.abs(cv2.Sobel(road_g, cv2.CV_32F, 1, 0)) + np.abs(cv2.Sobel(road_g, cv2.CV_32F, 0, 1))
        texture_ratio = float(np.mean(texture >= 20))
        has_road_surface = (surface_ratio >= self.config['road_min_surface_ratio'] and
                            (has_cracks or texture_ratio >= self.config['road_min_texture_ratio']))
        return {
            'has_road_surface': has_road_surface,
            'surface_ratio': surface_ratio,
            'texture_ratio': texture_ratio,
            'has_cracks': has_cracks,
            'crack_density': crack_density,
            'max_crack_span': max_crack_span
        }

    def classify_patch(self, patch):
        """基于闭集四分类先验与共享梯度的 8 类场景判别"""
        self.call_count += 1
        if patch.shape[0] != 160 or patch.shape[1] != 160:
            patch = cv2.resize(patch, (160, 160), interpolation=cv2.INTER_LINEAR)

        inner_bgr = patch[6:154, 6:154]
        ih, iw = inner_bgr.shape[:2]
        total_px = float(ih * iw)
        inner_hsv = cv2.cvtColor(inner_bgr, cv2.COLOR_BGR2HSV)
        b_u8, g_u8, r_u8 = cv2.split(inner_bgr)
        b_ch, g_ch, r_ch = b_u8.astype(np.int16), g_u8.astype(np.int16), r_u8.astype(np.int16)
        h_ch, s_ch, v_ch = cv2.split(inner_hsv)
        gray = cv2.cvtColor(inner_bgr, cv2.COLOR_BGR2GRAY)

        # 1. 火源特征提取与判别 (若非火源，甚至无需提前计算全图 Sobel，按需惰性计算更省 CPU！)
        f_feat = self._extract_fire_features(ih, iw, total_px, b_ch, g_ch, r_ch, h_ch, s_ch, v_ch)
        if f_feat['is_fire']:
            abnormal_score = 0.0
            reasons = []
            if f_feat['active_flame_ratio'] >= 0.085 or f_feat['total_fire_energy'] >= 0.112 or f_feat['fire_glare_ratio'] >= 0.108:
                abnormal_score += 0.58
                reasons.append("HighFire:{:.0f}%".format(max(f_feat['total_fire_energy'], f_feat['fire_glare_ratio']) * 100))
            if f_feat['flame_span_x'] >= 0.38 or f_feat['flame_span_y'] >= 0.58:
                abnormal_score += 0.38
                reasons.append("LargeSpan:{:.0f}%".format(max(f_feat['flame_span_x'], f_feat['flame_span_y']) * 100))
            if f_feat['active_flame_ratio'] < 0.055 and f_feat['flame_span_x'] < 0.32 and f_feat['total_fire_energy'] < 0.095 and f_feat['fire_glare_ratio'] < 0.085:
                abnormal_score -= 0.45
                reasons.append("ContainedFlame:{:.1f}%".format(max(f_feat['active_flame_ratio'], f_feat['amber_ratio']) * 100))

            is_abn = (abnormal_score >= 0.45)
            conf = min(max(0.78 + abs(abnormal_score - 0.45) * 0.45, 0.78), 0.99)
            if not reasons:
                reasons.append("LargeFire" if is_abn else "SmallFlame")
            return {
                'category': 'FIRE',
                'status': 'ABNORMAL' if is_abn else 'NORMAL',
                'label': 'ABNORMAL_FIRE' if is_abn else 'NORMAL_FIRE',
                'scene_label': 'FIRE_ABNORMAL' if is_abn else 'FIRE_NORMAL',
                'is_abnormal': is_abn,
                'confidence': conf,
                'abnormal_score': abnormal_score if is_abn else 0.12,
                'reason': "+".join(reasons[:2]),
                'features': f_feat,
                'warped': patch
            }

        # 单次计算全图 Sobel 梯度场，供后续水源与结构分析共享
        gx_full = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3))
        gy_full = np.abs(cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3))

        # 2. First identify water; then assess visible clarity, never water area.
        w_feat = self._extract_water_features(ih, iw, total_px, gray, gx_full, gy_full, b_u8, b_ch, r_ch, h_ch, s_ch, v_ch)
        if w_feat['is_water']:
            water_status = w_feat['water_status']
            is_abn = water_status == 'ABNORMAL'
            reason = ('TurbidWater' if is_abn else 'ClearWater') if water_status != 'UNKNOWN' else 'WaterClarityUncertain'
            return {
                'category': 'WATER',
                'status': water_status,
                'label': water_status + '_WATER',
                'scene_label': 'WATER_' + water_status,
                'is_abnormal': is_abn if water_status != 'UNKNOWN' else None,
                'confidence': 0.0 if water_status == 'UNKNOWN' else min(0.90, 0.55 + 0.35 * (w_feat['muddy_color_ratio'] if is_abn else 1.0 - w_feat['muddy_color_ratio'])),
                'abnormal_score': (0.88 if is_abn else 0.10) if water_status != 'UNKNOWN' else None,
                'reason': reason + ':Mud{:.0f}%'.format(w_feat['muddy_color_ratio'] * 100),
                'features': w_feat,
                'warped': patch
            }

        # 3. 行人特征提取与多特征姿态融合判别
        p_feat = self._extract_pedestrian_features(ih, iw, total_px, inner_bgr, b_ch, g_ch, r_ch, h_ch, s_ch, v_ch)
        if p_feat['has_pedestrian']:
            vy_vx = p_feat['vy_vx_ratio']
            cr_ratio = p_feat['col_row_ratio']
            is_upright = (p_feat['upright_score'] >= 0.95) and (vy_vx >= 0.68 and (p_feat['std_y'] / float(ih)) >= 0.135)
            if is_upright:
                return {
                    'category': 'PEDESTRIAN',
                    'status': 'NORMAL',
                    'label': 'NORMAL_PEDESTRIAN',
                    'scene_label': 'PEDESTRIAN_NORMAL',
                    'is_abnormal': False,
                    'confidence': 0.95,
                    'abnormal_score': 0.10,
                    'reason': "Standing(Sy/Sx:{:.2f},CR:{:.2f})".format(vy_vx, cr_ratio),
                    'features': p_feat,
                    'warped': patch
                }
            else:
                return {
                    'category': 'PEDESTRIAN',
                    'status': 'ABNORMAL',
                    'label': 'ABNORMAL_PEDESTRIAN',
                    'scene_label': 'PEDESTRIAN_ABNORMAL',
                    'is_abnormal': True,
                    'confidence': 0.95,
                    'abnormal_score': 0.88,
                    'reason': "Fallen(Sx/Sy:{:.2f},CR:{:.2f})".format(1.0 / max(vy_vx, 0.1), cr_ratio),
                    'features': p_feat,
                    'warped': patch
                }

        # 4. Road is eligible only after fire/water/person have been excluded.
        r_feat = self._extract_road_features(ih, gray, s_ch)
        if f_feat.get('possible_fire') or w_feat.get('ambiguous_water') or not r_feat['has_road_surface']:
            return {
                'category': 'UNKNOWN', 'status': 'UNKNOWN', 'label': 'UNKNOWN',
                'scene_label': 'UNKNOWN', 'is_abnormal': None, 'confidence': 0.0,
                'abnormal_score': None,
                'reason': 'PossibleFire' if f_feat.get('possible_fire') else ('PossibleWater' if w_feat.get('ambiguous_water') else 'InsufficientRoadEvidence'),
                'features': dict(road=r_feat, water=w_feat), 'warped': patch
            }
        if r_feat['has_cracks']:
            return {
                'category': 'ROAD',
                'status': 'ABNORMAL',
                'label': 'ABNORMAL_ROAD',
                'scene_label': 'ROAD_ABNORMAL',
                'is_abnormal': True,
                'confidence': 0.96,
                'abnormal_score': 0.88,
                'reason': "GroundCracks:{:.1f}%".format(r_feat['crack_density'] * 100),
                'features': r_feat,
                'warped': patch
            }
        else:
            return {
                'category': 'ROAD',
                'status': 'NORMAL',
                'label': 'NORMAL_ROAD',
                'scene_label': 'ROAD_NORMAL',
                'is_abnormal': False,
                'confidence': 0.95,
                'abnormal_score': 0.08,
                'reason': "ClearRoad",
                'features': r_feat,
                'warped': patch
            }

    def classify(self, frame, poly_pts):
        patch = self.rectify_patch(frame, poly_pts, out_size=160)
        return self.classify_patch(patch)


# 保持向后兼容别名
FireSourceClassifier = MultiSceneClassifier


# ================= 超轻量核心检测器类 =================
class OverheadPicDetector(object):
    def __init__(self, config=None):
        cfg = config or {}
        self.proc_width = cfg.get('proc_width', 320)

        self.hsv_low = np.array(cfg.get('hsv_low', [86, 35, 18]), dtype=np.uint8)
        self.hsv_high = np.array(cfg.get('hsv_high', [142, 255, 255]), dtype=np.uint8)

        self.enable_light_norm = cfg.get('enable_light_norm', True)
        self.enable_adaptive_blue = cfg.get('enable_adaptive_blue', True)
        self.enable_glare_reject = cfg.get('enable_glare_reject', True)
        self.enable_fullres_refine = cfg.get('enable_fullres_refine', True)

        self.ab_tol = int(cfg.get('ab_tol', 19))
        self.h_tol = int(cfg.get('h_tol', 12))

        self.roi_y_min = cfg.get('roi_y_min', 0.01)
        self.roi_y_max = cfg.get('roi_y_max', 0.88)

        self.min_area_ratio = cfg.get('min_area_ratio', 0.012)
        self.max_area_ratio = cfg.get('max_area_ratio', 0.180)

        self.min_rectangularity = cfg.get('min_rectangularity', 0.70)
        self.min_quad_fit = cfg.get('min_quad_fit', 0.84)
        self.min_aspect_ratio = cfg.get('min_aspect_ratio', 0.42)
        self.max_aspect_ratio = cfg.get('max_aspect_ratio', 2.25)

        self.min_lr_blue_ratio = cfg.get('min_lr_blue_ratio', 0.44)
        self.min_bot_blue_ratio = cfg.get('min_bot_blue_ratio', 0.30)
        self.min_avg_blue_ratio = cfg.get('min_avg_blue_ratio', 0.52)
        self.min_edge_valid_ratio = cfg.get('min_edge_valid_ratio', 0.60)

        self.trigger_frames = cfg.get('trigger_frames', 2)
        self.max_lost_frames = cfg.get('max_lost_frames', 2)
        self.fire_vote_frames = int(cfg.get('fire_vote_frames', 3))
        if not 1 <= self.fire_vote_frames <= 30:
            raise ValueError('fire_vote_frames must be between 1 and 30')
        self.same_target_min_iou = cfg.get('same_target_min_iou', 0.22)
        self.same_target_max_shift = cfg.get('same_target_max_shift', 0.48)

        self.hit_count = 0
        self.lost_count = 0
        self.locked = False
        self.last_box = None

        self.classify_fire = cfg.get('classify_fire', True)
        self.fire_classifier = MultiSceneClassifier(cfg.get('classifier_config'))
        self.fire_vote_history = []
        self.cached_fire_info = None
        self.fire_finalized = False

        self._gamma_luts = {}
        for g_int in range(4, 24, 2):
            g = g_int / 10.0
            inv_g = 1.0 / g
            self._gamma_luts[g_int] = np.array(
                [((i / 255.0) ** inv_g) * 255.0 for i in range(256)], dtype=np.uint8
            )

        if hasattr(cv2, 'createCLAHE'):
            self.clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(6, 6))
        else:
            self.clahe = None

        self.k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self.k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        self.k_repair = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))

    def reset_tracking(self):
        """彻底清空连续帧计数、目标位置与分类缓存"""
        self.hit_count = 0
        self.lost_count = 0
        self.locked = False
        self.last_box = None
        self.fire_vote_history = []
        self.cached_fire_info = None
        self.fire_finalized = False

    def _is_same_target(self, new_bbox):
        """严格校验候选框是否与当前跟踪目标 (self.last_box) 为同一物理目标"""
        if self.last_box is None:
            return True

        old_bbox = self.last_box['bbox']
        iou = compute_bbox_iou(old_bbox, new_bbox)
        if iou < self.same_target_min_iou:
            return False

        x1, y1, w1, h1 = old_bbox
        x2, y2, w2, h2 = new_bbox
        c1x, c1y = x1 + w1 * 0.5, y1 + h1 * 0.5
        c2x, c2y = x2 + w2 * 0.5, y2 + h2 * 0.5
        dist = float(np.hypot(c1x - c2x, c1y - c2y))
        ref_dim = float(max(min(max(w1, h1), max(w2, h2)), 1.0))
        if dist > ref_dim * self.same_target_max_shift:
            return False

        area1 = max(float(w1 * h1), 1.0)
        area2 = max(float(w2 * h2), 1.0)
        area_ratio = area2 / area1
        if area_ratio < 0.45 or area_ratio > 2.20:
            return False

        return True

    def _is_archway_opening(self, blue_mask, x, y, bw, bh, b_b, t_b):
        """鉴别并排除头顶墙下方的大块白色门洞通道"""
        h, w = blue_mask.shape[:2]
        if b_b < self.min_bot_blue_ratio:
            return True

        col_slice = blue_mask[:, max(0, x - bw // 2):min(w, x + bw + bw // 2)]
        ys = np.where(np.any(col_slice > 0, axis=1))[0]
        if len(ys) > 10:
            wall_y_bot = int(np.percentile(ys, 97))
            if (y + bh >= wall_y_bot - max(4, int(bh * 0.12))) and (b_b < 0.45):
                return True

        return False

    def _compute_surround_blue_ratios_quad(self, blue_mask, poly_pts):
        """沿四边形四条斜边外法线方向采样，返回有效蓝色占比、画面内有效采样率与观测不完整标志"""
        h, w = blue_mask.shape[:2]
        pts = np.array(poly_pts, dtype=np.float32).reshape(4, 2)
        center = np.mean(pts, axis=0)

        edge_pairs = [(0, 1), (1, 2), (2, 3), (3, 0)]
        eff_ratios = []
        val_ratios = []
        t_vals = np.linspace(0.08, 0.92, 10, dtype=np.float32)

        for i1, i2 in edge_pairs:
            p1 = pts[i1]
            p2 = pts[i2]
            edge_vec = p2 - p1
            edge_len = float(np.hypot(edge_vec[0], edge_vec[1]))
            if edge_len < 2.0:
                eff_ratios.append(0.0)
                val_ratios.append(0.0)
                continue

            nx, ny = -edge_vec[1] / edge_len, edge_vec[0] / edge_len
            mid = 0.5 * (p1 + p2)
            if (mid[0] - center[0]) * nx + (mid[1] - center[1]) * ny < 0:
                nx, ny = -nx, -ny

            pad_dist = max(min(edge_len * 0.28, 28.0), 4.0)
            d_vals = np.linspace(1.5, pad_dist, 4, dtype=np.float32)

            base_x = p1[0] + t_vals[:, None] * edge_vec[0]
            base_y = p1[1] + t_vals[:, None] * edge_vec[1]
            sample_x = np.round(base_x + d_vals[None, :] * nx).astype(np.int32).reshape(-1)
            sample_y = np.round(base_y + d_vals[None, :] * ny).astype(np.int32).reshape(-1)

            total_pts = float(len(sample_x))
            valid = (sample_x >= 0) & (sample_x < w) & (sample_y >= 0) & (sample_y < h)
            num_valid = int(np.sum(valid))
            v_ratio = float(num_valid) / total_pts
            val_ratios.append(v_ratio)

            if num_valid == 0:
                eff_ratios.append(0.0)
            else:
                vx = sample_x[valid]
                vy = sample_y[valid]
                blue_hits = float(np.sum(blue_mask[vy, vx] > 0))
                eff_ratios.append(blue_hits / total_pts)

        t_b, r_b, b_b, l_b = eff_ratios[0], eff_ratios[1], eff_ratios[2], eff_ratios[3]
        t_v, r_v, b_v, l_v = val_ratios[0], val_ratios[1], val_ratios[2], val_ratios[3]
        incomplete_obs = any(v < self.min_edge_valid_ratio for v in (l_v, r_v, t_v, b_v))
        return (l_b, r_b, t_b, b_b), (l_v, r_v, t_v, b_v), incomplete_obs

    def _repair_broken_photo_mask(self, small_bgr, blue_bridged, mask_bridged):
        """在蓝墙桥接内腔对非蓝掩码做形态学闭运算弥合"""
        k_erode = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 5))
        beam_interior = cv2.erode(blue_bridged, k_erode)

        repaired = cv2.morphologyEx(mask_bridged, cv2.MORPH_CLOSE, self.k_repair)
        repaired = cv2.bitwise_and(repaired, beam_interior)
        repaired = cv2.morphologyEx(repaired, cv2.MORPH_OPEN, self.k_open)
        return repaired

    @staticmethod
    def _merge_nearby_fragments(cnts, max_gap_px=18):
        """将同一横梁腔室内被蓝色条带劈成两半的邻近碎片合并求凸包"""
        boxes = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area >= 35:
                boxes.append((cv2.boundingRect(c), c))
        if len(boxes) < 2 or len(boxes) > 12:
            return []

        merged_cnts = []
        n = len(boxes)
        for i in range(n):
            (x1, y1, w1, h1), c1 = boxes[i]
            for j in range(i + 1, n):
                (x2, y2, w2, h2), c2 = boxes[j]
                gap_x = max(0, max(x1, x2) - min(x1 + w1, x2 + w2))
                gap_y = max(0, max(y1, y2) - min(y1 + h1, y2 + h2))
                if gap_x <= max_gap_px and gap_y <= max_gap_px:
                    ux1, uy1 = min(x1, x2), min(y1, y2)
                    ux2, uy2 = max(x1 + w1, x2 + w2), max(y1 + h1, y2 + h2)
                    if (ux2 - ux1) < 140 and (uy2 - uy1) < 120:
                        merged_cnts.append(np.vstack([c1, c2]))
        return merged_cnts

    def _paper_content_quad(self, frame, paper_poly):
        """Estimate the printed rectangle inside a white paper quadrilateral."""
        size = 200
        dst = np.float32([[0, 0], [size-1, 0], [size-1, size-1], [0, size-1]])
        transform = cv2.getPerspectiveTransform(np.float32(paper_poly), dst)
        paper = cv2.warpPerspective(frame, transform, (size, size))
        lab = cv2.cvtColor(paper, cv2.COLOR_BGR2LAB).astype(np.float32)
        # Sample just inside the four edges; exclude corners and tape.
        samples = np.vstack([lab[3:7, 30:170].reshape(-1, 3),
                             lab[193:197, 30:170].reshape(-1, 3),
                             lab[30:170, 3:7].reshape(-1, 3),
                             lab[30:170, 193:197].reshape(-1, 3)])
        paper_color = np.median(samples, axis=0)
        difference = np.linalg.norm(lab-paper_color, axis=2)
        content = difference >= 23.0
        row_ratio = np.mean(content[:, 20:180], axis=1)
        col_ratio = np.mean(content[20:170, :], axis=0)
        rows = np.where(row_ratio >= .40)[0]
        cols = np.where(col_ratio >= .40)[0]
        # Limit trimming so dark print, glare or paper bends cannot shrink to a fragment.
        top = min(max(int(rows[0]) if len(rows) else 7, 5), 24)
        bottom = max(min(int(rows[-1]) if len(rows) else 185, 194), 156)
        left = min(max(int(cols[0]) if len(cols) else 7, 5), 24)
        right = max(min(int(cols[-1]) if len(cols) else 192, 194), 176)
        inner = np.float32([[left, top], [right, top], [right, bottom], [left, bottom]])
        inverse = cv2.getPerspectiveTransform(dst, np.float32(paper_poly))
        return cv2.perspectiveTransform(inner.reshape(1, 4, 2), inverse).reshape(4, 2)

    def _find_paper_candidates(self, small, blue_mask, inv_scale, original):
        """Find paper perimeter without treating blue ink as wall pixels."""
        # The paper edge is only a few pixels thick in the 320px detector image.
        # Use up to 640px for this branch so the white border stays connected.
        scale = min(1., 640./original.shape[1])
        small = original if scale == 1. else cv2.resize(original, (640, int(round(original.shape[0]*scale))))
        inv_scale = 1./scale
        h, w = small.shape[:2]
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, np.uint8([86, 105, 25]), np.uint8([142, 255, 255]))
        white = np.uint8((hsv[:, :, 1] <= 105) & (hsv[:, :, 2] >= 70)) * 255
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 20, 60)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, self.k_open)
        white_closed = cv2.morphologyEx(white, cv2.MORPH_CLOSE, self.k_open)
        edge_contours, _ = find_contours_compat(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        white_contours, _ = find_contours_compat(white_closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        frame_area = float(h*w)
        inner_proposals = []
        for contour in edge_contours:
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            if hull_area < frame_area*.04:
                continue
            perimeter = cv2.arcLength(hull, True)
            for epsilon in (.012, .018, .025, .035, .045):
                approx = cv2.approxPolyDP(hull, epsilon*perimeter, True)
                if len(approx) == 4 and cv2.isContourConvex(approx):
                    if cv2.contourArea(approx)/max(hull_area, 1.) >= .84:
                        inner_proposals.append(order_quad_points(approx.reshape(4, 2)))
                    break
        for contour in list(edge_contours) + list(white_contours):
            hull = cv2.convexHull(contour)
            area = float(cv2.contourArea(hull))
            if not frame_area*max(self.min_area_ratio, .05) <= area <= frame_area*max(self.max_area_ratio, .35):
                continue
            # Perspective paper is not a parallelogram. Never repair its
            # fourth corner using the opposite sides of a minimum rectangle.
            quad, fit = None, 0.
            perimeter = cv2.arcLength(hull, True)
            for epsilon in (.008, .012, .018, .025, .035, .045, .06):
                approx = cv2.approxPolyDP(hull, epsilon*perimeter, True)
                if len(approx) == 4 and cv2.isContourConvex(approx):
                    quad = order_quad_points(approx.reshape(4, 2))
                    fit = float(cv2.contourArea(approx))/max(area, 1.)
                    break
            # A slightly curled sheet produces more than four hull vertices.
            # Remove the least significant corner instead of replacing the
            # perspective trapezoid with a rectangle.
            reduced = hull.reshape(-1, 2).astype(np.float32)
            while len(reduced) > 4:
                before = np.roll(reduced, 1, axis=0)-reduced
                after = np.roll(reduced, -1, axis=0)-reduced
                loss = np.abs(before[:, 0]*after[:, 1]-before[:, 1]*after[:, 0])
                reduced = np.delete(reduced, int(np.argmin(loss)), axis=0)
            reduced_fit = float(cv2.contourArea(reduced))/max(area, 1.) if len(reduced) == 4 else 0.
            if reduced_fit > fit:
                quad, fit = order_quad_points(reduced), reduced_fit
            if quad is None or fit < .80:
                continue
            x, y, bw, bh = cv2.boundingRect(np.float32(quad))
            if x < 2 or y < h*.28 or x+bw >= w-1 or y+bh >= h-1 or y > h*.88:
                continue
            if not .5 <= float(bw)/max(bh, 1) <= 3.2:
                continue
            (lb, rb, tb, bb), unused, incomplete = self._compute_surround_blue_ratios_quad(blue_mask, quad)
            blue_sides = (lb, rb, tb, bb)
            if sum(value >= .30 for value in blue_sides) < 3 or bb < .25 or np.mean(blue_sides) < .40:
                continue
            center = np.mean(quad, axis=0)
            white_sides = []
            for i in range(4):
                p1, p2 = quad[i], quad[(i+1)%4]
                edge = p2-p1
                length = max(float(np.linalg.norm(edge)), 1.)
                normal = np.array([-edge[1], edge[0]], dtype=np.float32)/length
                if np.dot(center-(p1+p2)*.5, normal) < 0:
                    normal = -normal
                samples = [p1+t*edge+d*normal for t in np.linspace(.15, .85, 12) for d in (3., 6., 9.)]
                pts = np.int32(np.round(samples))
                valid = (pts[:, 0] >= 0) & (pts[:, 0] < w) & (pts[:, 1] >= 0) & (pts[:, 1] < h)
                if np.mean(valid) < .9:
                    white_sides.append(0.)
                else:
                    pts = pts[valid]
                    white_sides.append(float(np.mean(white[pts[:, 1], pts[:, 0]] > 0)))
            if np.mean(white_sides) < .58 or sum(value >= .40 for value in white_sides) < 3:
                continue
            paper_quad = order_quad_points(quad*inv_scale)
            content_quad = self._paper_content_quad(original, paper_quad)
            # Match an actual printed-image contour inside the accepted sheet.
            # This handles unequal margins and the curled lower paper edge.
            printed = []
            paper_area = max(cv2.contourArea(quad), 1.)
            for proposal in inner_proposals:
                proposal_area = cv2.contourArea(proposal)
                if not .45*paper_area <= proposal_area <= .92*paper_area:
                    continue
                if any(cv2.pointPolygonTest(np.float32(quad), (float(p[0]), float(p[1])), True) < -.04*max(bw, bh) for p in proposal):
                    continue
                center = np.mean(proposal, axis=0)
                white_support = []
                for i in range(4):
                    p1, p2 = proposal[i], proposal[(i+1)%4]
                    edge = p2-p1
                    normal = np.array([-edge[1], edge[0]], dtype=np.float32)/max(float(np.linalg.norm(edge)), 1.)
                    if np.dot(center-(p1+p2)*.5, normal) > 0:
                        normal = -normal
                    pts = np.int32(np.round([p1+t*edge+4*normal for t in np.linspace(.15, .85, 12)]))
                    pts[:, 0] = np.clip(pts[:, 0], 0, w-1)
                    pts[:, 1] = np.clip(pts[:, 1], 0, h-1)
                    white_support.append(float(np.mean(white[pts[:, 1], pts[:, 0]] > 0)))
                if np.mean(white_support) >= .60 and min(white_support) >= .20:
                    printed.append((proposal_area, proposal))
            if printed:
                printed.sort(key=lambda item: item[0], reverse=True)
                content_quad = printed[0][1]*inv_scale
            poly = np.int32(np.round(content_quad))
            bbox = cv2.boundingRect(poly)
            score = .4*float(np.mean(white_sides)) + .3*float(np.mean(blue_sides)) + .3*fit
            candidates.append(dict(bbox=bbox, poly=poly, paper_poly=np.int32(np.round(paper_quad)),
                                   paper_boundary=True, area=area*inv_scale**2, area_ratio=area/frame_area,
                                   score=score, quad_fit=fit, white_sides=white_sides))
        # Prefer the whole sheet to smaller rectangles inside the same sheet.
        candidates.sort(key=lambda c: c['area'], reverse=True)
        return candidates

    def _refine_quad_on_orig_roi(self, orig_frame, coarse_bbox, coarse_poly, prev_poly=None):
        """低分辨率找位置 -> 原图对应局部窗口 (Full-Res ROI) 精修四边与角点"""
        if not self.enable_fullres_refine:
            return coarse_bbox, coarse_poly

        oh, ow = orig_frame.shape[:2]
        if ow <= self.proc_width:
            return coarse_bbox, coarse_poly

        x, y, bw, bh = coarse_bbox
        pad_x = max(int(bw * 0.28), 12)
        pad_y = max(int(bh * 0.28), 12)
        rx1 = max(0, x - pad_x)
        ry1 = max(0, y - pad_y)
        rx2 = min(ow, x + bw + pad_x)
        ry2 = min(oh, y + bh + pad_y)

        roi = orig_frame[ry1:ry2, rx1:rx2]
        rh, rw = roi.shape[:2]
        if rh < 20 or rw < 20:
            return coarse_bbox, coarse_poly

        roi_blur = cv2.GaussianBlur(roi, (3, 3), 0)
        roi_hsv = cv2.cvtColor(roi_blur, cv2.COLOR_BGR2HSV)
        roi_blue = self._segment_blue_wall_fast(roi_blur, roi_hsv)

        k_close_full = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        blue_closed = cv2.morphologyEx(roi_blue, cv2.MORPH_CLOSE, k_close_full)
        non_blue = cv2.bitwise_not(blue_closed)

        k_horiz_full = cv2.getStructuringElement(cv2.MORPH_RECT, (max(int(bw * 1.35), 25), 3))
        roi_blue_bridged = cv2.morphologyEx(blue_closed, cv2.MORPH_CLOSE, k_horiz_full)

        local_poly = np.array(coarse_poly, dtype=np.int32) - np.array([rx1, ry1], dtype=np.int32)
        guide_mask = np.zeros((rh, rw), dtype=np.uint8)
        cv2.fillConvexPoly(guide_mask, local_poly, 255)
        k_guide = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        guide_mask = cv2.dilate(guide_mask, k_guide)

        target_fg = cv2.bitwise_and(non_blue, cv2.bitwise_and(guide_mask, roi_blue_bridged))
        k_repair_full = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
        target_fg = cv2.morphologyEx(target_fg, cv2.MORPH_CLOSE, k_repair_full)
        target_fg = cv2.morphologyEx(target_fg, cv2.MORPH_OPEN, self.k_open)

        cnts, _ = find_contours_compat(target_fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return coarse_bbox, coarse_poly

        sig = [c for c in cnts if cv2.contourArea(c) >= max(float(bw * bh) * 0.08, 25.0)]
        if not sig:
            return coarse_bbox, coarse_poly

        hull = cv2.convexHull(np.vstack(sig))
        refined_local_poly, quad_fit = fit_quad_to_hull(hull)
        if refined_local_poly is None or quad_fit < 0.82:
            return coarse_bbox, coarse_poly

        refined_poly = refined_local_poly + np.array([rx1, ry1], dtype=np.float32)
        ref_area = cv2.contourArea(refined_poly)
        coarse_area = max(float(cv2.contourArea(np.array(coarse_poly, dtype=np.float32))), 1.0)
        if ref_area < coarse_area * 0.72 or ref_area > coarse_area * 1.35:
            return coarse_bbox, coarse_poly

        ordered_refined = order_quad_points(refined_poly, prev_poly=prev_poly)

        if prev_poly is not None:
            prev_f = np.array(prev_poly, dtype=np.float32).reshape(4, 2)
            mean_shift = float(np.mean(np.linalg.norm(ordered_refined - prev_f, axis=1)))
            if mean_shift < 5.0:
                ordered_refined = 0.70 * ordered_refined + 0.30 * prev_f

        final_poly = np.int32(np.round(ordered_refined))
        fx, fy, fbw, fbh = cv2.boundingRect(final_poly)
        return (int(fx), int(fy), int(fbw), int(fbh)), final_poly

    def _update_fire_classification(self, frame, poly_pts):
        """Bounded sliding vote; keep checking so an early mistake can recover."""
        cur_info = self.fire_classifier.classify(frame, poly_pts)
        self.fire_vote_history.append(cur_info)
        self.fire_vote_history = self.fire_vote_history[-self.fire_vote_frames:]
        label_counts = {}
        for item in self.fire_vote_history:
            if item.get('status') == 'UNKNOWN':
                continue
            lbl = item['label']
            label_counts[lbl] = label_counts.get(lbl, 0) + 1
        best_label = max(label_counts, key=label_counts.get) if label_counts else None
        win_votes = label_counts.get(best_label, 0)
        required = int(np.ceil(2.0*self.fire_vote_frames/3.0))
        agreed = len(self.fire_vote_history) >= self.fire_vote_frames and win_votes >= required
        rep_item = next((item for item in reversed(self.fire_vote_history) if item['label'] == best_label), cur_info)
        def is_road(item):
            return item.get('category') == 'ROAD' or item['label'].endswith('_ROAD')
        # Road has the lowest priority: a recent water/person/fire/unknown
        # observation blocks road confirmation even when road has more votes.
        if is_road(rep_item) and not all(is_road(item) and item.get('status') != 'UNKNOWN' for item in self.fire_vote_history):
            agreed = False
        self.fire_finalized = bool(agreed)
        if agreed:
            voted_info = dict(rep_item)
        else:
            voted_info = dict(cur_info)
            voted_info.update(status='UNKNOWN', label='UNKNOWN', scene_label='UNKNOWN',
                              is_abnormal=None, confidence=0.0, abnormal_score=None,
                              reason='ConfirmingClass:{}/{}'.format(win_votes, self.fire_vote_frames))
        voted_info['votes'] = "{}/{}".format(win_votes, len(self.fire_vote_history))
        self.cached_fire_info = voted_info
        return self.cached_fire_info

    def _normalize_lighting_fast(self, small_bgr):
        b, g, r = cv2.split(small_bgr)
        mb = cv2.mean(b)[0]
        mg = cv2.mean(g)[0]
        mr = cv2.mean(r)[0]
        m_gray = (mb + mg + mr) / 3.0
        if mb > 1.0 and mg > 1.0 and mr > 1.0:
            gb = min(max(1.0 + 0.35 * ((m_gray / mb) - 1.0), 0.82), 1.22)
            gg = min(max(1.0 + 0.35 * ((m_gray / mg) - 1.0), 0.82), 1.22)
            gr = min(max(1.0 + 0.35 * ((m_gray / mr) - 1.0), 0.82), 1.22)
            if abs(gb - 1.0) > 0.03 or abs(gr - 1.0) > 0.03:
                b = cv2.convertScaleAbs(b, alpha=gb, beta=0)
                g = cv2.convertScaleAbs(g, alpha=gg, beta=0)
                r = cv2.convertScaleAbs(r, alpha=gr, beta=0)
                small_wb = cv2.merge([b, g, r])
            else:
                small_wb = small_bgr
        else:
            small_wb = small_bgr

        hsv = cv2.cvtColor(small_wb, cv2.COLOR_BGR2HSV)
        h_chan, s_chan, v_chan = cv2.split(hsv)

        sh, sw = v_chan.shape[:2]
        mid_v = cv2.mean(v_chan[int(sh * 0.15):int(sh * 0.75), :])[0]
        if mid_v < 115.0 or mid_v > 180.0 or self.last_box is not None:
            ref_v = min(mid_v, 70.0) if self.last_box is not None else mid_v
            gamma = np.log(max(ref_v, 15.0) / 255.0) / np.log(135.0 / 255.0)
            g_key = int(round(min(max(gamma, 0.4), 2.2) * 10.0))
            if g_key % 2 == 1:
                g_key += 1
            lut = self._gamma_luts.get(g_key)
            if lut is not None:
                v_chan = cv2.LUT(v_chan, lut)

        if self.clahe is not None:
            v_chan = self.clahe.apply(v_chan)

        return cv2.cvtColor(cv2.merge([h_chan, s_chan, v_chan]), cv2.COLOR_HSV2BGR)

    def _segment_blue_wall_fast(self, blurred, hsv):
        b_ch, g_ch, r_ch = cv2.split(blurred)
        b_minus_r = cv2.subtract(b_ch, r_ch)
        g_minus_b = cv2.subtract(g_ch, b_ch)
        blue_dom = cv2.bitwise_and(
            cv2.threshold(b_minus_r, 18, 255, cv2.THRESH_BINARY)[1],
            cv2.threshold(g_minus_b, 6, 255, cv2.THRESH_BINARY_INV)[1]
        )

        coarse_hsv = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        coarse_blue = cv2.bitwise_and(coarse_hsv, blue_dom)

        if not self.enable_adaptive_blue:
            return coarse_blue

        min_blue_px = max(20, int(blurred.shape[0] * blurred.shape[1] * 0.005))
        if cv2.countNonZero(coarse_blue) < min_blue_px:
            return coarse_blue

        lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
        mean_lab = cv2.mean(lab, mask=coarse_blue)
        mean_hsv = cv2.mean(hsv, mask=coarse_blue)
        ma, mb = int(mean_lab[1]), int(mean_lab[2])
        mh = int(mean_hsv[0])

        low_ab = np.array([0, max(0, ma - self.ab_tol), max(0, mb - self.ab_tol)], dtype=np.uint8)
        high_ab = np.array([255, min(255, ma + self.ab_tol), min(255, mb + self.ab_tol)], dtype=np.uint8)
        ab_ok = cv2.inRange(lab, low_ab, high_ab)

        low_h = np.array([max(0, mh - self.h_tol), 35, 18], dtype=np.uint8)
        high_h = np.array([min(180, mh + self.h_tol), 255, 255], dtype=np.uint8)
        h_ok = cv2.inRange(hsv, low_h, high_h)

        strict_blue = cv2.bitwise_and(coarse_blue, cv2.bitwise_and(ab_ok, h_ok))

        glare_hsv = cv2.inRange(hsv, np.array([84, 0, 170], dtype=np.uint8), np.array([132, 255, 255], dtype=np.uint8))
        b_high = cv2.threshold(b_ch, 209, 255, cv2.THRESH_BINARY)[1]
        br_glare = cv2.threshold(b_minus_r, 12, 255, cv2.THRESH_BINARY)[1]
        spec_glare_blue = cv2.bitwise_and(glare_hsv, cv2.bitwise_and(b_high, br_glare))

        return cv2.bitwise_or(strict_blue, spec_glare_blue)

    def _is_specular_glare(self, small_bgr, x, y, bw, bh):
        if not self.enable_glare_reject:
            return False

        roi_bgr = small_bgr[y:y + bh, x:x + bw]
        if roi_bgr.size == 0 or bw < 4 or bh < 4:
            return True

        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        b_ch, g_ch, r_ch = cv2.split(roi_bgr.astype(np.int16))
        h_ch, s_ch, v_ch = cv2.split(roi_hsv)

        has_dark_ink = (v_ch < 155)
        has_warm_or_green = (r_ch - b_ch > 8) | (g_ch - b_ch > 12) | ((s_ch > 55) & ((h_ch < 82) | (h_ch > 136)))
        non_glare_ratio = float(np.mean(has_dark_ink | has_warm_or_green))
        if non_glare_ratio >= 0.04:
            return False

        glare_core_or_halo = ((v_ch > 180) & (b_ch >= r_ch - 6)) | ((h_ch >= 82) & (h_ch <= 135))
        glare_pixel_ratio = float(np.mean(glare_core_or_halo))

        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        ch, cw = gray.shape[:2]
        center_patch = gray[int(ch * 0.25):max(int(ch * 0.75), 1), int(cw * 0.25):max(int(cw * 0.75), 1)]
        border_mask = np.ones((ch, cw), dtype=bool)
        border_mask[int(ch * 0.20):int(ch * 0.80), int(cw * 0.20):int(cw * 0.80)] = False
        border_pixels = gray[border_mask]

        if center_patch.size > 0 and border_pixels.size > 0:
            center_mean = float(np.mean(center_patch))
            border_mean = float(np.mean(border_pixels))
            if glare_pixel_ratio > 0.92 and (center_mean > 210.0) and (center_mean - border_mean > 12.0):
                return True

        if glare_pixel_ratio > 0.97 and non_glare_ratio < 0.015:
            return True

        return False

    def _find_candidates_in_mask(self, small, blue_raw, inv_scale):
        """返回当前掩码下所有合格的候选目标列表"""
        h, w = small.shape[:2]
        frame_area = float(h * w)
        min_area = frame_area * self.min_area_ratio
        max_area = frame_area * self.max_area_ratio

        blue_closed = cv2.morphologyEx(blue_raw, cv2.MORPH_CLOSE, self.k_close)
        non_blue = cv2.bitwise_not(blue_closed)

        y_top = int(h * self.roi_y_min)
        y_bot = int(h * self.roi_y_max)
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        roi_mask[y_top:y_bot, :] = 255

        mask_standard = cv2.morphologyEx(cv2.bitwise_and(non_blue, roi_mask), cv2.MORPH_OPEN, self.k_open)

        bridge_w = max(int(w * 0.34), 16)
        k_horiz = cv2.getStructuringElement(cv2.MORPH_RECT, (bridge_w, 3))
        blue_bridged = cv2.morphologyEx(blue_closed, cv2.MORPH_CLOSE, k_horiz)
        mask_bridged = cv2.morphologyEx(
            cv2.bitwise_and(mask_standard, blue_bridged), cv2.MORPH_OPEN, self.k_open
        )

        mask_repaired = self._repair_broken_photo_mask(small, blue_bridged, mask_bridged)

        cnts_std, _ = find_contours_compat(mask_standard, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts_brg, _ = find_contours_compat(mask_bridged, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts_rep, _ = find_contours_compat(mask_repaired, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_merged = self._merge_nearby_fragments(cnts_brg)
        all_cnts = list(cnts_std) + list(cnts_brg) + list(cnts_rep) + cnts_merged

        valid_candidates = []
        far_candidate = None
        rejected_glare_boxes = []

        for c in all_cnts:
            hull = cv2.convexHull(c)
            area = cv2.contourArea(hull)
            if area < min_area * 0.15 or area > max_area:
                continue

            x, y, bw, bh = cv2.boundingRect(hull)
            if bh <= 0 or bw <= 0:
                continue

            aspect = float(bw) / float(bh)
            if aspect < self.min_aspect_ratio or aspect > self.max_aspect_ratio:
                continue

            min_rect = cv2.minAreaRect(hull)
            rw, rh = min_rect[1]
            rect_area = float(rw * rh)
            if rect_area <= 1.0:
                continue
            rectangularity = area / rect_area
            if rectangularity < self.min_rectangularity:
                continue

            poly_pts_small, quad_fit = fit_quad_to_hull(hull)
            if poly_pts_small is None or quad_fit < self.min_quad_fit:
                continue

            (l_b, r_b, t_b, b_b), (l_v, r_v, t_v, b_v), incomplete_obs = \
                self._compute_surround_blue_ratios_quad(blue_closed, poly_pts_small)

            avg_blue = (l_b + r_b + t_b + b_b) / 4.0
            active_sides = sum([1 for ratio in (l_b, r_b, t_b, b_b) if ratio >= 0.30])
            valid_sides = sum([1 for v in (l_v, r_v, t_v, b_v) if v >= self.min_edge_valid_ratio])

            if l_v < self.min_edge_valid_ratio or r_v < self.min_edge_valid_ratio or valid_sides < 3:
                continue
            if l_b < self.min_lr_blue_ratio or r_b < self.min_lr_blue_ratio:
                continue
            if avg_blue < self.min_avg_blue_ratio or active_sides < 3:
                continue

            if self._is_archway_opening(blue_closed, x, y, bw, bh, b_b, t_b):
                continue

            if self._is_specular_glare(small, x, y, bw, bh):
                ox, oy, obw, obh = int(x * inv_scale), int(y * inv_scale), int(bw * inv_scale), int(bh * inv_scale)
                if not any(abs(ox - gx) < 15 and abs(oy - gy) < 15 for (gx, gy, _, _) in rejected_glare_boxes):
                    rejected_glare_boxes.append((ox, oy, obw, obh))
                continue

            obs_coverage = float(l_v + r_v + t_v + b_v) / 4.0
            obs_penalty = (0.70 + 0.30 * min(l_v, r_v, t_v, b_v)) if incomplete_obs else 1.0
            score = (quad_fit * 0.5 + avg_blue * 0.5) * obs_penalty

            orig_bbox = (
                int(round(x * inv_scale)),
                int(round(y * inv_scale)),
                int(round(bw * inv_scale)),
                int(round(bh * inv_scale))
            )
            orig_poly = order_quad_points(poly_pts_small * inv_scale)

            cand_info = {
                'bbox': orig_bbox,
                'poly': np.int32(np.round(orig_poly)),
                'area': area * (inv_scale ** 2),
                'area_ratio': area / frame_area,
                'rectangularity': rectangularity,
                'quad_fit': quad_fit,
                'surround_blue': (l_b, r_b, t_b, b_b),
                'valid_ratios': (l_v, r_v, t_v, b_v),
                'obs_coverage': obs_coverage,
                'incomplete_obs': incomplete_obs,
                'score': score
            }

            if area < min_area:
                if far_candidate is None or score > far_candidate['score']:
                    far_candidate = cand_info
                continue

            merged = False
            for idx, existing in enumerate(valid_candidates):
                if compute_bbox_iou(existing['bbox'], orig_bbox) > 0.55:
                    if (cand_info['area'] > existing['area'] * 1.12 and score >= existing['score'] - 0.05) or (score > existing['score'] and cand_info['area'] >= existing['area'] * 0.88):
                        valid_candidates[idx] = cand_info
                    merged = True
                    break
            if not merged:
                valid_candidates.append(cand_info)

        valid_candidates.sort(key=lambda item: item['score'], reverse=True)
        return valid_candidates, far_candidate, rejected_glare_boxes, blue_closed, mask_bridged

    @staticmethod
    def _merge_candidate_lists(base_list, extra_list):
        merged_list = list(base_list)
        for cand in extra_list:
            found = False
            for idx, existing in enumerate(merged_list):
                if compute_bbox_iou(existing['bbox'], cand['bbox']) > 0.55:
                    if (cand['area'] > existing['area'] * 1.12 and cand['score'] >= existing['score'] - 0.05) or (cand['score'] > existing['score'] and cand['area'] >= existing['area'] * 0.88):
                        merged_list[idx] = cand
                    found = True
                    break
            if not found:
                merged_list.append(cand)
        merged_list.sort(key=lambda item: item['score'], reverse=True)
        return merged_list

    def _select_target_from_candidates(self, candidates):
        """优先匹配已有目标 (self.last_box)，失去后再选最高分新目标"""
        if not candidates:
            return None, False

        if self.last_box is not None:
            matched_cands = [c for c in candidates if self._is_same_target(c['bbox'])]
            if matched_cands:
                matched_cands.sort(
                    key=lambda c: compute_bbox_iou(self.last_box['bbox'], c['bbox']),
                    reverse=True
                )
                chosen = matched_cands[0]
                chosen['poly'] = np.int32(np.round(
                    order_quad_points(chosen['poly'], prev_poly=self.last_box['poly'])
                ))
                return chosen, True
            else:
                return candidates[0], False

        return candidates[0], False

    def detect_frame(self, frame):
        orig_h, orig_w = frame.shape[:2]
        if orig_w > self.proc_width:
            scale = float(self.proc_width) / float(orig_w)
            proc_h = max(int(round(orig_h * scale)), 1)
            small = cv2.resize(frame, (self.proc_width, proc_h), interpolation=cv2.INTER_NEAREST)
            inv_scale = 1.0 / scale
        else:
            small = frame
            inv_scale = 1.0

        h, w = small.shape[:2]
        frame_area = float(h * w)

        blurred_orig = cv2.GaussianBlur(small, (3, 3), 0)
        hsv_orig = cv2.cvtColor(blurred_orig, cv2.COLOR_BGR2HSV)
        blue_orig = self._segment_blue_wall_fast(blurred_orig, hsv_orig)
        blue_ratio_orig = float(cv2.countNonZero(blue_orig)) / frame_area

        candidates = []
        far_candidate = None
        rejected_glare_boxes = []
        blue_closed = blue_orig
        mask_bridged = blue_orig

        paper_candidates = self._find_paper_candidates(small, blue_orig, inv_scale, frame) if blue_ratio_orig >= .05 else []
        if paper_candidates:
            candidates = paper_candidates
        elif blue_ratio_orig >= 0.05:
            candidates, far_candidate, rejected_glare_boxes, blue_closed, mask_bridged = \
                self._find_candidates_in_mask(small, blue_orig, inv_scale)

        selected_cand, matched_existing = self._select_target_from_candidates(candidates)

        need_light_retry = not paper_candidates and ((not candidates) or (self.last_box is not None and not matched_existing))
        if need_light_retry and self.enable_light_norm:
            mean_v = cv2.mean(hsv_orig[:, :, 2])[0]
            if mean_v < 110.0 or blue_ratio_orig >= 0.02 or self.last_box is not None:
                norm_small = self._normalize_lighting_fast(small)
                blurred_norm = cv2.GaussianBlur(norm_small, (3, 3), 0)
                hsv_norm = cv2.cvtColor(blurred_norm, cv2.COLOR_BGR2HSV)
                blue_norm = self._segment_blue_wall_fast(blurred_norm, hsv_norm)
                blue_combined = cv2.bitwise_or(blue_norm, blue_orig)

                if float(cv2.countNonZero(blue_combined)) / frame_area >= 0.05:
                    cands_norm, far_n, glare_n, _, _ = self._find_candidates_in_mask(norm_small, blue_norm, inv_scale)
                    cands_comb, far_c, glare_c, blue_closed, mask_bridged = self._find_candidates_in_mask(norm_small, blue_combined, inv_scale)
                    extra_cands = self._merge_candidate_lists(cands_norm, cands_comb)
                    candidates = self._merge_candidate_lists(candidates, extra_cands)

                    if far_candidate is None:
                        far_candidate = far_c if far_c is not None else far_n
                    for gbox in (glare_n + glare_c):
                        if not any(abs(gbox[0] - gx) < 15 and abs(gbox[1] - gy) < 15 for (gx, gy, _, _) in rejected_glare_boxes):
                            rejected_glare_boxes.append(gbox)

                    selected_cand, matched_existing = self._select_target_from_candidates(candidates)

        if selected_cand is not None:
            if self.last_box is not None and not matched_existing:
                if self.locked and self.lost_count < self.max_lost_frames:
                    self.lost_count += 1
                    result = {
                        'status': 'LOST_HOLD',
                        'hit_count': self.hit_count,
                        'lost_count': self.lost_count,
                        'fire_finalized': self.fire_finalized,
                        'lock_ready': False,
                        'target': None,
                        'rejected_glare': rejected_glare_boxes
                    }
                    return result, {'blue_mask': blue_closed, 'candidate_mask': mask_bridged}
                else:
                    self.reset_tracking()

            prev_poly_ref = self.last_box['poly'] if (self.last_box is not None and matched_existing) else None
            if selected_cand.get('paper_boundary'):
                ref_poly = np.int32(np.round(order_quad_points(selected_cand['poly'], prev_poly=prev_poly_ref)))
                ref_bbox = cv2.boundingRect(ref_poly)
            else:
                ref_bbox, ref_poly = self._refine_quad_on_orig_roi(
                    frame, selected_cand['bbox'], selected_cand['poly'], prev_poly=prev_poly_ref
                )
            selected_cand['bbox'] = ref_bbox
            selected_cand['poly'] = ref_poly

            self.lost_count = 0
            self.hit_count += 1
            self.last_box = selected_cand
            if self.hit_count >= self.trigger_frames:
                self.locked = True
            status = 'LOCKED' if self.locked else 'DETECTING'
            active_target = selected_cand

            if status == 'LOCKED' and self.classify_fire:
                active_target['fire_info'] = self._update_fire_classification(frame, active_target['poly'])
        else:
            if self.locked and self.lost_count < self.max_lost_frames:
                self.lost_count += 1
                status = 'LOST_HOLD'
                active_target = None
            else:
                self.reset_tracking()
                status = 'FAR' if far_candidate is not None else 'NONE'
                active_target = far_candidate

        lock_ready = (status == 'LOCKED') and (self.fire_finalized if self.classify_fire else True)

        result = {
            'status': status,
            'hit_count': self.hit_count,
            'lost_count': self.lost_count,
            'fire_finalized': self.fire_finalized,
            'lock_ready': lock_ready,
            'target': active_target,
            'rejected_glare': rejected_glare_boxes
        }
        return result, {'blue_mask': blue_closed, 'candidate_mask': mask_bridged}

    @staticmethod
    def draw_result(frame, result, show_rejected_glare=True):
        """自适应分辨率安全绘制 (支持 8 类场景：正常类绿框，异常类红框)"""
        vis = frame.copy()
        h, w = vis.shape[:2]
        status = result['status']
        target = result['target']

        bar_h = min(32, max(16, int(h * 0.12)))

        if show_rejected_glare and 'rejected_glare' in result:
            for (gx, gy, gw, gh) in result['rejected_glare']:
                cv2.rectangle(vis, (gx, gy), (gx + gw, gy + gh), (0, 0, 220), 1)
                if h >= 60 and w >= 80:
                    cv2.putText(vis, "GLARE IGNORED", (gx, max(bar_h + 10, gy - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 235), 1)

        if target is not None:
            x, y, bw, bh = target['bbox']
            poly = target['poly']
            line_thick = 1 if min(h, w) < 160 else 3
            pt_radius = 2 if min(h, w) < 160 else 5

            if status == 'FAR':
                cv2.polylines(vis, [poly.reshape(-1, 1, 2)], True, (0, 165, 255), max(1, line_thick - 1))
                if h >= 80 and w >= 120:
                    label = "FAR ({},{}) {}x{}".format(x, y, bw, bh)
                    cv2.putText(vis, label, (max(2, x), max(bar_h + 12, y - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1)
            else:
                fire_info = target.get('fire_info')
                is_abn = False
                if fire_info is not None:
                    is_abn = fire_info.get('is_abnormal', 'ABNORMAL' in fire_info.get('label', ''))

                if fire_info is not None and fire_info.get('status') == 'UNKNOWN':
                    box_color = (0, 190, 255)
                    bg_color = (0, 120, 170)
                elif is_abn:
                    box_color = (0, 0, 255)
                    bg_color = (0, 0, 200)
                else:
                    box_color = (0, 255, 0)
                    bg_color = (0, 160, 0)

                cv2.polylines(vis, [poly.reshape(-1, 1, 2)], True, box_color, line_thick)
                cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 255), 1)
                for pt in poly:
                    cv2.circle(vis, (int(pt[0]), int(pt[1])), pt_radius, (0, 0, 255), -1)
                cx, cy = int(x + bw / 2), int(y + bh / 2)
                cv2.circle(vis, (cx, cy), max(1, pt_radius - 1), (255, 0, 255), -1)

                if fire_info is not None:
                    label = "{} ({:.0f}%)".format(fire_info['label'], fire_info['confidence'] * 100)
                    if w >= 280:
                        label += " [{}]".format(fire_info['reason'])

                    max_thumb_h = h - bar_h - 8
                    max_thumb_w = int(w * 0.30)
                    thumb_sz = min(110, max_thumb_h, max_thumb_w)
                    if thumb_sz >= 24 and fire_info.get('warped') is not None:
                        warped_thumb = cv2.resize(fire_info['warped'], (thumb_sz, thumb_sz))
                        ty1 = bar_h + 4
                        ty2 = ty1 + thumb_sz
                        tx2 = w - 4
                        tx1 = tx2 - thumb_sz
                        if tx1 >= 0 and ty2 <= h:
                            vis[ty1:ty2, tx1:tx2] = warped_thumb
                            cv2.rectangle(vis, (tx1, ty1), (tx2, ty2), box_color, 1)
                else:
                    label = "PIC [{}] ({},{}) {}x{} S:{:.2f}".format(
                        status, x, y, bw, bh, target['score']
                    )

                if h >= 60 and w >= 80:
                    font_scale = 0.36 if w < 220 else 0.46
                    font_thick = 1 if w < 220 else 2
                    lbl_h = 16 if w < 220 else 24
                    ly2 = max(bar_h + lbl_h, y)
                    ly1 = max(bar_h, ly2 - lbl_h)
                    lx1 = max(0, min(x, w - 60))
                    lx2 = min(w, lx1 + int(len(label) * (7 if w < 220 else 9.2)))
                    cv2.rectangle(vis, (lx1, ly1), (lx2, ly2), bg_color, -1)
                    cv2.putText(vis, label, (lx1 + 2, ly2 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thick)

        status_color = (0, 220, 0) if status == 'LOCKED' else (
            (0, 180, 255) if status in ('DETECTING', 'FAR', 'LOST_HOLD') else (120, 120, 120)
        )
        cv2.rectangle(vis, (0, 0), (w, bar_h), (30, 30, 30), -1)
        top_scale = 0.40 if w < 200 else 0.62
        top_thick = 1 if w < 200 else 2
        cv2.putText(vis, "STATUS: {} | HITS: {}".format(status, result['hit_count']),
                    (6, max(12, bar_h - 6)), cv2.FONT_HERSHEY_SIMPLEX, top_scale, status_color, top_thick)
        return vis


# ================= 命令行与低负载巡检主循环 =================
def orient_camera_frame(frame, rotation=180):
    """Rotate camera/video input before both detection and display."""
    return cv2.flip(frame, -1) if rotation == 180 else frame


class DisplayRefresh(object):
    """Simulated preview jitter only; no sleeps, queues or acquisition changes."""
    def __init__(self, hold_seconds, every_seconds, clock=None, rng=None):
        self.hold_seconds=hold_seconds;self.every_seconds=every_seconds
        self.clock=clock or getattr(time,'monotonic',time.time)
        self.rng=rng or random.Random()
        self.next_hold=None;self.hold_until=None
    def ready(self):
        if self.hold_seconds<=0:return True
        now=self.clock()
        if self.hold_until is not None:
            if now<self.hold_until:return False
            self.hold_until=None
            # Sometimes several short interruptions cluster together.
            gap=(self.rng.uniform(.2,.6) if self.rng.random()<.25 else
                 self.rng.uniform(.6,1.7)*self.every_seconds)
            self.next_hold=now+gap
            return True  # Always show the latest frame after a hold.
        if self.next_hold is None:
            self.next_hold=now+self.rng.uniform(.6,1.7)*self.every_seconds
        if now>=self.next_hold:
            factor=(self.rng.uniform(1.6,2.5) if self.rng.random()<.15 else
                    self.rng.triangular(.4,1.4,.7))
            self.hold_until=now+min(1.,self.hold_seconds*factor)
            return False
        return True


def main():
    parser = argparse.ArgumentParser(description="Ultra-Low-CPU Multi-Scene Overhead Picture Detector (v5)")
    parser.add_argument("--source", type=str, default="0",
                        help="Camera index ('0'), video path, or image path")
    parser.add_argument("--save", type=str, default="",
                        help="Optional path to save output image")
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI window")
    parser.add_argument("--fps", type=float, default=5.0,
                        help="Target processing/display rate in Hz (default 5 FPS)")
    parser.add_argument("--width", type=int, default=320,
                        help="Requested camera width (default 320 pixels)")
    parser.add_argument("--height", type=int, default=240,
                        help="Requested camera height (default 240 pixels)")
    parser.add_argument("--rotate", type=int, choices=[0, 180], default=180,
                        help="Camera/video rotation before detection (default 180 degrees; image files unchanged)")
    parser.add_argument("--classifier-config", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'classifier_config.json'),
                        help="Optional JSON file containing classifier thresholds")
    parser.add_argument("--trigger-frames", type=int, default=2,
                        help="Consecutive frames needed to confirm lock (default 2)")
    parser.add_argument("--max-lost-frames", type=int, default=2,
                        help="Grace frames to hold lock during brief dropout after locking (default 2)")
    parser.add_argument("--vote-frames", type=int, default=3,
                        help="Sliding classification window (default 3); two-thirds agreement, road needs all frames to be road")
    parser.add_argument("--no-classify", action="store_true",
                        help="Locate and box the picture on the blue wall ONLY, disabling content classification")
    parser.add_argument("--classify-only", action="store_true",
                        help="Explicitly classify a standalone cropped scene image without blue wall detection")
    parser.add_argument("--exposure", type=float, default=None,
                        help="Optional manual exposure value for V4L2 camera")
    parser.add_argument("--exit-on-lock", action="store_true",
                        help="Automatically exit once a picture is locked (and classified if enabled)")
    parser.add_argument("--display-hold-seconds", type=float, default=.4,
                        help="Typical preview jitter duration, capped at 1 s; 0 disables (default .4 s)")
    parser.add_argument("--display-hold-every", type=float, default=4.,
                        help="Typical preview jitter spacing; randomized with occasional clusters (default 4 s)")
    parser.add_argument("--display-seed", type=int, default=None,
                        help="Optional reproducible preview jitter seed; classification is unaffected")
    args = parser.parse_args()
    if (not np.isfinite(args.display_hold_seconds) or not np.isfinite(args.display_hold_every)
            or args.display_hold_seconds<0 or args.display_hold_every<=0
            or args.display_hold_seconds>=args.display_hold_every):
        parser.error('Display hold requires 0 <= duration < interval, both finite')
    if args.width<=0 or args.height<=0 or not np.isfinite(args.fps) or args.fps<=0:
        parser.error('Camera dimensions and FPS must be positive and finite')

    if hasattr(os, 'nice'):
        try:
            os.nice(10)
        except Exception:
            pass

    classifier_config = {}
    if os.path.isfile(args.classifier_config):
        with open(args.classifier_config) as config_file:
            classifier_config = json.load(config_file)
        if not isinstance(classifier_config, dict):
            parser.error('Classifier configuration must be a JSON object')
    detector = OverheadPicDetector({
        'classifier_config': classifier_config,
        'trigger_frames': args.trigger_frames,
        'max_lost_frames': args.max_lost_frames,
        'fire_vote_frames': args.vote_frames,
        'classify_fire': not args.no_classify
    })

    ext = os.path.splitext(args.source)[1].lower()
    if ext in ['.jpg', '.jpeg', '.png', '.bmp']:
        frame = cv2.imread(args.source)
        if frame is None:
            print("[ERROR] Cannot read image:", args.source)
            sys.exit(1)

        if args.classify_only:
            f_info = detector.fire_classifier.classify_patch(frame)
            print("[CLASSIFY-ONLY] Category={}, Status={}, Label={} ({:.0f}%) Reason={}".format(
                f_info.get('category', 'N/A'), f_info.get('status', 'N/A'),
                f_info['label'], f_info['confidence'] * 100, f_info['reason']))
            if args.save:
                vis = frame.copy()
                cv2.putText(vis, "CLASSIFY-ONLY: {} ({:.0f}%)".format(f_info['label'], f_info['confidence'] * 100),
                            (6, min(28, frame.shape[0] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.imwrite(args.save, vis)
            return

        detector.trigger_frames = 1
        detector.fire_vote_frames = 1
        res, _ = detector.detect_frame(frame)
        vis = detector.draw_result(frame, res)
        if res['target'] is not None and res['status'] == 'LOCKED':
            t = res['target']
            if detector.classify_fire and 'fire_info' in t:
                f_info = t['fire_info']
                print("[{}] BBox={}, Score={:.2f}, Category={}, Label={} ({:.0f}%) Reason={}".format(
                    res['status'], t['bbox'], t['score'], f_info.get('category', 'N/A'),
                    f_info.get('label', 'N/A'), f_info.get('confidence', 0) * 100, f_info.get('reason', '')))
            else:
                print("[{}] BBox={}, QuadFit={:.2f}, Score={:.2f} (Locate-Only Mode)".format(
                    res['status'], t['bbox'], t.get('quad_fit', 0.0), t['score']))
        else:
            print("[INFO] Wall picture NOT locked. Status: {}, RejectedGlare: {}".format(
                res['status'], len(res.get('rejected_glare', []))))

        if args.save:
            cv2.imwrite(args.save, vis)
            print("[INFO] Saved result to:", args.save)
        if not args.headless:
            cv2.imshow("Overhead Picture Detector", vis)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print("[ERROR] Cannot open video source:", args.source)
        sys.exit(1)

    set_cam_props_compat(cap, args.width, args.height, exposure=args.exposure, fps=args.fps)
    print('[INFO] Camera request={}x{}; processing/display={:.1f} FPS; device reports={}x{} @ {:.1f} FPS'.format(
        args.width,args.height,args.fps,int(cap.get(getattr(cv2,'CAP_PROP_FRAME_WIDTH',3))),
        int(cap.get(getattr(cv2,'CAP_PROP_FRAME_HEIGHT',4))),cap.get(getattr(cv2,'CAP_PROP_FPS',5))))
    frame_interval = 1.0 / max(args.fps, 1.0)
    print("[INFO] Low-CPU loop started at {:.1f} FPS (classify_scene={}).".format(
        args.fps, detector.classify_fire))
    saved_once = False
    display_refresh=DisplayRefresh(args.display_hold_seconds,args.display_hold_every,rng=random.Random(args.display_seed))
    if not args.headless and args.display_hold_seconds>0:
        print('[SIMULATED DISPLAY JITTER] Irregular preview holds: typical {:.2f}s, spacing about {:.2f}s; detection continues.'.format(
            args.display_hold_seconds,args.display_hold_every))

    while True:
        t_start = time.time()

        if isinstance(src, int) and args.fps <= 12.0:
            cap.grab()

        ret, frame = cap.read()
        if not ret or frame is None:
            break

        frame = orient_camera_frame(frame, args.rotate)
        res, _ = detector.detect_frame(frame)
        proc_ms = (time.time() - t_start) * 1000.0

        if (not args.headless) or (res['lock_ready'] and args.save and not saved_once):
            vis = detector.draw_result(frame, res)
            if vis.shape[1] >= 240:
                cv2.putText(vis, "CPU: {:.1f}ms".format(proc_ms), (vis.shape[1] - 125, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 255, 200), 1)
        else:
            vis = None

        if res['status'] == 'LOCKED' and res['target'] is not None:
            t = res['target']
            if detector.classify_fire and 'fire_info' in t:
                f_info = t['fire_info']
                print("[LOCKED] BBox={} | Scene={} ({:.0f}%, votes={}) | Ready={} | CPU={:.1f}ms".format(
                    t['bbox'], f_info.get('label', 'N/A'), f_info.get('confidence', 0) * 100,
                    f_info.get('votes', '1/1'), res['lock_ready'], proc_ms))
            else:
                print("[LOCKED] BBox={} | Score={:.2f} (Locate-Only) | Ready={} | CPU={:.1f}ms".format(
                    t['bbox'], t['score'], res['lock_ready'], proc_ms))

            if res['lock_ready']:
                if args.save and not saved_once and vis is not None:
                    cv2.imwrite(args.save, vis)
                    print("[INFO] Saved locked frame to:", args.save)
                    saved_once = True
                if args.exit_on_lock:
                    break

        if not args.headless and vis is not None:
            if display_refresh.ready():
                cv2.imshow("Overhead Picture Detector", vis)
            # Pump GUI events even while holding the last frame; q/Esc stay responsive.
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

        elapsed = time.time() - t_start
        sleep_sec = frame_interval - elapsed
        if sleep_sec > 0.002:
            time.sleep(sleep_sec)

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
