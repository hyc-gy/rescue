"""MaixCAM：2027 智能救援视觉识别（独立入口，旧 main.py 保留）。

规则依据《智能+.pdf》第16—19页：
普通物资=绿色40mm正方体；核心物资=黑色40mm正三棱锥；
伤员=橘色80x40x40mm长方体；危险物=浅蓝色40mm正方体。
前三类均为双方公共目标，红蓝只用于区分安全区。决赛类别需现场调整。

部署前必须训练/导出包含下面 LABELS 的 YOLOv5 Maix 模型，设置 MODEL_PATH。
不能把旧七分类模型的 class_id 按此表重新解释；缺少新模型时拒绝启动。
分区标注应覆盖各半区内部，含不同观察方向；同色两半区需利用隔板和布局。
检测框只提供观察结果，不代表允许推送；入口、隔板及停车检查由后续控制实现。

新协议与旧 55 AA 协议隔离，现有 STM32 maixcam.c 需另行增加解析：
命令：A5 5A + version(1) + mode(1) + team(1) + CRC16(2)，共7字节。
协议version升级为3，命令和结果都使用此版本。
结果：A5 5B + !BBBBBBBBBBHHHHBB + CRC16(2)，共24字节，大端。
字段：version,seq,mode,team,type,count,hazard_count,normal_count,core_count,
      patient_count,cx,cy,w,h,score,valid。
CRC16/CCITT-FALSE：初值FFFF、多项式1021，覆盖帧头和正文，CRC大端。
mode: 0全部观察，1普通物资，2核心物资，3伤员，4保留（不再接受），5己方物资区，
      6己方伤员区，7全部救援目标（普通物资+核心物资+伤员），
      8混合物资（普通物资+核心物资），9运输物品确认。
team: 0未配置，1蓝方，2红方。type取 TargetType，0=未发现。
count是当前模式可见数量，不是载荷数、完成数或允许一次搬运数量。
危险物永不参与目标选择及count；普通物资、核心物资、伤员也另行分类统计。
搜索模式(mode 0-8)的目标总数、三类数量及hazard_count均只统计SEARCH_ROI内目标中心。
确认模式(mode 9)只统计CONFIRM_ROI这一个区域，且不输出导航目标。
CONFIRM_ROI使用归一化(左,上,右,下)参数，由用户按新相机角度及夹具视野调整；
STM32需检查帧新鲜度及hazard_count，不能因valid=0忽略危险物数据。
遮挡、漏检仍可能发生，单帧类别计数为0不等于已确认抓取失败或成功；
建议连续多帧稳定后由STM32确认物品类别，三类总数及危险物数量保持独立。
valid只表示本帧存在检测目标，不表示转运合法或已经送达。
首次只运一个普通物资、伤员单独一个、物资最多三个等由STM32任务管理。
建议阶段：首次用mode=1；首次普通物资送达确认后用mode=7搜寻下一目标；
选定物资后可用mode=8补充物资，选定伤员后用mode=3单独转运。
所有模式按中心坐标加权选择一个目标并返回其真实type，count为合计可见数量。
横向居中权重0.65，纵向靠近权重0.35；使用实际图像宽高归一化。
固定相机姿态下将中心y越大视为越近，不将它当作实际测距。
mode=7不允许物资与伤员混运；mode=8不自动限制载荷为三个。
"""

import os
import struct


MODEL_PATH = "/root/models/rescue_2027/model.mud"
# 必须与模型内置标签顺序严格一致。语义由训练数据决定，颜色仅为初赛提示。
LABELS = (
    "normal_material", "core_material", "patient", "hazard",
    "blue_material_zone", "blue_patient_zone",
    "red_material_zone", "red_patient_zone",
)
CONF_THRESHOLD = 0.70
IOU_THRESHOLD = 0.45
# 分数越小越优：优先减少转向，再考虑靠近程度。权重无需随分辨率调整。
TARGET_X_WEIGHT = 0.65
TARGET_Y_WEIGHT = 0.35
# 搜索阶段的目标选择和所有数量统计均使用此区域；参数为归一化(左,上,右,下)。
SEARCH_ROI = (0.0, 0.0, 1.0, 1.0)
# 确认阶段只使用此区域。请根据相机和夹具的实际画面修改这四个参数。
CONFIRM_ROI = (0.0, 0.3125, 1.0, 1.0)
VERSION = 3
RESULT_FMT = "!BBBBBBBBBBHHHHBB"
COMMAND_HEADER = b"\xA5\x5A"
RESULT_HEADER = b"\xA5\x5B"


class TargetType:
    NONE = 0
    NORMAL_MATERIAL = 1
    CORE_MATERIAL = 2
    PATIENT = 3
    HAZARD = 4
    BLUE_MATERIAL_ZONE = 5
    BLUE_PATIENT_ZONE = 6
    RED_MATERIAL_ZONE = 7
    RED_PATIENT_ZONE = 8


class Mode:
    OBSERVE = 0
    NORMAL_MATERIAL = 1
    CORE_MATERIAL = 2
    PATIENT = 3
    MATERIAL_ZONE = 5
    PATIENT_ZONE = 6
    RESCUE_TARGETS = 7  # 普通物资、核心物资、伤员，排除危险物和安全区。
    MIXED_MATERIALS = 8  # 普通物资与核心物资可混合转运。
    CONFIRM_CARGO = 9  # 只统计CONFIRM_ROI内类别，不输出导航目标。


TYPE_BY_LABEL = {name: index + 1 for index, name in enumerate(LABELS)}
VALID_MODES = (0, 1, 2, 3, 5, 6, 7, 8, 9)


def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def with_crc(data):
    return data + struct.pack("!H", crc16(data))


class CommandParser:
    """保留残帧，处理一次读取多帧及错位数据；只接受有效CRC和参数。"""

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        commands = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != COMMAND_HEADER:
                del self.buffer[0]
                continue
            if len(self.buffer) < 7:
                break
            frame = bytes(self.buffer[:7])
            if crc16(frame[:5]) != struct.unpack("!H", frame[5:])[0]:
                del self.buffer[0]
                continue
            del self.buffer[:7]
            version, mode, team = frame[2:5]
            if version == VERSION and mode in VALID_MODES and team in (0, 1, 2):
                commands.append((mode, team))
        return commands


def allowed_types(mode, team):
    if mode == Mode.CONFIRM_CARGO:
        return set()
    if mode == Mode.OBSERVE:
        return set(TYPE_BY_LABEL.values()) - {TargetType.HAZARD}
    if mode in (Mode.NORMAL_MATERIAL, Mode.CORE_MATERIAL, Mode.PATIENT):
        return {mode}
    if mode == Mode.RESCUE_TARGETS:
        return {TargetType.NORMAL_MATERIAL, TargetType.CORE_MATERIAL, TargetType.PATIENT}
    if mode == Mode.MIXED_MATERIALS:
        return {TargetType.NORMAL_MATERIAL, TargetType.CORE_MATERIAL}
    if team not in (1, 2):
        return set()  # 未选队伍时不猜测己方安全区。
    if mode == Mode.MATERIAL_ZONE:
        return {TargetType.BLUE_MATERIAL_ZONE if team == 1 else TargetType.RED_MATERIAL_ZONE}
    if mode == Mode.PATIENT_ZONE:
        return {TargetType.BLUE_PATIENT_ZONE if team == 1 else TargetType.RED_PATIENT_ZONE}
    return set()


def target_position_cost(obj, image_width, image_height):
    """中心到画面中线的归一化偏差 + 中心到画面底部的归一化距离。"""
    cx = obj.x + obj.w / 2
    cy = obj.y + obj.h / 2
    x_error = min(1.0, abs(cx - image_width / 2) / (image_width / 2))
    y_nearness = max(0.0, min(1.0, cy / image_height))
    return TARGET_X_WEIGHT * x_error + TARGET_Y_WEIGHT * (1.0 - y_nearness)


def point_in_roi(cx, cy, image_width, image_height, roi):
    left, top, right, bottom = roi
    return (left <= cx / image_width <= right
            and top <= cy / image_height <= bottom)


def select_target(objects, mode, team, image_width, image_height, roi=SEARCH_ROI):
    if image_width <= 0 or image_height <= 0:
        raise ValueError("图像宽高必须大于0")
    left, top, right, bottom = roi
    if not (0 <= left < right <= 1 and 0 <= top < bottom <= 1):
        raise ValueError("目标选择ROI无效")
    allowed = allowed_types(mode, team)
    candidates = [o for o in objects if 0 <= o.class_id < len(LABELS)
                  and o.score >= CONF_THRESHOLD and o.w > 0 and o.h > 0
                  and TYPE_BY_LABEL[LABELS[o.class_id]] in allowed
                  and point_in_roi(o.x + o.w / 2, o.y + o.h / 2,
                                   image_width, image_height, roi)]
    # 几何分数相同时选y更大者，再比较置信度；各类别按相同规则竞争。
    candidates.sort(key=lambda o: (
        target_position_cost(o, image_width, image_height),
        -(o.y + o.h / 2), -o.score,
    ))
    return candidates[0] if candidates else None, min(len(candidates), 255)


def u16(value):
    return max(0, min(65535, int(value)))


def count_types_in_roi(objects, image_width, image_height, roi=SEARCH_ROI):
    """统计ROI内三类目标和危险物，不受导航模式筛选影响。"""
    left, top, right, bottom = roi
    if image_width <= 0 or image_height <= 0 or not (0 <= left < right <= 1 and 0 <= top < bottom <= 1):
        raise ValueError("运输检查区域或图像尺寸无效")
    counts = {
        TargetType.NORMAL_MATERIAL: 0,
        TargetType.CORE_MATERIAL: 0,
        TargetType.PATIENT: 0,
        TargetType.HAZARD: 0,
    }
    for obj in objects:
        if (0 <= obj.class_id < len(LABELS) and obj.score >= CONF_THRESHOLD
                and obj.w > 0 and obj.h > 0):
            kind = TYPE_BY_LABEL[LABELS[obj.class_id]]
            if kind not in counts:
                continue
            if point_in_roi(obj.x + obj.w / 2, obj.y + obj.h / 2,
                            image_width, image_height, roi):
                counts[kind] += 1
    return {kind: min(count, 255) for kind, count in counts.items()}


def result_packet(sequence, mode, team, target, count, cargo_counts):
    # 每帧重建结果，无目标时坐标、类别、数量全部清零，避免沿用上帧。
    kind = x = y = w = h = score = valid = 0
    if target is not None:
        kind = TYPE_BY_LABEL[LABELS[target.class_id]]
        x, y = u16(target.x + target.w / 2), u16(target.y + target.h / 2)
        w, h = u16(target.w), u16(target.h)
        score = max(0, min(100, int(target.score * 100)))
        valid = 1
    else:
        count = 0
    body = struct.pack(RESULT_FMT, VERSION, sequence & 255, mode, team,
                       kind, count,
                       *(max(0, min(255, int(cargo_counts[rescue_type])))
                         for rescue_type in (TargetType.HAZARD, TargetType.NORMAL_MATERIAL,
                                             TargetType.CORE_MATERIAL, TargetType.PATIENT)),
                       x, y, w, h, score, valid)
    return with_crc(RESULT_HEADER + body)


def main():
    from maix import camera, display, image, nn, app, uart, err, pinmap, time

    if not os.path.isfile(MODEL_PATH):
        raise RuntimeError("缺少新版模型，请训练八分类模型并设置 MODEL_PATH；不能使用旧七分类模型")
    detector = nn.YOLOv5(model=MODEL_PATH)
    model_labels = getattr(detector, "labels", None)
    if model_labels is None or tuple(model_labels) != LABELS:
        raise RuntimeError("模型标签与 LABELS 不一致，请检查模型导出标签和顺序")

    for pin, function in (("A16", "UART0_TX"), ("A17", "UART0_RX")):
        err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping failed")
    serial = uart.UART("/dev/ttyS0", 115200)
    cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
    screen = display.Display()
    parser = CommandParser()
    mode, team, sequence = Mode.OBSERVE, 0, 0

    while not app.need_exit():
        chunk = serial.read(256)
        if chunk:
            for mode, team in parser.feed(chunk):
                pass  # 使用最后一条有效命令；不在视觉端修改任务完成状态。
        frame = cam.read()
        # 在绘图前推理，避免文字和检测框污染模型输入。
        objects = detector.detect(frame, conf_th=CONF_THRESHOLD, iou_th=IOU_THRESHOLD)
        active_roi = CONFIRM_ROI if mode == Mode.CONFIRM_CARGO else SEARCH_ROI
        target, count = select_target(objects, mode, team, frame.width(), frame.height(), active_roi)
        cargo_counts = count_types_in_roi(objects, frame.width(), frame.height(), active_roi)
        serial.write(result_packet(sequence, mode, team, target, count, cargo_counts))
        sequence = (sequence + 1) & 255

        for obj in objects:
            if not 0 <= obj.class_id < len(LABELS):
                continue
            color = image.COLOR_GREEN if obj is target else image.COLOR_RED
            frame.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color)
            frame.draw_string(obj.x, obj.y, "%s %.2f" % (LABELS[obj.class_id], obj.score), color=color)
        roi_color = image.COLOR_GREEN if mode == Mode.CONFIRM_CARGO else image.COLOR_WHITE
        roi_x = int(active_roi[0] * frame.width())
        roi_y = int(active_roi[1] * frame.height())
        roi_w = int((active_roi[2] - active_roi[0]) * frame.width())
        roi_h = int((active_roi[3] - active_roi[1]) * frame.height())
        frame.draw_rect(roi_x, roi_y, roi_w, roi_h, color=roi_color, thickness=2)
        frame.draw_string(0, 0, "Mode:%d T:%d N:%d C:%d P:%d H:%d" % (
            mode, team, count, cargo_counts[TargetType.NORMAL_MATERIAL],
            cargo_counts[TargetType.CORE_MATERIAL], cargo_counts[TargetType.PATIENT],
            cargo_counts[TargetType.HAZARD]), color=image.COLOR_WHITE)
        screen.show(frame)
        time.sleep_ms(5)


if __name__ == "__main__":
    main()
