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

文件结构（从上到下）：
  1) 可调参数与协议常量
  2) TargetType / Mode 枚举
  3) CRC16、命令帧解析（CommandParser）
  4) 目标筛选与打分（allowed_types / select_target）
  5) 结果帧打包（result_packet）
  6) 主循环（main）：收命令 → 推理 → 选目标 → 统计 → 回传 → 画图
"""

import os       # 启动前检查模型文件是否存在
import struct   # 协议帧的二进制打包/解包，统一使用网络字节序（大端）


# ============================================================================
# 1) 可调参数
# ============================================================================

MODEL_PATH = "/root/models/rescue_2027/model.mud"
# 必须与模型内置标签顺序严格一致。语义由训练数据决定，颜色仅为初赛提示。
# 注意：class_id 是这里的下标，协议里回传的 type = class_id + 1（见 TYPE_BY_LABEL）。
LABELS = (
    "normal_material", "core_material", "patient", "hazard",
    "blue_material_zone", "blue_patient_zone",
    "red_material_zone", "red_patient_zone",
)
CONF_THRESHOLD = 0.70   # YOLO 置信度阈值：低于该值的检测框不参与选择，也不计入数量
IOU_THRESHOLD = 0.45    # NMS 的 IoU 阈值：越大保留的重复框越多，越小删得越狠
# 分数越小越优：优先减少转向，再考虑靠近程度。权重无需随分辨率调整。
TARGET_X_WEIGHT = 0.65  # 横向居中项权重（偏离画面中线越远，代价越大）
TARGET_Y_WEIGHT = 0.35  # 纵向靠近项权重（中心越靠画面下方，代价越小）
# 搜索阶段的目标选择和所有数量统计均使用此区域；参数为归一化(左,上,右,下)。
SEARCH_ROI = (0.0, 0.0, 1.0, 1.0)
# 确认阶段只使用此区域。请根据相机和夹具的实际画面修改这四个参数。
CONFIRM_ROI = (0.0, 0.3125, 1.0, 1.0)
VERSION = 3                                        # 协议版本，命令帧与结果帧必须一致
# 结果帧正文格式：10 个 uint8 + 4 个 uint16 + 2 个 uint8，共 20 字节，大端。
# 依次为 version,seq,mode,team,type,count,hazard,normal,core,patient,cx,cy,w,h,score,valid
RESULT_FMT = "!BBBBBBBBBBHHHHBB"
COMMAND_HEADER = b"\xA5\x5A"                       # 主控 → MaixCAM 命令帧包头
RESULT_HEADER = b"\xA5\x5B"                        # MaixCAM → 主控 结果帧包头


# ============================================================================
# 2) 类型与模式枚举
# ============================================================================

class TargetType:
    """结果帧中 type 字段的取值，0 表示本帧没有可选目标。"""

    NONE = 0
    NORMAL_MATERIAL = 1      # 普通物资（绿色正方体）
    CORE_MATERIAL = 2        # 核心物资（黑色正三棱锥）
    PATIENT = 3              # 伤员（橘色长方体）
    HAZARD = 4               # 危险物（浅蓝色正方体），永不作为搬运目标
    BLUE_MATERIAL_ZONE = 5   # 蓝方物资区
    BLUE_PATIENT_ZONE = 6    # 蓝方伤员区
    RED_MATERIAL_ZONE = 7    # 红方物资区
    RED_PATIENT_ZONE = 8     # 红方伤员区


class Mode:
    """命令帧中的 mode 字段，决定“当前要找什么”。"""

    OBSERVE = 0
    NORMAL_MATERIAL = 1
    CORE_MATERIAL = 2
    PATIENT = 3
    # 4 号模式已保留但不再接受，因此 VALID_MODES 里没有 4
    MATERIAL_ZONE = 5
    PATIENT_ZONE = 6
    RESCUE_TARGETS = 7  # 普通物资、核心物资、伤员，排除危险物和安全区。
    MIXED_MATERIALS = 8  # 普通物资与核心物资可混合转运。
    CONFIRM_CARGO = 9  # 只统计CONFIRM_ROI内类别，不输出导航目标。


# 标签名 → 协议 type 号：class_id 从 0 开始，type 从 1 开始，因此要 +1。
TYPE_BY_LABEL = {name: index + 1 for index, name in enumerate(LABELS)}
# 只接受协议文档里定义的模式号；4 号保留，不在此列表中。
VALID_MODES = (0, 1, 2, 3, 5, 6, 7, 8, 9)


# ============================================================================
# 3) CRC16 与命令帧解析
# ============================================================================

def crc16(data):
    """CRC-16/CCITT-FALSE：初值 0xFFFF、多项式 0x1021、不做结果异或。

    参数 data 为 bytes/bytearray；返回 0~0xFFFF 的整数。
    发送和接收必须使用同一套参数，否则 CRC 永远对不上。
    """
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            # 最高位为 1 时左移后异或多项式，否则只左移；始终截断到 16 位
            value = ((value << 1) ^ 0x1021) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def with_crc(data):
    """在 data 末尾追加 2 字节大端 CRC16，返回完整帧。"""
    return data + struct.pack("!H", crc16(data))


class CommandParser:
    """保留残帧，处理一次读取多帧及错位数据；只接受有效CRC和参数。"""

    def __init__(self):
        # 滑动缓冲区：串口一次 read 可能包含半帧、多帧或垃圾字节
        self.buffer = bytearray()

    def feed(self, chunk):
        """喂入一段串口数据，返回本次解析出的所有合法命令 [(mode, team), ...]。"""
        self.buffer.extend(chunk)
        commands = []
        while len(self.buffer) >= 2:
            # 步骤1：把窗口头部对齐到命令帧头 A5 5A，不是就丢一个字节重找
            if self.buffer[:2] != COMMAND_HEADER:
                del self.buffer[0]
                continue
            # 步骤2：不足 7 字节说明帧还没收全，保留缓冲区等下一次数据
            if len(self.buffer) < 7:
                break
            # 步骤3：校验前 5 字节的 CRC（末尾 2 字节为大端 CRC）
            frame = bytes(self.buffer[:7])
            if crc16(frame[:5]) != struct.unpack("!H", frame[5:])[0]:
                # CRC 错：说明刚才的 A5 5A 可能是数据里的巧合，丢 1 字节重新同步
                del self.buffer[0]
                continue
            # 步骤4：CRC 正确，消费整帧，再检查版本/模式/队伍是否合法
            del self.buffer[:7]
            version, mode, team = frame[2:5]
            if version == VERSION and mode in VALID_MODES and team in (0, 1, 2):
                commands.append((mode, team))
        return commands


# ============================================================================
# 4) 目标筛选与打分
# ============================================================================

def allowed_types(mode, team):
    """返回当前模式下允许作为导航目标的 TargetType 集合。

    危险物永远不会出现在返回值里；安全区只在对应模式且已配置队伍时才允许。
    """
    # 确认模式只统计夹持区域，不输出导航目标
    if mode == Mode.CONFIRM_CARGO:
        return set()
    # 观察模式：所有目标都看，但排除危险物
    if mode == Mode.OBSERVE:
        return set(TYPE_BY_LABEL.values()) - {TargetType.HAZARD}
    # 单一类别模式：type 号与 mode 号恰好一致，直接复用
    if mode in (Mode.NORMAL_MATERIAL, Mode.CORE_MATERIAL, Mode.PATIENT):
        return {mode}
    # 救援目标：三类可搬运物资，不含安全区和危险物
    if mode == Mode.RESCUE_TARGETS:
        return {TargetType.NORMAL_MATERIAL, TargetType.CORE_MATERIAL, TargetType.PATIENT}
    # 混合物资：只允许普通物资 + 核心物资
    if mode == Mode.MIXED_MATERIALS:
        return {TargetType.NORMAL_MATERIAL, TargetType.CORE_MATERIAL}
    # 以下为安全区模式：必须知道己方队伍颜色，否则无法判断哪个区是“己方”
    if team not in (1, 2):
        return set()  # 未选队伍时不猜测己方安全区。
    if mode == Mode.MATERIAL_ZONE:
        # 蓝队去蓝方物资区，红队去红方物资区
        return {TargetType.BLUE_MATERIAL_ZONE if team == 1 else TargetType.RED_MATERIAL_ZONE}
    if mode == Mode.PATIENT_ZONE:
        # 蓝队去蓝方伤员区，红队去红方伤员区
        return {TargetType.BLUE_PATIENT_ZONE if team == 1 else TargetType.RED_PATIENT_ZONE}
    return set()


def target_position_cost(obj, image_width, image_height):
    """中心到画面中线的归一化偏差 + 中心到画面底部的归一化距离。

    返回值越小越优：
      - 横向越靠近画面中线，x_error 越小；
      - 纵向越靠近画面底部（固定相机下视为越近），(1 - y_nearness) 越小。
    """
    cx = obj.x + obj.w / 2
    cy = obj.y + obj.h / 2
    # 横向误差归一化到 [0, 1]：0=正中，1=贴边
    x_error = min(1.0, abs(cx - image_width / 2) / (image_width / 2))
    # 纵向靠近程度归一化到 [0, 1]：1=画面最下方（最近）
    y_nearness = max(0.0, min(1.0, cy / image_height))
    return TARGET_X_WEIGHT * x_error + TARGET_Y_WEIGHT * (1.0 - y_nearness)


def point_in_roi(cx, cy, image_width, image_height, roi):
    """判断归一化中心点 (cx, cy) 是否落在归一化 ROI 内。

    ROI 参数为 (left, top, right, bottom)，取值范围 0~1；
    比较时把像素中心除以图像宽高，统一转换到归一化坐标。
    """
    left, top, right, bottom = roi
    return (left <= cx / image_width <= right
            and top <= cy / image_height <= bottom)


def select_target(objects, mode, team, image_width, image_height, roi=SEARCH_ROI):
    """在当前模式下从检测结果中选出唯一导航目标。

    筛选条件：类别合法、置信度达标、宽高有效、类型在当前模式允许集合内、
    且目标中心落在 ROI 内。返回 (目标对象或 None, 候选数量)。
    """
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
    # 几何分数相同时选y更大者（更近），再比较置信度；各类别按相同规则竞争。
    candidates.sort(key=lambda o: (
        target_position_cost(o, image_width, image_height),
        -(o.y + o.h / 2), -o.score,
    ))
    # count 表示“当前模式下可见候选数”，上限 255 以适配协议单字节字段
    return candidates[0] if candidates else None, min(len(candidates), 255)


def u16(value):
    """把浮点/整数坐标安全转换成 uint16，越界时截断而不是让 struct.pack 抛异常。"""
    return max(0, min(65535, int(value)))


def count_types_in_roi(objects, image_width, image_height, roi=SEARCH_ROI):
    """统计ROI内三类目标和危险物，不受导航模式筛选影响。

    返回字典 {TargetType: 数量}，每个数量都限制在 0~255。
    危险物单独统计，用于 STM32 侧做安全熔断。
    """
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
        # 只统计有效检测框；类别不在这四类里的（例如安全区）直接跳过
        if (0 <= obj.class_id < len(LABELS) and obj.score >= CONF_THRESHOLD
                and obj.w > 0 and obj.h > 0):
            kind = TYPE_BY_LABEL[LABELS[obj.class_id]]
            if kind not in counts:
                continue
            # 统计依据是目标中心点是否落在 ROI 内，而不是框与 ROI 是否相交
            if point_in_roi(obj.x + obj.w / 2, obj.y + obj.h / 2,
                            image_width, image_height, roi):
                counts[kind] += 1
    # 协议里数量字段是 1 字节，超过 255 时截断
    return {kind: min(count, 255) for kind, count in counts.items()}


# ============================================================================
# 5) 结果帧打包
# ============================================================================

def result_packet(sequence, mode, team, target, count, cargo_counts):
    """把本帧检测结果打包成 24 字节结果帧（含包头与 CRC）。

    正文字段顺序：version,seq,mode,team,type,count,
                  hazard,normal,core,patient,cx,cy,w,h,score,valid
    其中坐标/宽高为图像像素（大端 uint16），score 为 0~100 的百分比。
    """
    # 每帧重建结果，无目标时坐标、类别、数量全部清零，避免沿用上帧。
    kind = x = y = w = h = score = valid = 0
    if target is not None:
        # 目标中心与宽高取整到 uint16；置信度转成 0~100 的整数
        kind = TYPE_BY_LABEL[LABELS[target.class_id]]
        x, y = u16(target.x + target.w / 2), u16(target.y + target.h / 2)
        w, h = u16(target.w), u16(target.h)
        score = max(0, min(100, int(target.score * 100)))
        valid = 1
    else:
        # 没有目标时 count 必须清零，不能把“候选数量”误当成有效目标数
        count = 0
    # 按固定顺序打包四类数量：危险物、普通物资、核心物资、伤员
    body = struct.pack(RESULT_FMT, VERSION, sequence & 255, mode, team,
                       kind, count,
                       *(max(0, min(255, int(cargo_counts[rescue_type])))
                         for rescue_type in (TargetType.HAZARD, TargetType.NORMAL_MATERIAL,
                                             TargetType.CORE_MATERIAL, TargetType.PATIENT)),
                       x, y, w, h, score, valid)
    return with_crc(RESULT_HEADER + body)


# ============================================================================
# 6) 主循环
# ============================================================================

def main():
    # 延迟导入 maix 模块：便于在 PC 上做纯逻辑测试/静态检查时不依赖硬件库
    from maix import camera, display, image, nn, app, uart, err, pinmap, time

    # 启动前检查模型：缺模型或标签不匹配时直接报错退出，避免“带病运行”
    if not os.path.isfile(MODEL_PATH):
        raise RuntimeError("缺少新版模型，请训练八分类模型并设置 MODEL_PATH；不能使用旧七分类模型")
    detector = nn.YOLOv5(model=MODEL_PATH)
    model_labels = getattr(detector, "labels", None)
    if model_labels is None or tuple(model_labels) != LABELS:
        raise RuntimeError("模型标签与 LABELS 不一致，请检查模型导出标签和顺序")

    # 把 A16/A17 复用为 UART0 的 TX/RX，用于和 STM32 通信
    for pin, function in (("A16", "UART0_TX"), ("A17", "UART0_RX")):
        err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping failed")
    serial = uart.UART("/dev/ttyS0", 115200)   # 与 STM32 约定：115200, 8N1
    # 相机分辨率/格式直接跟随模型输入，避免额外缩放造成偏移
    cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
    screen = display.Display()                 # 板载屏幕，仅用于本地调试画面
    parser = CommandParser()                   # 串口命令滑动窗口解析器
    mode, team, sequence = Mode.OBSERVE, 0, 0  # 默认观察模式、未配置队伍、帧序号从 0 开始

    while not app.need_exit():
        # --- 1) 接收 STM32 命令：一次 read 可能包含多条，只保留最后一条有效命令 ---
        chunk = serial.read(256)
        if chunk:
            for mode, team in parser.feed(chunk):
                pass  # 使用最后一条有效命令；不在视觉端修改任务完成状态。

        # --- 2) 取图并推理（必须在画图之前，避免文字/框污染模型输入） ---
        frame = cam.read()
        # 在绘图前推理，避免文字和检测框污染模型输入。
        objects = detector.detect(frame, conf_th=CONF_THRESHOLD, iou_th=IOU_THRESHOLD)

        # --- 3) 按模式选择 ROI：确认模式只看夹具附近，其余模式看整个搜索区 ---
        active_roi = CONFIRM_ROI if mode == Mode.CONFIRM_CARGO else SEARCH_ROI
        target, count = select_target(objects, mode, team, frame.width(), frame.height(), active_roi)
        # 无论什么模式都统计三类物资和危险物，供 STM32 做安全/完成判断
        cargo_counts = count_types_in_roi(objects, frame.width(), frame.height(), active_roi)

        # --- 4) 回传结果帧，并递增 8 位帧序号（STM32 用它去重） ---
        serial.write(result_packet(sequence, mode, team, target, count, cargo_counts))
        sequence = (sequence + 1) & 255

        # --- 5) 本地调试绘图：命中目标画绿框，其余画红框，并标出 ROI ---
        for obj in objects:
            if not 0 <= obj.class_id < len(LABELS):
                continue
            color = image.COLOR_GREEN if obj is target else image.COLOR_RED
            frame.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color)
            frame.draw_string(obj.x, obj.y, "%s %.2f" % (LABELS[obj.class_id], obj.score), color=color)
        # 确认模式的 ROI 用绿色标出，其他模式用白色，方便现场调整 CONFIRM_ROI
        roi_color = image.COLOR_GREEN if mode == Mode.CONFIRM_CARGO else image.COLOR_WHITE
        roi_x = int(active_roi[0] * frame.width())
        roi_y = int(active_roi[1] * frame.height())
        roi_w = int((active_roi[2] - active_roi[0]) * frame.width())
        roi_h = int((active_roi[3] - active_roi[1]) * frame.height())
        frame.draw_rect(roi_x, roi_y, roi_w, roi_h, color=roi_color, thickness=2)
        # 左上角状态栏：模式、队伍、候选数、三类物资数、危险物数
        frame.draw_string(0, 0, "Mode:%d T:%d N:%d C:%d P:%d H:%d" % (
            mode, team, count, cargo_counts[TargetType.NORMAL_MATERIAL],
            cargo_counts[TargetType.CORE_MATERIAL], cargo_counts[TargetType.PATIENT],
            cargo_counts[TargetType.HAZARD]), color=image.COLOR_WHITE)
        screen.show(frame)
        # 主动让出 5ms，避免单核板子上纯 Python 循环把 CPU 占满
        time.sleep_ms(5)


if __name__ == "__main__":
    main()
