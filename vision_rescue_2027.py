"""MaixCAM：2027 智能救援视觉识别（独立入口，旧 main.py 保留）。

规则依据《智能+.pdf》第16—19页：
普通物资=绿色40mm正方体；核心物资=黑色40mm正三棱锥；
伤员=橘色80x40x40mm长方体；危险物=浅蓝色40mm正方体。
前三类均为双方公共目标，红蓝只用于区分安全区。决赛类别需现场调整。

本协议与旧 55 AA 协议隔离，帧结构与 STM32 maixcam.c 保持一致：
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
count在模式9表示确认范围内普通物资+核心物资+伤员的数量合计，不含危险物；
其他模式表示筛选后的导航候选数量。它不是累计送达数或允许一次搬运数量。
危险物永不参与目标选择及count；普通物资、核心物资、伤员也另行分类统计。
搜索模式(mode 0-8)的目标总数、三类数量及hazard_count均只统计SEARCH_ROI内目标中心。
确认模式(mode 9)只统计CONFIRM_ROI；三类物品要求检测框完整在区域内，
危险物只要与区域有交叠就计数；该模式不输出导航目标。
CONFIRM_ROI使用归一化(左,上,右,下)参数；第三姿态低头后整幅画面对应夹具范围；
STM32需检查帧新鲜度及hazard_count，不能因valid=0忽略危险物数据。
遮挡、漏检仍可能发生，单帧类别计数为0不等于已确认抓取失败或成功；
建议连续多帧稳定后由STM32确认物品类别，三类总数及危险物数量保持独立。
valid只表示本帧存在检测目标，不表示转运合法或已经送达。
首次只运一个普通物资、伤员单独一个、物资最多三个等由STM32任务管理。
建议阶段：首次用mode=1；首次普通物资送达确认后用mode=7搜寻下一目标；
选定物资后可用mode=8补充物资，选定伤员后用mode=3单独转运。
物资按类别加权后的检测框面积选目标；模式9不选导航目标，独立回传三类数量合计。
安全区/分界线的本地预览不复用旧半区类型号；配套STM32修改前不回传这些导航框。
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

MODEL_PATH = "/root/models/rescue_2027/model_332145.mud"
# 必须与模型内置标签顺序严格一致。语义由训练数据决定，颜色仅为初赛提示。
# class_id 是模型标签下标；协议 type 使用下方 TYPE_BY_LABEL 的固定映射。
LABELS = (
    "HAZARD", "Blue_Zone", "Red_Zone", "Red_Zone_Split",
    "Blue_Zone_Split", "CORE_MATERIAL", "PATIENT", "NORMAL_MATERIAL",
)
CONF_THRESHOLD = 0.50   # 检测、计数和预览共用门槛，包含等于 0.50 的框
IOU_THRESHOLD = 0.45    # NMS 的 IoU 阈值：越大保留的重复框越多，越小删得越狠
# 加权面积相同时，用位置代价打破平局；这里不是实际测距。
TARGET_X_WEIGHT = 0.65  # 横向居中项权重（偏离画面中线越远，代价越大）
TARGET_Y_WEIGHT = 0.35  # 纵向靠近项权重（中心越靠画面下方，代价越小）
# 搜索阶段的目标选择和所有数量统计均使用此区域；参数为归一化(左,上,右,下)。
SEARCH_ROI = (0.0, 0.0, 1.0, 1.0)
# 相机姿态按流程编号，由STM32舵机执行：1=出发最低头，2=抬头找物并接近，
# 3=低头检查夹子，4=更抬头寻找安全区（比第二姿态更抬头）。
# 第一姿态用于满足出发整车高度不超过20cm的要求；
# 由STM32在出发阶段保持，进入找物阶段后再抬到第二姿态，不能因相机程序启动就抬头。
# 第一姿态与第三姿态用途不同，不默认共用同一个舵机位置；具体位置由队友标定。
# 姿态编号不是Mode模式号，也不是已约定的串口字段。
# 第三姿态的整幅画面就是夹子检查范围，因此运输确认不再额外裁掉画面上部。
# 必须由业务流程等低头动作稳定并取得新画面后再确认，mode=9本身不代表舵机已到位。
CONFIRM_ROI = (0.0, 0.0, 1.0, 1.0)
SPLIT_CONFIRM_FRAMES = 10  # 切换前连续新画面均需通过区域包含检查
SPLIT_LOST_MS = 500       # 仅跟随状态的本地计时；保持底盘速度需 STM32 配套
LOG_INTERVAL_MS = 500     # MaixVision终端：状态变化立即输出，普通数据每500ms输出一次。
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


class NavigationAction:
    """回基地识别的五种业务状态；字符串供本地使用，不是串口类型号。"""

    FOLLOW_ZONE = "FOLLOW_ZONE"                  # 跟随本帧可见的己方整体安全区。
    FOLLOW_SPLIT = "FOLLOW_SPLIT"                # 跟随本帧可见的己方分界线。
    HOLD_LAST_SPEED = "HOLD_LAST_SPEED"          # 已跟线后短时漏检，请求按剩余时间保持速度。
    SEARCH_ZONE = "SEARCH_ZONE"                  # 没有可用参照，重新寻找己方整体安全区。
    NOT_RETURNING_TO_BASE = "NOT_RETURNING_TO_BASE"  # 非回基地阶段，不干涉其他任务。


# 模型 class_id 随标签顺序变化；物资协议类型号独立于标签顺序。
TYPE_BY_LABEL = {
    "HAZARD": TargetType.HAZARD,
    "CORE_MATERIAL": TargetType.CORE_MATERIAL,
    "PATIENT": TargetType.PATIENT,
    "NORMAL_MATERIAL": TargetType.NORMAL_MATERIAL,
}
# 选物优先级为检测框面积乘权重；浅蓝危险物在 allowed_types 中排除，不参与排序。
MATERIAL_AREA_WEIGHTS = {
    TargetType.NORMAL_MATERIAL: 1.0,
    TargetType.CORE_MATERIAL: 3.0,
    TargetType.PATIENT: 1.8,
}
# 场地区域使用模型标签判断，不借用旧协议中的“物资区/伤员区”编号。
ZONE_LABELS = ("Blue_Zone", "Red_Zone")
TEAM_ZONE_LABELS = {
    1: ("Blue_Zone", "Blue_Zone_Split"),
    2: ("Red_Zone", "Red_Zone_Split"),
}
# 只接受协议文档里定义的模式号；4 号保留，不在此列表中。
VALID_MODES = (0, 1, 2, 3, 5, 6, 7, 8, 9)
# 固定调试配置：平时直接修改下面两个数字，保存后通过 MaixVision 在相机运行。
# 配置不变时可以一直沿用，无需每次启动重新填写；暂时不用触屏选队。
# 模式：0=观察全部可搬运物，1=普通物资，2=核心物资，3=伤员，
#       5=回己方物资区，6=回己方伤员区，7=三类救援目标，8=普通+核心，9=运输确认。
# 4不可用；例如只调蓝方回物资区，填写 INITIAL_MODE=5、INITIAL_TEAM=1。
# 注意：这两项是启动初值；收到 STM32 的有效模式/队伍命令后，仍会采用命令值。
# 本轮先调蓝方、1号位；出发位对应的起步动作由主控执行，不占用Mode模式号。
INITIAL_MODE = 9  # 默认启动为夹内物品数量确认；收到有效STM32命令后按主控模式运行。
INITIAL_TEAM = 1  # 当前默认蓝方；后续手动改红方时填2。


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


def valid_detection(obj):
    """预览、筛选和计数使用同一门槛；不限制相邻帧位置变化。"""
    return (0 <= obj.class_id < len(LABELS) and obj.score >= CONF_THRESHOLD
            and obj.w > 0 and obj.h > 0)


def center_in_box(obj, box):
    """只检查检测框中心；边界算作框内，不要求整个 obj 框位于 box 内。"""
    cx, cy = obj.x + obj.w / 2, obj.y + obj.h / 2
    return box.x <= cx <= box.x + box.w and box.y <= cy <= box.y + box.h


class ZoneFollower:
    """本地安全区/分界线选择，用于预览和后续对接，不输出旧半区协议类型。

    每张新推理画面只调用一次 update。重新进入找区模式或更换队伍需 reset。
    HOLD 只表示 500ms 内短暂漏检，不返回旧检测框冒充本帧结果，也不控制电机。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.confirm_frames = 0
        self.following_split = False
        self.last_split_ms = None
        self.state = "SEARCH"

    def update(self, objects, team, now_ms):
        if team not in TEAM_ZONE_LABELS:
            raise ValueError("回基地前必须配置队伍：1=蓝方，2=红方")
        zone_label, split_label = TEAM_ZONE_LABELS[team]
        zones = [o for o in objects if valid_detection(o) and LABELS[o.class_id] == zone_label]
        splits = [o for o in objects if valid_detection(o) and LABELS[o.class_id] == split_label]
        # 同类存在多个框时优先置信度；暂不加入跳变过滤或跟踪匹配。
        zone = max(zones, key=lambda o: o.score, default=None)
        if self.following_split:
            # 已通过确认，不再依赖本帧整体安全区是否可见。
            split = max(splits, key=lambda o: o.score, default=None)
            if split is not None:
                self.last_split_ms = now_ms
                self.state = "SPLIT"
                return split
            if now_ms - self.last_split_ms < SPLIT_LOST_MS:
                self.state = "HOLD"
                return None
            self.reset()  # 超时后从头进行包含检查和连续 10 帧确认
        contained = [o for o in splits if any(center_in_box(o, z) for z in zones)]
        split = max(contained, key=lambda o: o.score, default=None)
        self.confirm_frames = self.confirm_frames + 1 if split is not None else 0
        if self.confirm_frames >= SPLIT_CONFIRM_FRAMES:
            self.following_split = True
            self.last_split_ms = now_ms
            self.state = "SPLIT"
            return split
        self.state = "ZONE" if zone is not None else "SEARCH"
        return zone


def navigation_observation(follower, target, mode, team, now_ms):
    """导航业务结果，供界面及后续报文适配使用；不是新的串口帧定义。

    purpose 保留物资/伤员的运输目的，action 只取 NavigationAction 的五种状态。
    FOLLOW_ZONE/FOLLOW_SPLIT 携带当前参照框；HOLD_LAST_SPEED 只提供剩余
    宽限时间，不拿旧框冒充新检测，也不代表底盘已经保持速度。
    SEARCH_ZONE 没有可用参照；NOT_RETURNING_TO_BASE 清空找区历史，
    只表示当前不执行找区逻辑，不要求底盘停车。队伍是进入回基地前的配置条件。
    """
    purpose = {Mode.MATERIAL_ZONE: "MATERIAL", Mode.PATIENT_ZONE: "PATIENT"}.get(mode)
    if purpose is None:
        # 即使单独调用此函数，退出回基地时也不保留上一轮跟线和计时状态。
        follower.reset()
        action = NavigationAction.NOT_RETURNING_TO_BASE
    else:
        if team not in TEAM_ZONE_LABELS:
            raise ValueError("回基地前必须配置队伍：1=蓝方，2=红方")
        action = {"ZONE": NavigationAction.FOLLOW_ZONE, "SPLIT": NavigationAction.FOLLOW_SPLIT,
                  "HOLD": NavigationAction.HOLD_LAST_SPEED, "SEARCH": NavigationAction.SEARCH_ZONE}[follower.state]
    visible = target is not None and action in (NavigationAction.FOLLOW_ZONE, NavigationAction.FOLLOW_SPLIT)
    hold_remaining = 0
    if action == NavigationAction.HOLD_LAST_SPEED:
        hold_remaining = max(0, SPLIT_LOST_MS - (now_ms - follower.last_split_ms))
    return {
        "mode": mode,
        "purpose": purpose,
        "team": team,
        "action": action,
        "reference_label": LABELS[target.class_id] if visible else None,
        "reference_visible": visible,
        "bbox": (target.x + target.w / 2, target.y + target.h / 2, target.w, target.h)
                if visible else None,
        "confidence": target.score if visible else 0.0,
        "confirm_frames": follower.confirm_frames if purpose is not None else 0,
        "hold_remaining_ms": hold_remaining,
    }


def select_target(objects, mode, team, image_width, image_height, roi=SEARCH_ROI):
    """在当前模式下从检测结果中选出唯一导航目标。

    筛选条件：类别合法、置信度达标、宽高有效、类型在当前模式允许集合内、
    且目标中心落在 ROI 内并位于已检测到的红蓝安全区以外。
    物资按 w*h*类别倍率从大到小选择，返回 (目标对象或 None, 候选数量)。
    """
    if image_width <= 0 or image_height <= 0:
        raise ValueError("图像宽高必须大于0")
    left, top, right, bottom = roi
    if not (0 <= left < right <= 1 and 0 <= top < bottom <= 1):
        raise ValueError("目标选择ROI无效")
    allowed = allowed_types(mode, team)
    zones = [o for o in objects if valid_detection(o) and LABELS[o.class_id] in ZONE_LABELS]
    candidates = [o for o in objects if valid_detection(o)
                  and TYPE_BY_LABEL.get(LABELS[o.class_id]) in allowed
                  and point_in_roi(o.x + o.w / 2, o.y + o.h / 2,
                                   image_width, image_height, roi)
                  and not any(center_in_box(o, zone) for zone in zones)]
    # 排除基地内物块仅影响导航候选，不改变独立类别计数和危险物观察。
    candidates.sort(key=lambda o: (
        -(o.w * o.h * MATERIAL_AREA_WEIGHTS[TYPE_BY_LABEL[LABELS[o.class_id]]]),
        target_position_cost(o, image_width, image_height),
        -(o.y + o.h / 2), -o.score,
    ))
    # count 表示“当前模式下可见候选数”，上限 255 以适配协议单字节字段
    return candidates[0] if candidates else None, min(len(candidates), 255)


def u16(value):
    """把浮点/整数坐标安全转换成 uint16，越界时截断而不是让 struct.pack 抛异常。"""
    return max(0, min(65535, int(value)))


def count_types_in_roi(objects, image_width, image_height, roi=SEARCH_ROI, require_full_box=False):
    """统计ROI内三类目标和危险物，不受导航模式筛选影响。

    返回字典 {TargetType: 数量}，每个数量都限制在 0~255。
    搜索阶段按中心点统计。确认阶段 require_full_box=True：三类物品要求
    四角完整进入区域，危险物则按矩形交叠计数，不能因其未完全进入而漏掉。
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
        if valid_detection(obj):
            kind = TYPE_BY_LABEL.get(LABELS[obj.class_id])
            if kind not in counts:
                continue
            if require_full_box:
                left_px, right_px = left * image_width, right * image_width
                top_px, bottom_px = top * image_height, bottom * image_height
                if kind == TargetType.HAZARD:
                    inside = (obj.x < right_px and obj.x + obj.w > left_px
                              and obj.y < bottom_px and obj.y + obj.h > top_px)
                else:
                    inside = (left_px <= obj.x and obj.x + obj.w <= right_px
                              and top_px <= obj.y and obj.y + obj.h <= bottom_px)
            else:
                inside = point_in_roi(obj.x + obj.w / 2, obj.y + obj.h / 2,
                                      image_width, image_height, roi)
            if inside:
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
    模式9的count为确认范围内普通+核心+伤员数量合计；无导航目标时valid和框仍为0。
    """
    # 无导航目标时清零目标字段，独立类别数量仍照常发送。
    kind = x = y = w = h = score = valid = 0
    if target is not None:
        # 目标中心与宽高取整到 uint16；置信度转成 0~100 的整数
        kind = TYPE_BY_LABEL[LABELS[target.class_id]]
        x, y = u16(target.x + target.w / 2), u16(target.y + target.h / 2)
        w, h = u16(target.w), u16(target.h)
        score = max(0, min(100, int(target.score * 100)))
        valid = 1
    else:
        # 无导航目标时清零候选数；模式9的确认总数在下方单独填写。
        count = 0
    if mode == Mode.CONFIRM_CARGO:
        # 适配STM32的target_num：确认总数不依赖导航目标，危险物仍单独计数。
        count = min(255, sum(cargo_counts[k] for k in (
            TargetType.NORMAL_MATERIAL, TargetType.CORE_MATERIAL, TargetType.PATIENT)))
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

MODE_NAMES_CN = {
    Mode.OBSERVE: "观察全部可搬运物", Mode.NORMAL_MATERIAL: "寻找绿色普通物资",
    Mode.CORE_MATERIAL: "寻找黑色核心物资", Mode.PATIENT: "寻找橙色伤员",
    Mode.MATERIAL_ZONE: "回己方物资区", Mode.PATIENT_ZONE: "回己方伤员区",
    Mode.RESCUE_TARGETS: "寻找三类救援目标", Mode.MIXED_MATERIALS: "寻找普通和核心物资",
    Mode.CONFIRM_CARGO: "检查运输范围内的物品数量",
}
TEAM_NAMES_CN = {1: "蓝方", 2: "红方"}
LABEL_NAMES_CN = {
    "HAZARD": "浅蓝危险物", "NORMAL_MATERIAL": "绿色普通物资",
    "CORE_MATERIAL": "黑色核心物资", "PATIENT": "橙色伤员",
    "Blue_Zone": "蓝方整体安全区", "Red_Zone": "红方整体安全区",
    "Blue_Zone_Split": "蓝方分界线", "Red_Zone_Split": "红方分界线",
}
NAVIGATION_NAMES_CN = {
    NavigationAction.FOLLOW_ZONE: "跟随整体安全区中心",
    NavigationAction.FOLLOW_SPLIT: "跟随分界线中心",
    NavigationAction.HOLD_LAST_SPEED: "分界线暂时漏检，请求保持上一速度",
    NavigationAction.SEARCH_ZONE: "未找到可用参照，请求旋转寻找安全区",
    NavigationAction.NOT_RETURNING_TO_BASE: "非回基地阶段",
}


def format_debug_status(mode, team, source, navigation, target, count, cargo_counts, navigation_connected=False):
    """终端中文摘要；只描述视觉结果，不把动作请求写成车辆已执行。"""
    returning = navigation["purpose"] is not None
    state = NAVIGATION_NAMES_CN[navigation["action"]] if returning else MODE_NAMES_CN[mode]
    detail = "无可用目标"
    if target is not None:
        detail = "%s，中心=(%.1f,%.1f)，宽高=(%.1f,%.1f)，置信度=%.2f" % (
            LABEL_NAMES_CN[LABELS[target.class_id]], target.x + target.w / 2,
            target.y + target.h / 2, target.w, target.h, target.score)
    text = "[视觉状态] 队伍=%s；模式=%d（%s）；配置来源=%s\n当前=%s；目标=%s" % (
        TEAM_NAMES_CN[team], mode, MODE_NAMES_CN[mode], source, state, detail)
    if returning:
        text += "\n分界线确认=%d/%d帧；漏检保持剩余=%d毫秒" % (
            navigation["confirm_frames"], SPLIT_CONFIRM_FRAMES, navigation["hold_remaining_ms"])
        text += "；导航数据已交发送适配，接收执行待核对" if navigation_connected else "；导航发送适配未接入，仅本地判断"
    else:
        text += "；可选目标=%d" % count
    return text + "\n范围内数量：普通=%d，核心=%d，伤员=%d，危险=%d" % (
        cargo_counts[TargetType.NORMAL_MATERIAL], cargo_counts[TargetType.CORE_MATERIAL],
        cargo_counts[TargetType.PATIENT], cargo_counts[TargetType.HAZARD])


def main(navigation_sender=None):
    """navigation_sender由双方确认协议后接入，签名为sender(serial, observation)。

    observation包含mode/team/purpose/action/reference_label/bbox/confidence/
    confirm_frames/hold_remaining_ms/sequence；坐标为中心xywh像素，分数为0—1。
    此回调每张推理新画面调用一次，与原24字节结果帧共享sequence。
    未提供回调时不发送臆定扩展帧，终端明确显示未接入；原7/24字节协议照常工作。
    """
    # 配置在启动前完成，不设置 WAIT_TEAM 等待状态，也不擅自默认红蓝。
    if INITIAL_MODE not in VALID_MODES or INITIAL_TEAM not in TEAM_ZONE_LABELS:
        raise ValueError("启动前请设置 INITIAL_MODE，并将 INITIAL_TEAM 填为1（蓝）或2（红）")
    print("[启动配置] 队伍=%s；模式=%d（%s）；来源=代码固定配置" % (
        TEAM_NAMES_CN[INITIAL_TEAM], INITIAL_MODE, MODE_NAMES_CN[INITIAL_MODE]), flush=True)
    # 延迟导入 maix 模块：便于在 PC 上做纯逻辑测试/静态检查时不依赖硬件库
    from maix import camera, display, image, nn, app, uart, err, pinmap, time

    # 启动前检查模型：缺模型或标签不匹配时直接报错退出，避免“带病运行”
    if not os.path.isfile(MODEL_PATH):
        raise RuntimeError("找不到模型，请将模型包上传到相机并核对 MODEL_PATH: %s" % MODEL_PATH)
    # 明确让结果对应当前输入画面，避免双缓冲把转姿态前的结果带入新阶段。
    detector = nn.YOLOv5(model=MODEL_PATH, dual_buff=False)
    model_labels = getattr(detector, "labels", None)
    if model_labels is None or tuple(model_labels) != LABELS:
        raise RuntimeError("模型标签不一致；需要 %r，实际 %r" % (LABELS, model_labels))

    # 把 A16/A17 复用为 UART0 的 TX/RX，用于和 STM32 通信
    for pin, function in (("A16", "UART0_TX"), ("A17", "UART0_RX")):
        err.check_raise(pinmap.set_pin_function(pin, function), "UART pin mapping failed")
    serial = uart.UART("/dev/ttyS0", 115200)   # 与 STM32 约定：115200, 8N1
    # 相机分辨率/格式直接跟随模型输入，避免额外缩放造成偏移
    cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
    screen = display.Display()                 # 板载屏幕，仅用于本地调试画面
    parser = CommandParser()                   # 串口命令滑动窗口解析器
    mode, team, sequence = INITIAL_MODE, INITIAL_TEAM, 0
    zone_follower = ZoneFollower()
    zone_context = None
    print("[导航接口] " + ("已绑定发送适配，接收及执行仍需核对" if navigation_sender is not None else "尚未接入；原24字节物资结果正常发送，整区/分界线仅本地判断"), flush=True)
    config_source = "代码固定配置"
    last_log_key = None
    last_log_ms = 0
    # 仅供MaixVision诊断：TX是写入尝试次数，write=24不代表STM32已收到。
    tx_attempts = tx_errors = rx_bytes = accepted_commands = 0
    last_rx = b""

    while not app.need_exit():
        # --- 1) 接收 STM32 命令：一次 read 可能包含多条，只保留最后一条有效命令 ---
        chunk = serial.read(256)
        if chunk:
            rx_bytes += len(chunk)
            last_rx = bytes(chunk[-32:])
            for command_mode, command_team in parser.feed(chunk):
                # 帧格式仍兼容team=0，但不让未配置命令覆盖启动前选好的队伍。
                # 模式和队伍一起采用，避免只改模式却误用另一队伍。
                if command_team in TEAM_ZONE_LABELS:
                    accepted_commands += 1
                    if config_source != "STM32主控命令" or (mode, team) != (command_mode, command_team):
                        print("[主控命令] %s，模式%d（%s） → %s，模式%d（%s）" % (
                            TEAM_NAMES_CN[team], mode, MODE_NAMES_CN[mode],
                            TEAM_NAMES_CN[command_team], command_mode, MODE_NAMES_CN[command_mode]), flush=True)
                    mode, team = command_mode, command_team
                    config_source = "STM32主控命令"

        # --- 2) 取图并推理（必须在画图之前，避免文字/框污染模型输入） ---
        frame = cam.read()
        # 在绘图前推理，避免文字和检测框污染模型输入。
        objects = detector.detect(frame, conf_th=CONF_THRESHOLD, iou_th=IOU_THRESHOLD)

        # 模式 5/6 仍是主控的旧回基地请求。新视觉选择先用于本地预览，
        # 新区域/分界线 type 未与 STM32 同步前，不向它发送有歧义的导航框。
        context = (mode, team) if mode in (Mode.MATERIAL_ZONE, Mode.PATIENT_ZONE) else None
        if context != zone_context:
            zone_follower.reset()
            zone_context = context
        zone_target = None
        observed_ms = time.ticks_ms()
        if context is not None:
            zone_target = zone_follower.update(objects, team, observed_ms)
        navigation = navigation_observation(zone_follower, zone_target, mode, team, observed_ms)

        # --- 3) 按模式选择 ROI：确认模式只看夹具附近，其余模式看整个搜索区 ---
        active_roi = CONFIRM_ROI if mode == Mode.CONFIRM_CARGO else SEARCH_ROI
        target, count = select_target(objects, mode, team, frame.width(), frame.height(), active_roi)
        # 无论什么模式都统计三类物资和危险物，供 STM32 做安全/完成判断
        cargo_counts = count_types_in_roi(objects, frame.width(), frame.height(), active_roi,
                                         require_full_box=mode == Mode.CONFIRM_CARGO)

        # --- 4) 回传结果帧，并递增 8 位帧序号（STM32 用它去重） ---
        packet = result_packet(sequence, mode, team, target, count, cargo_counts)
        written = serial.write(packet)
        tx_attempts += 1
        if written != len(packet):
            tx_errors += 1
        navigation["sequence"] = sequence
        if navigation_sender is not None:
            navigation_sender(serial, navigation)
        sequence = (sequence + 1) & 255

        if mode == Mode.CONFIRM_CARGO:
            # 模式9逐帧短日志：读取本次实际发送报文，不能把相机帧数当成主控确认次数。
            print("[CONFIRM] t=%d seq=%d N=%d C=%d P=%d H=%d num=%d write=%d/24" % (
                observed_ms, packet[3], packet[9], packet[10], packet[11], packet[8], packet[7], written), flush=True)

        # 日志使用标准输出供MaixVision查看，不向STM32串口写入中文文本。
        # 模式、队伍、来源或导航状态变化立即打印；坐标/计数按时间间隔刷新，避免逐帧刷屏。
        log_key = (mode, team, config_source, navigation["action"])
        if log_key != last_log_key or observed_ms - last_log_ms >= LOG_INTERVAL_MS:
            selected = zone_target if context is not None else target
            # 长中文摘要分行独立输出，降低MaixVision在传输分块处拆断UTF-8字符的风险。
            summary = format_debug_status(mode, team, config_source, navigation,
                                          selected, count, cargo_counts, navigation_sender is not None)
            for line in summary.splitlines():
                print(line, flush=True)
            # 实际送入UART的报文；与视觉摘要共用节流，不逐帧刷屏。
            print("[UART] TX=%d err=%d write=%d/%d RX=%d CMD=%d seq=%d num=%d" % (
                tx_attempts, tx_errors, written, len(packet), rx_bytes, accepted_commands, packet[3], packet[7]), flush=True)
            print("[UART-TX] " + packet.hex(), flush=True)
            if last_rx:
                print("[UART-RX last32] " + last_rx.hex(), flush=True)
            last_log_key, last_log_ms = log_key, observed_ms

        # --- 5) 本地调试绘图：命中目标画绿框，其余画红框，并标出 ROI ---
        for obj in objects:
            if not valid_detection(obj):
                continue
            color = image.COLOR_GREEN if obj is target or obj is zone_target else image.COLOR_RED
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
        frame.draw_string(0, 0, "M:%d Team:%d Candidates:%d" % (mode, team, count), color=image.COLOR_WHITE)
        frame.draw_string(0, 16, "N:%d C:%d P:%d H:%d" % (
            cargo_counts[TargetType.NORMAL_MATERIAL], cargo_counts[TargetType.CORE_MATERIAL],
            cargo_counts[TargetType.PATIENT], cargo_counts[TargetType.HAZARD]), color=image.COLOR_WHITE)
        selected = zone_target if context is not None else target
        name = LABELS[selected.class_id] if selected is not None else "NONE"
        frame.draw_string(0, 32, name, color=image.COLOR_GREEN)
        if context is not None:
            frame.draw_string(0, 48, "Local:%s %d/%d" % (
                zone_follower.state, zone_follower.confirm_frames, SPLIT_CONFIRM_FRAMES), color=image.COLOR_WHITE)
            frame.draw_string(0, 64, "Nav adapter active" if navigation_sender is not None else "Nav adapter pending", color=image.COLOR_WHITE)
            frame.draw_string(0, 80, "%s %dms" % (
                navigation["action"], navigation["hold_remaining_ms"]), color=image.COLOR_WHITE)
        screen.show(frame)
        # 主动让出 5ms，避免单核板子上纯 Python 循环把 CPU 占满
        time.sleep_ms(5)


if __name__ == "__main__":
    main()
