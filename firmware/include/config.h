// ============================================================
//  config.h — 三舵机糖豆分色器 v3 · 硬件配置与调参中心
// ------------------------------------------------------------
//  机构: 泡棉板 L 形骨架
//        糖豆盒 → 单列导管 → 挡板A(上) → [单颗测色仓] → 挡板B(下)
//        (A 与 B 之间就是检测仓, B 同时充当仓底与放料门)
//        → 摇臂上的斜导流槽 → 5 个方形收纳盒 + 1 个废料盒
//
//  传感器: AS7341 11 通道光谱传感器 (I2C 0x39, 带板载白光 LED)
//
//  ★ 所有引脚、角度、时序、阈值都在这里改，main.cpp 无需改动。
// ============================================================
#pragma once

#include <Arduino.h>
#include <Adafruit_AS7341.h>   // 为了 AS7341_GAIN_* / AS7341_CHANNEL_* 枚举

#define FW_VERSION "3.0"

// ------------------------------------------------------------
// 1. 引脚分配 (ESP32 DevKitC / NodeMCU-32S)
//    注意: GPIO34-39 仅可输入; GPIO6-11 接 Flash 不可用;
//          GPIO0/2/12/15 为 strapping 引脚, 上电电平影响启动
// ------------------------------------------------------------
#define PIN_I2C_SDA         21  // I2C 数据 (AS7341 + OLED 共用)
#define PIN_I2C_SCL         22  // I2C 时钟

#define PIN_SERVO_GATE_A    13  // 挡板A (上闸门) SG90
#define PIN_SERVO_GATE_B    14  // 挡板B (下闸门 = 检测仓底板兼放料门) SG90
#define PIN_SERVO_SORTER     4  // 底部摇臂 (斜导流槽) SG90

#define PIN_IR_SENSOR       34  // TCRT5000 到位检测 (仅输入脚, 无内部上拉! 建议 10k 上拉到 3V3)
#define PIN_LIGHT_LED       33  // (可选) 外接白光 LED —— 默认不用, AS7341 模块自带 LED
#define PIN_VIB_MOTOR       25  // (可选) 糖豆盒防架桥振动马达

#define PIN_BUZZER          26  // 蜂鸣器
#define PIN_BUTTON          27  // 按键 (短按 暂停/继续, 长按3s 计数清零)
#define PIN_LED_STRIP       32  // WS2812 状态灯
#define PIN_LED_ONBOARD      2  // 板载 LED

// AS7341 模块自带的白光 LED 由芯片的 LDR 引脚驱动, 走 I2C 控制。
// 只有当你用的是"LED 直接接 VCC 常亮"或"没有板载 LED"的模块时,
// 才把下面这行改成 1, 并按 03 文档 §5 焊外接 LED 驱动。
#define USE_EXTERNAL_LED     0

// ------------------------------------------------------------
// 2. I2C 设备地址
// ------------------------------------------------------------
#define ADDR_AS7341       0x39  // AS7341 光谱传感器
#define ADDR_SSD1306      0x3C  // OLED (少数模块为 0x3D)

// ------------------------------------------------------------
// 3. 糖豆与颜色定义
// ------------------------------------------------------------
// 球形糖豆(Skittles 那类) 直径≈厚度≈13mm;
// 扁圆形糖豆(M&M 那类) 直径 13mm、厚度 5mm —— 此时把 CANDY_THICKNESS_MM 改成 5。
// 这两个数直接决定测色仓的三个内尺寸, 请用游标卡尺实测。
#define CANDY_DIAMETER_MM      13
#define CANDY_THICKNESS_MM     13

// ---- 单颗测色仓 (图注: A 和 B 之间就是检测仓) ----
// 仓腔只需要比糖豆大一点点: 越小越不容易漏进杂光, 判色越稳
#define CHAMBER_W_MM   (CANDY_DIAMETER_MM  + 2)   // 内宽 = 糖豆直径 + 2
#define CHAMBER_D_MM   (CANDY_THICKNESS_MM + 2)   // 内深 = 糖豆厚度 + 2
#define CHAMBER_H_MM   (CANDY_DIAMETER_MM  + 2)   // A—B 净高 = 糖豆直径 + 2

#define COLOR_COUNT            5   // 红 橙 黄 绿 蓝

// ---- 盒位角度 (舵机角度, 0-180) ----
// 盒位弧线整体比示意图外移了 5° (30..170 而不是 25..165):
// 因为斜导流槽的落点在 r=97 附近, 0° 废料槽必须占到 r≈75-135,
// 而 60mm 方盒摆在 25° 时, 它的近角会伸到 y≈6.6mm 的位置,
// 正好压住废料槽 —— 移到 30° 后近角退到 y≈14mm, 留出 4mm 间隙。
//
//        170° 橙        30° 红
//          \             /
//   135° 蓝  \  100° 绿  /  65° 黄
//             \    |    /
//              \   |   /
//            [摇臂轴 = 落料轴]
//                   |
//                 0° 待机位 / 废料槽
//
// 颜色索引顺序 = 色相顺序(红橙黄绿蓝), 与 kBinAngle 的顺序解耦,
// 因此这张表不单调 —— 改盒位摆放只需改这张表, 逻辑完全不受影响。
static const int kBinAngle[COLOR_COUNT] = {
  /* RED    */  30,
  /* ORANGE */ 170,
  /* YELLOW */  65,
  /* GREEN  */ 100,
  /* BLUE   */ 135
};

// 待机位 / 未知颜色(废料)落点角度
#define SORTER_HOME_ANGLE   0

// 舵机安装微调: 实际角度 = kBinAngle + SORTER_TRIM_DEG, 再夹到 [0,180]
// 若装配后所有落点整体偏了同一个角度, 改这里; 若只有某一个盒偏, 改 kBinAngle
#define SORTER_TRIM_DEG     0

// 舵机脉冲宽度 (µs)。SG90 常用 500-2400; 若舵机在端点"吱吱"叫,
// 说明撞限位了, 把范围收窄到 600-2300 或缩小 kBinAngle 的极值
#define SERVO_MIN_US        500
#define SERVO_MAX_US       2400

// 颜色名字 (OLED / 串口显示), 顺序须与上面 kBinAngle 的索引一致
static const char* const kColorName[COLOR_COUNT] = {
  "RED", "ORANGE", "YELLOW", "GREEN", "BLUE"
};

// ------------------------------------------------------------
// 4. 判色参数
// ------------------------------------------------------------
// AS7341 有 8 个可见光通道 + Clear + NIR。
// 本固件用**归一化光谱向量的最近邻**判色(比色相区间稳得多),
// 色相/饱和度只作为诊断显示与"未标定时的兜底"。
//
// 通道顺序 (下标 0..7): F1=415 F2=445 F3=480 F4=515 F5=555 F6=590 F7=630 F8=680 nm
#define SPEC_BANDS             8

// 积分时间 TINT = (ATIME + 1) × (ASTEP + 1) × 2.78 µs
// 19/599 → 20 × 600 × 2.78µs ≈ 33.4 ms。AS7341 一次完整读要跑两遍 SMUX
// (F1-F4 一遍, F5-F8 一遍), 所以单次 readAllChannels 约 70 ms。
// 想更快就减小 ATIME; 想更稳就加大(上限 255)。
#define AS7341_ATIME          19
#define AS7341_ASTEP         599
#define AS7341_GAIN          AS7341_GAIN_16X

// 模块自带白光 LED 的驱动电流 (mA), 允许 4-258
#define AS7341_LED_MA         20

// 连续采样次数, 逐通道取中位数, 抑制糖衣反光毛刺 (奇数)
#define COLOR_SAMPLE_TIMES     3
#define T_SAMPLE_GAP           5    // 单次采样之间的间隔 (ms)

// "有糖豆"的亮度门限 (Clear 通道原始值, 0-65535)
// ★ 与 ATIME/ASTEP/GAIN 和 LED 电流强相关! 换档位后必须用串口命令 't' 重新核对:
//   检测仓空着时的 Clear 值 → 设 MIN_CLEAR_LEVEL 为该值的 1.5~2 倍
#define MIN_CLEAR_LEVEL      800

// "有颜色"的最低饱和度 (0-1), 低于此值视为白色/背景
// 只在"未标定"的兜底判色里使用
#define MIN_SATURATION       0.25f

// 未标定时使用的默认色相区间 (HSV 色相, 0-360)。红色区间跨越 0°
static const float kDefaultHueMin[COLOR_COUNT] = { 345.0f, 18.0f, 42.0f,  72.0f, 165.0f };
static const float kDefaultHueMax[COLOR_COUNT] = {  18.0f, 42.0f, 72.0f, 165.0f, 260.0f };

// 已标定时: 归一化光谱向量的平方距离超过此值就判为 UNKNOWN
// 归一化后 8 个分量各约 0.125; 0.004 大致相当于每个通道平均偏 2%
#define MAX_SPECTRAL_DIST    0.004f

// ------------------------------------------------------------
// 5. 舵机行程 (单位: 度)
// ------------------------------------------------------------
// 闸板: 摇臂 20mm, 转角 65° → 直线行程约 21mm, 足够让开导管内孔
#define GATE_CLOSE_ANGLE     0   // 闸板全闭 (挡住糖豆)
#define GATE_OPEN_ANGLE     65   // 闸板全开 (放行一颗)

// ------------------------------------------------------------
// 6. 时序参数 (单位: ms), 均为可调
// ------------------------------------------------------------
#define T_GATE_MOVE         150  // 闸板开/关一次所需时间
#define T_GATE_SETTLE       120  // 糖豆落到挡板B 上后静置
#define T_COLOR_SETTLE      120  // 判色前静置 (等仓内气流/微振动平息)
#define T_SORTER_MOVE       450  // 底部摇臂最大行程(0°→170°)转位时间
#define T_DROP_HOLD         220  // 挡板B保持打开时间 (糖豆落到导流槽上)
#define T_DROP_RECOVER      180  // 挡板B关闭回位时间
#define T_BOOT_HOME_SETTLE  700  // 上电回待机位等待时间

// 到位确认
#define T_ARRIVAL_POLL       15  // TCRT5000 轮询间隔
#define T_ARRIVAL_TIMEOUT   700  // 单次到位确认超时 → 重开一次挡板A
#define MAX_ARRIVAL_RETRY     3  // 连续重试次数, 超过则判"卡料"

// 长按清计数
#define T_BUTTON_LONGPRESS 3000

// 连续多少颗未识别 → 蜂鸣提示
#define MAX_CONSECUTIVE_UNKNOWN 5

// ------------------------------------------------------------
// 7. FreeRTOS 任务参数
// ------------------------------------------------------------
#define TASK_GATE_CORE      1   // 机械时序固定到 核1
#define TASK_SORTER_CORE    1
#define TASK_COLOR_CORE     0   // I2C 采样固定到 核0 (与 OLED 同核, 靠互斥量串行)
#define TASK_UI_CORE        0

#define TASK_PRIO_GATE      4   // 闸门时序: 实时性最高
#define TASK_PRIO_SORTER    3
#define TASK_PRIO_COLOR     3
#define TASK_PRIO_UI        1   // 显示: 最低, 随时可被打断

#define STACK_GATE          3072
#define STACK_SORTER        3072
#define STACK_COLOR         4096
#define STACK_UI            4096

// 队列深度: 最多缓存几颗待分拣的颜色结果
#define QUEUE_LEN_COLOR     4

// ------------------------------------------------------------
// 8. NVS (掉电保存) 键名
// ------------------------------------------------------------
#define NVS_NAMESPACE     "candy3"

// ------------------------------------------------------------
// 9. 可选升级: 糖豆盒防架桥振动马达 (默认关闭)
//    打开后需按 03 文档 §6 焊 MOSFET 驱动电路, GPIO25 接栅极
// ------------------------------------------------------------
#define ENABLE_VIB_UPGRADE   0
#if ENABLE_VIB_UPGRADE
  #define T_VIB_ON           120  // 振动脉动开启时长
  #define T_VIB_OFF          380  // 振动脉动关闭时长
#endif
