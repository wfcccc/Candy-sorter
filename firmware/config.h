// ============================================================
//  config.h — 糖豆分拣机 硬件配置与调参中心
//  所有引脚、角度、阈值、时序都在这里改，main.cpp 无需改动
// ============================================================
#pragma once

#include <Arduino.h>

// ------------------------------------------------------------
// 1. 引脚分配 (ESP32 DevKitC)
//    注意: GPIO34-39 仅可输入; GPIO6-11 接 Flash 不可用;
//          GPIO0/2/12/15 为 strapping 引脚, 上电电平影响启动
// ------------------------------------------------------------
#define PIN_I2C_SDA        21   // I2C 数据 (TCS34725 + OLED 共用)
#define PIN_I2C_SCL        22   // I2C 时钟

#define PIN_SERVO_DIVERTER 13   // 分拣导流槽舵机 (MG996R)
#define PIN_SERVO_FEEDER   14   // 进料星轮/闸门舵机 (SG90)
#define PIN_SERVO_RELEASE   4   // 释放挡板舵机 (SG90)

#define PIN_VIB_MOTOR      25   // 振动马达 (经 MOSFET/三极管 驱动)
#define PIN_IR_SENSOR      34   // 红外检测: 检测位有无糖豆 (input only)
#define PIN_BUZZER         26   // 蜂鸣器
#define PIN_BUTTON         27   // 按键 (校准/启停, 内部上拉, 按下为低)
#define PIN_LED_STRIP      32   // WS2812 状态灯带
#define PIN_LED_ONBOARD     2   // 板载 LED

// ------------------------------------------------------------
// 2. I2C 设备地址
// ------------------------------------------------------------
#define ADDR_TCS34725      0x29 // 颜色传感器
#define ADDR_SSD1306       0x3C // OLED

// ------------------------------------------------------------
// 3. 糖豆颜色定义
// ------------------------------------------------------------
#define COLOR_COUNT        6    // 分拣种类数 (红橙黄绿蓝紫)

// 每种颜色对应的导流槽角度 (MG996R, 0-180 度)
// 导流槽转到该角度时, 正对对应的容器
static const int kBinAngle[COLOR_COUNT] = { 0, 30, 60, 90, 120, 150 };

// 导流槽待机角度 (不指向任何容器, 避免糖豆误落)
#define DIVERTER_HOME_ANGLE 90

// 颜色名字 (用于 OLED 与串口显示), 顺序须与枚举一致
static const char* const kColorName[COLOR_COUNT] = {
  "RED", "ORANGE", "YELLOW", "GREEN", "BLUE", "PURPLE"
};

// 默认色相区间 (HSV 色相, 0-360), 用于未标定时判色
// 顺序: 红 橙 黄 绿 蓝 紫
static const float kDefaultHueMin[COLOR_COUNT] = { 345.0f,  18.0f,  42.0f,  72.0f, 165.0f, 260.0f };
static const float kDefaultHueMax[COLOR_COUNT] = {  18.0f,  42.0f,  72.0f, 165.0f, 260.0f, 345.0f };

// 判定为 "有颜色" 所需的最低饱和度 (0-1), 低于此值视为白色/背景
#define MIN_SATURATION     0.28f

// 判定为 "有糖豆" 所需的最低亮度 (原始 Clear 通道, 0-65535)
#define MIN_CLEAR_LEVEL    900

// ------------------------------------------------------------
// 4. 传感器采样参数
// ------------------------------------------------------------
// TCS34725 积分时间: 影响灵敏度与抗噪, 越长越稳但越慢
// 可选: TCS34725_INTEGRATIONTIME_2_4MS / 24MS / 50MS / 101MS / 154MS / 700MS
#define TCS_INTEGRATION    TCS34725_INTEGRATIONTIME_101MS
#define TCS_GAIN           TCS34725_GAIN_4X

// 连续采样次数, 取中位数, 抑制反光毛刺
#define COLOR_SAMPLE_TIMES 7

// ------------------------------------------------------------
// 5. 舵机行程 (单位: 度), 按你的机械结构实测微调
// ------------------------------------------------------------
#define FEEDER_IDLE_ANGLE      0    // 进料轮待机
#define FEEDER_STEP_ANGLE     60    // 进料轮每次转动角度 (一个工位)

#define RELEASE_CLOSE_ANGLE    0    // 释放挡板关闭 (挡住糖豆)
#define RELEASE_OPEN_ANGLE    85    // 释放挡板打开 (放行糖豆)

// ------------------------------------------------------------
// 6. 时序参数 (单位: 毫秒), 均为可调
// ------------------------------------------------------------
#define T_SETTLE_AFTER_FEED    220  // 进料后等待糖豆静止
#define T_COLOR_SETTLE         120  // 判色前静置(消除振动)
#define T_DIVERTER_MOVE        380  // 导流槽转到位所需时间
#define T_RELEASE_HOLD         260  // 释放挡板保持打开时间
#define T_RELEASE_RECOVER      200  // 释放挡板回位时间
#define T_FEEDER_CYCLE        1500  // 两次进料之间的最小间隔
#define T_VIB_ON               120  // 振动马达脉动开启时长
#define T_VIB_OFF              380  // 振动马达脉动关闭时长
#define T_VIB_TIMEOUT        15000  // 送料超时(ms), 超时报警

// 连续多少颗未识别 → 提示卡料
#define MAX_CONSECUTIVE_UNKNOWN 3

// ------------------------------------------------------------
// 7. FreeRTOS 任务参数
// ------------------------------------------------------------
#define TASK_COLOR_CORE     0   // 判色任务固定到 核0
#define TASK_SORTER_CORE    1   // 分拣任务固定到 核1
#define TASK_FEEDER_CORE    1
#define TASK_UI_CORE        0

#define TASK_PRIO_SORTER    3   // 分拣: 实时性最高
#define TASK_PRIO_COLOR     2
#define TASK_PRIO_FEEDER    2
#define TASK_PRIO_UI        1   // 显示: 最低

#define STACK_COLOR         4096
#define STACK_SORTER        4096
#define STACK_FEEDER        3072
#define STACK_UI            4096

// 队列深度: 最多缓存几颗待分拣的颜色结果
#define QUEUE_LEN_COLOR     4

// ------------------------------------------------------------
// 8. NVS (掉电保存) 键名
// ------------------------------------------------------------
#define NVS_NAMESPACE       "candy"
