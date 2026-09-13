// ============================================================
//  三舵机糖豆分色器 v3 (Candy Sorter v3)
//  主控: ESP32 DevKitC | 框架: Arduino + FreeRTOS
// ------------------------------------------------------------
//  机械链 (泡棉板 L 形骨架):
//    糖豆盒 → 单列导管 → 挡板A(上) → [单颗测色仓] → 挡板B(下)
//    (A 与 B 之间就是检测仓; B 同时充当仓底板与放料门)
//    → 摇臂上的斜导流槽 → 5 个方形收纳盒 + 1 个废料盒
//
//  传感器: AS7341 11 通道光谱传感器 (I2C 0x39, 板载白光 LED)
//          TCS34725 只有 3 个宽带通道, AS7341 有 8 个可见光窄带通道,
//          所以本固件用**归一化光谱向量的最近邻**判色, 而不是色相区间。
//
//  数据流 (生产者-消费者, 由 FreeRTOS 调度):
//
//    [TaskGate]           [TaskColor]         [TaskSorter]        [TaskUI]
//     开A放一颗       ──▶   读光谱判色     ──▶   转摇臂+B放料   ──▶  OLED/灯/串口
//     TCRT5000到位确认     中值滤波+光谱近邻     计数
//         ▲                                          │
//         └──────────── semNextCandy ────────────────┘
//
//  同步原语:
//    semCandyOnB    二值信号量: Gate   -> Color   糖豆已停在挡板B(检测仓底板)上
//    queueColor     队列(深度4): Color -> Sorter  判色结果
//    semNextCandy   二值信号量: Sorter -> Gate    上一颗已放行, 可以放下一颗
//    i2cMutex       互斥量    : AS7341 与 OLED 共用 I2C 总线
//    countMutex     互斥量    : 保护计数变量
//
//  串口(115200)命令 —— 详见 'h':
//    c <0-4>  标定检测仓里那颗糖豆       s  保存到 Flash    r  清除标定
//    p  打印状态与计数                  z  计数清零        u  暂停/继续
//    a <0|1> 手动开关挡板A              b <0|1> 手动开关挡板B
//    d <角度> 手动转底部摇臂             m  进入手动模式并演示一次完整动作
//    x  退出手动模式, 恢复自动分拣        t  切换原始光谱数据流
// ============================================================

#include <Arduino.h>
#include <Wire.h>

// ---- 第三方库 ----
#include <Adafruit_AS7341.h>
#include <ESP32Servo.h>
#include <U8g2lib.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// ---- FreeRTOS ----
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "config.h"

// ============================================================
//  全局对象
// ============================================================
Adafruit_AS7341 as7341;

Servo servoGateA;    // 挡板A (上)    —— 只由 TaskGate 操作
Servo servoGateB;    // 挡板B (下, 兼放料门) —— 只由 TaskSorter 操作
Servo servoSort;     // 底部摇臂      —— 只由 TaskSorter 操作

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_NeoPixel strip(1, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
Preferences prefs;

// ---- 同步原语 ----
static SemaphoreHandle_t i2cMutex        = nullptr;
static SemaphoreHandle_t countMutex      = nullptr;
static SemaphoreHandle_t semCandyOnB     = nullptr;
static SemaphoreHandle_t semNextCandy    = nullptr;
static QueueHandle_t     queueColor      = nullptr;

// ---- 共享状态 ----
typedef enum {
  STATE_BOOT = 0,
  STATE_IDLE,
  STATE_METERING,      // 开A, 放一颗到B
  STATE_WAIT_ARRIVAL,  // TCRT5000 确认到位
  STATE_SENSING,       // AS7341 判色
  STATE_SORTING,       // 底部摇臂转位
  STATE_RELEASING,     // 开B放料
  STATE_JAM,
  STATE_NO_SENSOR,
  STATE_PAUSED,
  STATE_MANUAL
} SystemState;

static volatile SystemState g_state = STATE_BOOT;

// 自动运行 / 手动调试 / 暂停
static volatile bool g_running    = true;   // false = 暂停自动分拣
static volatile bool g_manualMode = false;  // true  = 任人手动操作舵机
static volatile bool g_gateIdle   = true;   // TaskGate  当前未在动舵机
static volatile bool g_sorterIdle = true;   // TaskSorter 当前未在动舵机

static uint32_t g_count[COLOR_COUNT] = {0};  // 各颜色计数 (受 countMutex 保护)
static uint32_t g_unknown   = 0;             // 未识别计数
static uint32_t g_total     = 0;             // 总计数
static volatile uint32_t g_jamCount  = 0;    // 卡料/到位失败次数
static volatile bool     g_rawStream = false;// 串口原始数据流开关
static volatile int      g_consecUnknown = 0;// 连续判不出颜色的颗数
// 上电自检失败锁存 (TCRT5000 悬空 / AS7341 不在总线上)。
// 一旦置位就拒绝自动分拣 —— 否则会凭空确认糖豆到位、空放、空计数。
static volatile bool     g_sensorFault = false;

// ---- 标定数据 ----
static float g_specRef[COLOR_COUNT][SPEC_BANDS]; // 归一化参考光谱 (每色一条)
static float g_hueRef[COLOR_COUNT];              // 参考色相, 仅用于显示
static bool  g_calibrated = false;

// ---- 最近一次判色结果 (给 UI 显示) ----
static volatile int   g_lastColor = -1;
static volatile float g_lastHue   = -1.0f;
static volatile float g_lastSat   =  0.0f;
static volatile float g_lastDist  = -1.0f;   // 与最近参考光谱的平方距离

// ---- 前向声明 ----
static void TaskGate(void* pv);
static void TaskColor(void* pv);
static void TaskSorter(void* pv);
static void TaskUI(void* pv);

// ============================================================
//  工具函数
// ============================================================

// 蜂鸣器: 用软件方波, 避免与 ESP32Servo 抢占 LEDC 定时器
static void beep(int times, int ms = 70, int freq = 2400) {
  const uint32_t half = 1000000UL / (uint32_t)freq / 2UL;
  for (int t = 0; t < times; t++) {
    const uint32_t end = millis() + (uint32_t)ms;
    while ((int32_t)(millis() - end) < 0) {
      digitalWrite(PIN_BUZZER, HIGH);
      delayMicroseconds(half);
      digitalWrite(PIN_BUZZER, LOW);
      delayMicroseconds(half);
    }
    if (t + 1 < times) vTaskDelay(pdMS_TO_TICKS(90));
  }
}

// 底部摇臂角度: 加装配微调并夹到 0-180
static inline int sorterAngle(int deg) {
  int a = deg + SORTER_TRIM_DEG;
  if (a < 0)   a = 0;
  if (a > 180) a = 180;
  return a;
}

// 色相环上的最短角距离 (0-180)
static inline float hueDistance(float a, float b) {
  const float d = fabsf(a - b);
  return (d > 180.0f) ? (360.0f - d) : d;
}

// 色相区间的"中点"。红色区间是 [345,360) ∪ [0,18), 跨越 0°,
// 直接取 (345+18)/2 = 181.5° 是错的(那是青色!), 必须走环形中点。
static inline float hueMid(float lo, float hi) {
  if (lo < hi) return (lo + hi) * 0.5f;
  float m = (lo + hi + 360.0f) * 0.5f;
  if (m >= 360.0f) m -= 360.0f;
  return m;
}

// 由伪 RGB 计算 HSV 色相与饱和度 (仅用于显示与未标定时的兜底判色)
static float computeHue(float r, float g, float b, float* satOut) {
  const float mx = fmaxf(r, fmaxf(g, b));
  const float mn = fminf(r, fminf(g, b));
  const float d  = mx - mn;
  if (mx <= 0.0f) { *satOut = 0.0f; return 0.0f; }
  *satOut = d / mx;
  if (d < 1e-6f) return 0.0f;
  float h;
  if (mx == r)      h = 60.0f * fmodf(((g - b) / d), 6.0f);
  else if (mx == g) h = 60.0f * (((b - r) / d) + 2.0f);
  else              h = 60.0f * (((r - g) / d) + 4.0f);
  if (h < 0.0f) h += 360.0f;
  return h;
}

// 8 个光谱通道 → 伪 RGB。
// 只是为了给人看一个色相值 + 未标定时的兜底判色, 不参与正式判色。
//   F1=415 F2=445 F3=480 F4=515 F5=555 F6=590 F7=630 F8=680 nm
static void spectrumToRgb(const uint16_t* f8, float* r, float* g, float* b) {
  *b = (float)(f8[1] + f8[2]);           // 445 + 480
  *g = (float)(f8[3] + f8[4]);           // 515 + 555
  *r = (float)(f8[5] + f8[6] + f8[7]);   // 590 + 630 + 680
}

// 未标定时的兜底判色: 查默认色相区间 (红色区间跨越 0°)
static int classifyColor(float hue, float sat) {
  if (sat < MIN_SATURATION) return -1;
  for (int i = 0; i < COLOR_COUNT; i++) {
    const float lo = kDefaultHueMin[i];
    const float hi = kDefaultHueMax[i];
    if (lo < hi) { if (hue >= lo && hue < hi) return i; }
    else         { if (hue >= lo || hue < hi) return i; }
  }
  return -1;
}

// 取中位数 (原地插入排序, n 很小)
static uint16_t medianU16(uint16_t* a, int n) {
  for (int i = 1; i < n; i++) {
    const uint16_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
  return a[n / 2];
}

// 把 8 个通道归一化成"光谱形状"向量(各分量之和为 1), 消掉亮度差异。
// 判色只看颜色成分, 不该被糖豆离传感器远近影响 —— 这一步是关键。
static void normalizeSpectrum(const uint16_t* f8, float* out) {
  float sum = 0.0f;
  for (int i = 0; i < SPEC_BANDS; i++) sum += (float)f8[i];
  if (sum <= 1.0f) {
    for (int i = 0; i < SPEC_BANDS; i++) out[i] = 0.0f;
    return;
  }
  for (int i = 0; i < SPEC_BANDS; i++) out[i] = (float)f8[i] / sum;
}

static inline float spectrumDistance(const float* a, const float* b) {
  float d = 0.0f;
  for (int i = 0; i < SPEC_BANDS; i++) { const float t = a[i] - b[i]; d += t * t; }
  return d;
}

// ---- AS7341 单次读取 ----
// ★ 注意: readAllChannels(buf) 的缓冲顺序是 F1..F8, CLEAR, NIR,
//   而 getChannel() 用的 as7341_color_channel_t 枚举顺序把 CLEAR_0/NIR_0
//   插在了 F4 和 F5 中间 —— 两者**不一致**。所以这里统一走
//   readAllChannels() + getChannel(), 用枚举名取值, 不碰裸缓冲下标。
// 另: AS7341 只有 6 个 ADC 通道, 内部要跑两遍 SMUX(F1-F4 / F5-F8),
//   每遍各等一个 TINT, 所以一次完整读取约 2×TINT ≈ 70ms。
static bool readAs7341Once(uint16_t* f8, uint16_t* clearOut, uint16_t* nirOut) {
  if (!as7341.readAllChannels()) return false;   // 内部自带等待与超时
  f8[0] = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
  f8[1] = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
  f8[2] = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
  f8[3] = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
  f8[4] = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
  f8[5] = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
  f8[6] = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
  f8[7] = as7341.getChannel(AS7341_CHANNEL_680nm_F8);
  *clearOut = as7341.getChannel(AS7341_CHANNEL_CLEAR);
  *nirOut   = as7341.getChannel(AS7341_CHANNEL_NIR);
  return true;
}

// 采多次取中位数, 抑制糖衣反光毛刺; 返回 true 表示亮度足够(有糖豆)
static bool readSpectrumMedian(uint16_t* outF8, uint16_t* outClear, uint16_t* outNir) {
  uint16_t s[SPEC_BANDS][COLOR_SAMPLE_TIMES];
  uint16_t clr[COLOR_SAMPLE_TIMES], nir[COLOR_SAMPLE_TIMES];

  // 先清零输出, 保证任何一条失败路径上调用方读到的都是 0 而不是栈垃圾
  for (int b = 0; b < SPEC_BANDS; b++) outF8[b] = 0;
  *outClear = 0;
  *outNir   = 0;

  for (int k = 0; k < COLOR_SAMPLE_TIMES; k++) {
    // ★ i2cMutex 要罩住整个 readAllChannels: 它内部有多次寄存器读写和
    //   两段等待, 中途绝不能让 OLED 插进来抢总线
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1500)) != pdTRUE) return false;
    uint16_t f[SPEC_BANDS], c = 0, n = 0;
    const bool ok = readAs7341Once(f, &c, &n);
    xSemaphoreGive(i2cMutex);
    if (!ok) return false;

    for (int b = 0; b < SPEC_BANDS; b++) s[b][k] = f[b];
    clr[k] = c;
    nir[k] = n;
    vTaskDelay(pdMS_TO_TICKS(T_SAMPLE_GAP));
  }

  for (int b = 0; b < SPEC_BANDS; b++) outF8[b] = medianU16(s[b], COLOR_SAMPLE_TIMES);
  *outClear = medianU16(clr, COLOR_SAMPLE_TIMES);
  *outNir   = medianU16(nir, COLOR_SAMPLE_TIMES);

  return (*outClear >= MIN_CLEAR_LEVEL);
}

// 判色: 返回 0..COLOR_COUNT-1, 或 -1 表示未知
// 已标定 → 归一化光谱向量的最近邻; 未标定 → 伪 RGB 的色相区间兜底
static int classifySpectrum(const uint16_t* f8, float hue, float sat, float* distOut) {
  if (!g_calibrated) {
    if (distOut) *distOut = -1.0f;
    return classifyColor(hue, sat);
  }

  float v[SPEC_BANDS];
  normalizeSpectrum(f8, v);

  float sum = 0.0f;
  for (int i = 0; i < SPEC_BANDS; i++) sum += v[i];
  if (sum <= 0.0f) {                 // 全黑, 没有有效光谱
    if (distOut) *distOut = -1.0f;
    return -1;
  }

  int   best  = -1;
  float bestD = 1e9f;
  for (int c = 0; c < COLOR_COUNT; c++) {
    const float d = spectrumDistance(v, g_specRef[c]);
    if (d < bestD) { bestD = d; best = c; }
  }
  if (distOut) *distOut = bestD;
  return (bestD <= MAX_SPECTRAL_DIST) ? best : -1;
}

// 状态灯颜色
static void setStrip(int color) {
  uint8_t r = 40, g = 40, b = 40;              // UNKNOWN -> 灰
  switch (color) {
    case 0: r = 255; g =   0; b =   0; break;  // RED
    case 1: r = 255; g =  90; b =   0; break;  // ORANGE
    case 2: r = 255; g = 210; b =   0; break;  // YELLOW
    case 3: r =   0; g = 200; b =   0; break;  // GREEN
    case 4: r =   0; g =  90; b = 255; break;  // BLUE
    default: break;
  }
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

// 读取并分类一次 (供判色任务与标定功能复用)
static int sampleAndClassify(float* hueOut, float* satOut,
                             uint16_t* rawF8 = nullptr,
                             uint16_t* rawClear = nullptr,
                             uint16_t* rawNir = nullptr,
                             float*    distOut = nullptr) {
  uint16_t f8[SPEC_BANDS], c = 0, n = 0;
  const bool lit = readSpectrumMedian(f8, &c, &n);

  if (rawF8)    for (int i = 0; i < SPEC_BANDS; i++) rawF8[i] = f8[i];
  if (rawClear) *rawClear = c;
  if (rawNir)   *rawNir   = n;

  if (!lit) {
    if (hueOut)  *hueOut  = -1.0f;
    if (satOut)  *satOut  =  0.0f;
    if (distOut) *distOut = -1.0f;
    return -1;
  }

  float r, g, b, sat = 0.0f;
  spectrumToRgb(f8, &r, &g, &b);
  const float hue = computeHue(r, g, b, &sat);

  if (hueOut) *hueOut = hue;
  if (satOut) *satOut = sat;
  return classifySpectrum(f8, hue, sat, distOut);
}

// ============================================================
//  Task 1: 闸门 (挡板A) —— 一次只放一颗到挡板B(检测仓底板)
//    优先级最高, 只做机械时序与到位确认, 不做任何耗时 I2C
// ============================================================
static void TaskGate(void* pv) {
  (void)pv;

  servoGateA.attach(PIN_SERVO_GATE_A, SERVO_MIN_US, SERVO_MAX_US);
  servoGateA.write(GATE_CLOSE_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(300));

  Serial.println(F("[Gate] task started"));

  for (;;) {
    g_gateIdle = true;   // 告诉 UI: 现在可以安全地手动接管舵机

    if (g_manualMode) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if (g_sensorFault) {                       // 自检没过: 拒绝自动分拣
      g_state = STATE_NO_SENSOR;
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (!g_running)   { g_state = STATE_PAUSED; vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    // ---- 等 Sorter 放行 (上一颗已落盒) ----
    if (xSemaphoreTake(semNextCandy, pdMS_TO_TICKS(20000)) != pdTRUE) {
      continue;   // 长时间空闲, 继续等
    }
    g_gateIdle = false;
    if (g_manualMode) { xSemaphoreGive(semNextCandy); continue; }

    // ---- 1. 开A放一颗, 再关A ----
    //     A/B 闸板间距 = 一颗糖豆, 所以只会掉下一颗; A 关闭时刀刃正好落在
    //     两颗糖豆的切点上, 把上面的整列停住
    bool ok = false;
    for (int attempt = 0; attempt < MAX_ARRIVAL_RETRY && !ok; attempt++) {
      // ---- 关键: 每次重新开挡板A 之前, 先确认检测仓是空的 ----
      // 上一轮超时后糖豆可能紧接着就滑到位了. 这时若再开一次A, 就会在
      // 挡板B 上叠第二颗 —— 两颗一起判色、一起落进同一个盒子, 而且只计一次数.
      if (digitalRead(PIN_IR_SENSOR) == LOW) { ok = true; break; }
      if (g_manualMode) break;

      g_state = STATE_METERING;
      servoGateA.write(GATE_OPEN_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(T_GATE_MOVE));
      servoGateA.write(GATE_CLOSE_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(T_GATE_MOVE));
      vTaskDelay(pdMS_TO_TICKS(T_GATE_SETTLE));

      // ---- 2. TCRT5000 确认糖豆已停在挡板B上 ----
      g_state = STATE_WAIT_ARRIVAL;
      const uint32_t t0 = millis();
      while ((uint32_t)(millis() - t0) < T_ARRIVAL_TIMEOUT) {
        if (digitalRead(PIN_IR_SENSOR) == LOW) { ok = true; break; }  // LOW = 检测到
        vTaskDelay(pdMS_TO_TICKS(T_ARRIVAL_POLL));
      }
      if (!ok) {
        Serial.printf("[Gate] arrival timeout (attempt %d/%d)\n",
                      attempt + 1, MAX_ARRIVAL_RETRY);
      }
    }

    // 切到手动模式了: 把节拍原样还回去, 不做任何动作、不报警
    if (g_manualMode) {
      xSemaphoreGive(semNextCandy);
      continue;
    }

    // 判 JAM 之前再给最后一次机会 (糖豆可能刚刚才滑到位)
    if (!ok && digitalRead(PIN_IR_SENSOR) == LOW) ok = true;

    if (!ok) {
      // ---- 连续放不下来: 糖豆盒架空 / 单列导管卡豆 ----
      g_state = STATE_JAM;
      g_jamCount++;
      Serial.println(F("[Gate] JAM: no candy reached gate B. hopper empty or bridging?"));
      beep(3);
      vTaskDelay(pdMS_TO_TICKS(1200));
      xSemaphoreGive(semNextCandy);   // 归还节拍, 再试一次
      continue;
    }

    // ---- 3. 通知判色任务 ----
    vTaskDelay(pdMS_TO_TICKS(T_COLOR_SETTLE));
    g_state = STATE_SENSING;
    xSemaphoreGive(semCandyOnB);
    // 循环顶部重新等 semNextCandy (由 Sorter 在放料完成后给出)
  }
}

// ============================================================
//  Task 2: 判色 (读光谱 -> 逐通道中值滤波 -> 归一化最近邻)
//    固定在核0, 与 OLED 共用 I2C, 由 i2cMutex 串行
// ============================================================
static void TaskColor(void* pv) {
  (void)pv;
  Serial.println(F("[Color] task started"));

  for (;;) {
    // 手动模式下绝不能把 semCandyOnB 取走 —— 取走就等于把这颗糖豆的判色
    // 令牌弄丢了, 整条 闸门->判色->放料->闸门 的环路再也接不上.
    if (g_manualMode) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    if (xSemaphoreTake(semCandyOnB, pdMS_TO_TICKS(20000)) != pdTRUE) {
      continue;   // 长时间没糖豆
    }
    if (g_manualMode) {
      xSemaphoreGive(semCandyOnB);   // 极端时序下命中: 原样还回去
      continue;
    }

    float hue = -1.0f, sat = 0.0f, dist = -1.0f;
    uint16_t f8[SPEC_BANDS], clr = 0, nir = 0;
    const int color = sampleAndClassify(&hue, &sat, f8, &clr, &nir, &dist);

    g_lastColor = color;
    g_lastHue   = hue;
    g_lastSat   = sat;
    g_lastDist  = dist;

    Serial.printf("[Color] Clear=%u NIR=%u hue=%.1f sat=%.2f dist=%.4f -> %s\n",
                  clr, nir, hue, sat, dist,
                  (color >= 0) ? kColorName[color] : "UNKNOWN");

    // 无论识别成功与否都要入队, 否则糖豆会永远堵在检测仓里.
    // 入队失败(队列满)时不能静默丢弃: 那颗糖豆就再也不会有判色结果,
    // 只能靠 TaskSorter 里的看门狗按 UNKNOWN 放行, 这里至少报一声.
    if (xQueueSend(queueColor, &color, pdMS_TO_TICKS(2000)) != pdTRUE) {
      Serial.println(F("[Color] WARN: queue full, result dropped (sorter watchdog will release it)"));
    }
  }
}

// ============================================================
//  Task 3: 放料 (转摇臂 -> 开B -> 关B -> 计数 -> 放行下一颗)
//    拥有挡板B与底部摇臂舵机
// ============================================================
static void TaskSorter(void* pv) {
  (void)pv;

  // 上电: 挡板B关闭, 底部摇臂回待机位(废料位)
  servoGateB.attach(PIN_SERVO_GATE_B, SERVO_MIN_US, SERVO_MAX_US);
  servoGateB.write(GATE_CLOSE_ANGLE);

  servoSort.attach(PIN_SERVO_SORTER, SERVO_MIN_US, SERVO_MAX_US);
  servoSort.write(sorterAngle(SORTER_HOME_ANGLE));
  vTaskDelay(pdMS_TO_TICKS(T_BOOT_HOME_SETTLE));

  Serial.printf("[Sorter] task started, home=%d\n", sorterAngle(SORTER_HOME_ANGLE));

  for (;;) {
    g_sorterIdle = true;   // 告诉 UI: 现在可以安全地手动接管舵机
    if (g_manualMode) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    int  color = -1;
    bool have  = (xQueueReceive(queueColor, &color, pdMS_TO_TICKS(5000)) == pdTRUE);

    if (have && g_manualMode) {
      // 手动模式是在上面那次 5s 等待期间被打开的. 这一条判色结果对应的糖豆
      // 还压在挡板B 上, 绝不能扔 —— 原样放回队列头部, 交给 exitManual() 处理.
      // (如果这里 continue 丢掉它, semNextCandy 就再也还不上, 整机会静默停摆)
      (void)xQueueSendToFront(queueColor, &color, pdMS_TO_TICKS(100));
      have = false;
      continue;
    }

    if (!have) {
      // 手动模式是在那次 5s 等待期间被打开的: 立即交还舵机, 不要动任何东西
      if (g_manualMode) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

      // 空闲超时: 回待机位, 这样万一有糖豆误落也只会落到废料盒
      servoSort.write(sorterAngle(SORTER_HOME_ANGLE));

      // ---- 看门狗 ----
      // 正常流程下, 检测仓里有糖豆就一定有判色结果跟着来. 如果 TCRT 说
      // 糖豆还压在挡板B 上、却迟迟等不到结果 (队列满导致 xQueueSend 失败、
      // 或 TaskColor 异常), 整机会永久卡死 —— 这里按 UNKNOWN 强行放行.
      if (digitalRead(PIN_IR_SENSOR) == LOW) {
        Serial.println(F("[Sorter] WATCHDOG: candy on gate B with no color result -> drop as UNKNOWN"));
        g_jamCount++;
        color = -1;
        have  = true;
        setStrip(-1);
      } else {
        // 不要覆盖 JAM / NO SENSOR 这类故障显示, 否则操作员根本看不到
        if (g_state != STATE_JAM && g_state != STATE_NO_SENSOR) {
          g_state = g_running ? STATE_IDLE : STATE_PAUSED;
        }
        continue;
      }
    }

    g_sorterIdle = false;

    // ---- 1. 底部摇臂转到目标盒位 (未知颜色 -> 废料位) ----
    g_state = STATE_SORTING;
    const int angle = (color >= 0) ? kBinAngle[color] : SORTER_HOME_ANGLE;
    servoSort.write(sorterAngle(angle));
    setStrip(color);
    vTaskDelay(pdMS_TO_TICKS(T_SORTER_MOVE));

    // ---- 2. 打开挡板B, 糖豆落到摇臂的斜导流槽上, 靠重力滚进盒子 ----
    g_state = STATE_RELEASING;
    servoGateB.write(GATE_OPEN_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(T_DROP_HOLD));

    // ---- 3. 关闭挡板B, 并确认糖豆真的落下去了 ----
    //     不确认的话, 一颗粘在闸板上的糖豆会被照常计数, 下一颗还会叠上来
    servoGateB.write(GATE_CLOSE_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(T_DROP_RECOVER));

    bool released = (digitalRead(PIN_IR_SENSOR) != LOW);
    for (int retry = 0; !released && retry < 2; retry++) {
      Serial.println(F("[Sorter] candy still on gate B -> re-open"));
      servoGateB.write(GATE_OPEN_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(T_DROP_HOLD));
      servoGateB.write(GATE_CLOSE_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(T_DROP_RECOVER));
      released = (digitalRead(PIN_IR_SENSOR) != LOW);
    }

    if (!released) {
      // 糖豆真卡住了 (或 TCRT 一直报有豆). 不计入任何颜色计数, 归还节拍让
      // TaskGate 顶部那道"先确认检测仓是空的"检查接管, 下一轮重新处理它.
      g_jamCount++;
      Serial.println(F("[Sorter] JAM: gate B did not clear, will retry next cycle"));
      beep(2, 60, 1500);
      g_state = STATE_JAM;
      xSemaphoreGive(semNextCandy);
      continue;
    }

    // ---- 4. 糖豆确实落盒了, 计数 (加锁保护) ----
    if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
      if (color >= 0) g_count[color]++;
      else            g_unknown++;
      g_total++;
      xSemaphoreGive(countMutex);
    }

    // ---- 4b. 连续多颗判不出颜色 → 提示多半是没标定 / 遮光罩漏光 ----
    if (color < 0) {
      if (++g_consecUnknown >= MAX_CONSECUTIVE_UNKNOWN) {
        g_consecUnknown = 0;
        Serial.println(F("[Sorter] WARN: too many UNKNOWN in a row. "
                         "run 't' to check the spectrum, then re-calibrate ('c' + 's')"));
        beep(2, 60, 1800);
      }
    } else {
      g_consecUnknown = 0;
    }

    // ---- 5. 放行下一颗 ----
    //     注意: 底部摇臂**故意不马上回待机位**。它停在刚用过的盒位,
    //     等下一颗的判色结果出来后再转, 省掉一次 450ms 的往返
    g_state = g_running ? STATE_IDLE : STATE_PAUSED;
    xSemaphoreGive(semNextCandy);
  }
}

// ============================================================
//  Task 4: 人机界面 (OLED + 状态灯 + 按键 + 串口命令)
//    优先级最低, 随时可被打断
// ============================================================
static void drawOled() {
  char line[26];

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // ---- 标题行 + 状态 ----
  u8g2.drawStr(0, 9, "CANDY SORTER");
  const char* st = "BOOT";
  switch (g_state) {
    case STATE_IDLE:         st = "IDLE";    break;
    case STATE_METERING:     st = "METER";   break;
    case STATE_WAIT_ARRIVAL: st = "ARRIVE";  break;
    case STATE_SENSING:      st = "SENSE";   break;
    case STATE_SORTING:      st = "SORT";    break;
    case STATE_RELEASING:    st = "DROP";    break;
    case STATE_JAM:          st = "JAM!";    break;
    case STATE_NO_SENSOR:    st = "NO SENS"; break;
    case STATE_PAUSED:       st = "PAUSE";   break;
    case STATE_MANUAL:       st = "MANUAL";  break;
    default: break;
  }
  if (g_manualMode)        st = "MANUAL";
  else if (g_sensorFault)  st = "FAULT";   // 上电自检没过, 拒绝自动分拣
  u8g2.drawStr(128 - 6 * (int)strlen(st), 9, st);

  u8g2.drawHLine(0, 12, 128);

  // ---- 5 色计数 (两列: 左3右2) ----
  for (int i = 0; i < COLOR_COUNT; i++) {
    const int col = (i < 3) ? 0 : 1;
    const int row = (i < 3) ? i : (i - 3);
    const int x = col * 64;
    const int y = 24 + row * 11;
    uint32_t cnt;
    if (xSemaphoreTake(countMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      cnt = g_count[i];
      xSemaphoreGive(countMutex);
    } else {
      cnt = 0;
    }
    snprintf(line, sizeof(line), "%-6s %3lu", kColorName[i], (unsigned long)cnt);
    u8g2.drawStr(x, y, line);
  }

  // 右下角: 标定状态 + 最近一次判到的颜色 + 与参考光谱的距离
  const float d = g_lastDist;
  if (d >= 0.0f) {
    snprintf(line, sizeof(line), "CAL%c %.3s %.3f",
             g_calibrated ? '+' : '-',
             (g_lastColor >= 0) ? kColorName[g_lastColor] : "---", d);
  } else {
    snprintf(line, sizeof(line), "CAL%c %.3s --",
             g_calibrated ? '+' : '-',
             (g_lastColor >= 0) ? kColorName[g_lastColor] : "---");
  }
  u8g2.drawStr(64, 46, line);

  // ---- 底部: 总数 / 未知 / 卡料 ----
  if (xSemaphoreTake(countMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    const uint32_t tot = g_total, unk = g_unknown;
    xSemaphoreGive(countMutex);
    snprintf(line, sizeof(line), "TOT%lu UNK%lu J%lu",
             (unsigned long)tot, (unsigned long)unk, (unsigned long)g_jamCount);
  } else {
    snprintf(line, sizeof(line), "---");
  }
  u8g2.drawStr(0, 62, line);

  u8g2.sendBuffer();
  xSemaphoreGive(i2cMutex);
}

// ---- 进入 / 退出手动调试模式 ----
// 把停在挡板B 上的残留糖豆放到废料位。手动模式进/出、异常恢复都要用它,
// 否则下一次开挡板A 会在这颗糖豆上面再叠一颗。
static void dropResidualCandy() {
  servoSort.write(sorterAngle(SORTER_HOME_ANGLE));
  vTaskDelay(pdMS_TO_TICKS(T_SORTER_MOVE));

  if (digitalRead(PIN_IR_SENSOR) != LOW) return;   // 检测仓本来就是空的
  Serial.println(F(">> candy on gate B -> dropped to waste box"));
  for (int i = 0; i < 3; i++) {
    servoGateB.write(GATE_OPEN_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(T_DROP_HOLD));
    servoGateB.write(GATE_CLOSE_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(T_DROP_RECOVER));
    if (digitalRead(PIN_IR_SENSOR) != LOW) break;
  }
}

static void enterManual() {
  g_running    = false;
  g_manualMode = true;
  // 等两个机械任务把当前动作做完并让出舵机
  const uint32_t t0 = millis();
  bool idle = false;
  while ((uint32_t)(millis() - t0) < 4000) {
    if (g_gateIdle && g_sorterIdle) { idle = true; break; }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  if (!idle) {
    Serial.println(F("WARN: mechanical tasks still busy, taking over servos anyway"));
  }
  vTaskDelay(pdMS_TO_TICKS(150));

  // 先清场: 队列里的旧结果 + 挡板B 上的残留糖豆
  if (queueColor) xQueueReset(queueColor);
  dropResidualCandy();
  servoGateA.write(GATE_CLOSE_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(T_DROP_RECOVER));

  g_state = STATE_MANUAL;
  Serial.println(F(">> MANUAL mode. a<0|1> b<0|1> d<angle> m=demo  x=exit"));
}

static void exitManual() {
  // 手动模式下挡板B 上可能还停着糖豆. 直接恢复自动的话 TaskGate 会再放一颗
  // 到 B 上, 两颗叠在一起 —— 所以先把它送到废料盒扔掉.
  dropResidualCandy();

  servoGateA.write(GATE_CLOSE_ANGLE);
  servoGateB.write(GATE_CLOSE_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(T_DROP_RECOVER));

  // 清掉队列与判色令牌里可能残留的旧结果
  if (queueColor)   xQueueReset(queueColor);
  if (semCandyOnB)  (void)xSemaphoreTake(semCandyOnB, 0);

  g_manualMode = false;
  g_running    = true;
  g_state      = STATE_IDLE;
  xSemaphoreGive(semNextCandy);   // 重新启动节拍
  Serial.println(F(">> AUTO mode resumed"));
}

// 手动模式: 演示一次完整动作 (A -> B -> 底摇臂扫描 -> 放料)
static void manualDemo() {
  Serial.println(F(">> demo: gate A open/close"));
  servoGateA.write(GATE_OPEN_ANGLE);  vTaskDelay(pdMS_TO_TICKS(T_GATE_MOVE));
  servoGateA.write(GATE_CLOSE_ANGLE); vTaskDelay(pdMS_TO_TICKS(T_GATE_MOVE));
  vTaskDelay(pdMS_TO_TICKS(T_GATE_SETTLE));

  Serial.println(F(">> demo: rocker sweep 0 -> 180 -> 0"));
  for (int a = 0; a <= 180; a += 15) {
    servoSort.write(sorterAngle(a));
    vTaskDelay(pdMS_TO_TICKS(120));
  }
  for (int a = 180; a >= 0; a -= 15) {
    servoSort.write(sorterAngle(a));
    vTaskDelay(pdMS_TO_TICKS(120));
  }

  Serial.println(F(">> demo: gate B open/close"));
  servoGateB.write(GATE_OPEN_ANGLE);  vTaskDelay(pdMS_TO_TICKS(T_DROP_HOLD));
  servoGateB.write(GATE_CLOSE_ANGLE); vTaskDelay(pdMS_TO_TICKS(T_DROP_RECOVER));
  Serial.println(F(">> demo done"));
}

// ---- 串口命令 ----
static void printHelp() {
  Serial.println(F("---- commands ----"));
  Serial.printf (" c <0-%d>  calibrate candy in the chamber as color n\n", COLOR_COUNT - 1);
  Serial.println(F(" s        save calibration to flash"));
  Serial.println(F(" r        reset calibration (use built-in hue ranges)"));
  Serial.println(F(" p        print status & counters"));
  Serial.println(F(" z        zero counters"));
  Serial.println(F(" u        pause / resume sorting"));
  Serial.println(F(" t        toggle raw spectrum stream (for light/calibration tuning)"));
  Serial.println(F(" m        enter MANUAL mode + run one demo cycle"));
  Serial.println(F(" a <0|1>  manual gate A close/open   (manual mode only)"));
  Serial.println(F(" b <0|1>  manual gate B close/open   (manual mode only)"));
  Serial.println(F(" d <deg>  manual rocker servo angle  (manual mode only)"));
  Serial.println(F(" x        exit manual mode, resume auto"));
  Serial.println(F(" h        this help"));
}

static void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  const char c0 = cmd.charAt(0);

  if (c0 == 'h' || c0 == 'H' || c0 == '?') {
    printHelp();

  } else if (c0 == 'c' || c0 == 'C') {
    // 标定: c <0-4>  —— 存的是归一化光谱向量 + 色相
    const int idx = cmd.substring(1).toInt();
    if (idx < 0 || idx >= COLOR_COUNT) {
      Serial.printf("usage: c <0-%d>\n", COLOR_COUNT - 1);
      return;
    }
    uint16_t f8[SPEC_BANDS], clr = 0, nir = 0;
    if (!readSpectrumMedian(f8, &clr, &nir)) {
      Serial.printf("ERR: too dark (Clear=%u < MIN_CLEAR_LEVEL=%d) - no candy, or LED off\n",
                    clr, MIN_CLEAR_LEVEL);
      return;
    }
    normalizeSpectrum(f8, g_specRef[idx]);

    float r, g, b, sat = 0.0f;
    spectrumToRgb(f8, &r, &g, &b);
    const float hue = computeHue(r, g, b, &sat);
    g_hueRef[idx] = hue;
    g_calibrated  = true;

    Serial.printf("CAL %s: hue=%.1f sat=%.2f Clear=%u NIR=%u\n",
                  kColorName[idx], hue, sat, clr, nir);
    Serial.print(F("      spectrum F1..F8:"));
    for (int i = 0; i < SPEC_BANDS; i++) Serial.printf(" %.3f", g_specRef[idx][i]);
    Serial.println();
    Serial.println(F("send 's' to save to flash"));
    beep(1, 50, 3000);

  } else if (c0 == 's' || c0 == 'S') {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putBytes("spec", g_specRef, sizeof(g_specRef));
    prefs.putBytes("hue",  g_hueRef,  sizeof(g_hueRef));
    prefs.putBool("cal", g_calibrated);
    prefs.end();
    Serial.println(F("CAL saved to NVS"));
    beep(2, 50, 3000);

  } else if (c0 == 'r' || c0 == 'R') {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.clear();
    prefs.end();
    g_calibrated = false;
    for (int i = 0; i < COLOR_COUNT; i++) {
      g_hueRef[i] = hueMid(kDefaultHueMin[i], kDefaultHueMax[i]);
      for (int j = 0; j < SPEC_BANDS; j++) g_specRef[i][j] = 0.0f;
    }
    Serial.println(F("CAL cleared, default hue ranges restored (fallback mode)"));
    beep(1, 200, 1200);

  } else if (c0 == 'p' || c0 == 'P') {
    // TaskSorter 在另一个核上写这些计数, 必须在锁内取一份一致的快照
    uint32_t cnt[COLOR_COUNT];
    uint32_t tot = 0, unk = 0;
    if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < COLOR_COUNT; i++) cnt[i] = g_count[i];
      tot = g_total;
      unk = g_unknown;
      xSemaphoreGive(countMutex);
    } else {
      for (int i = 0; i < COLOR_COUNT; i++) cnt[i] = 0;
    }
    Serial.printf("fw=%s state=%d total=%lu unknown=%lu jam=%lu calibrated=%d raw=%d man=%d fault=%d\n",
                  FW_VERSION, (int)g_state, (unsigned long)tot,
                  (unsigned long)unk, (unsigned long)g_jamCount,
                  (int)g_calibrated, (int)g_rawStream, (int)g_manualMode,
                  (int)g_sensorFault);
    for (int i = 0; i < COLOR_COUNT; i++) {
      Serial.printf("  %-6s %lu   (bin %d deg)\n",
                    kColorName[i], (unsigned long)cnt[i], kBinAngle[i]);
    }
    Serial.printf("  rocker now=%d home=%d\n", servoSort.read(),
                  sorterAngle(SORTER_HOME_ANGLE));
    Serial.printf("  chamber WxDxH = %dx%dx%d mm  (candy %dx%d)\n",
                  CHAMBER_W_MM, CHAMBER_D_MM, CHAMBER_H_MM,
                  CANDY_DIAMETER_MM, CANDY_THICKNESS_MM);
    if (g_calibrated) {
      Serial.println(F("  reference spectra (F1..F8):"));
      for (int i = 0; i < COLOR_COUNT; i++) {
        Serial.printf("   %-6s", kColorName[i]);
        for (int j = 0; j < SPEC_BANDS; j++) Serial.printf(" %.3f", g_specRef[i][j]);
        Serial.println();
      }
    }

  } else if (c0 == 'z' || c0 == 'Z') {
    if (xSemaphoreTake(countMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      for (int i = 0; i < COLOR_COUNT; i++) g_count[i] = 0;
      g_unknown = 0;
      g_total   = 0;
      xSemaphoreGive(countMutex);
    }
    Serial.println(F("counters cleared"));

  } else if (c0 == 'u' || c0 == 'U') {
    if (g_manualMode) { Serial.println(F("in manual mode, use 'x' to resume")); return; }
    g_running = !g_running;
    g_state   = g_running ? STATE_IDLE : STATE_PAUSED;
    Serial.printf("sorting %s\n", g_running ? "RESUMED" : "PAUSED");

  } else if (c0 == 't' || c0 == 'T') {
    g_rawStream = !g_rawStream;
    Serial.printf("raw spectrum stream %s\n", g_rawStream ? "ON" : "OFF");

  } else if (c0 == 'm' || c0 == 'M') {
    if (!g_manualMode) enterManual();
    manualDemo();

  } else if (c0 == 'x' || c0 == 'X') {
    if (g_manualMode) exitManual();
    else              Serial.println(F("not in manual mode"));

  } else if (c0 == 'a' || c0 == 'A') {
    if (!g_manualMode) { Serial.println(F("enter manual mode with 'm' first")); return; }
    const int v = cmd.substring(1).toInt();
    servoGateA.write(v ? GATE_OPEN_ANGLE : GATE_CLOSE_ANGLE);
    Serial.printf("gate A -> %s\n", v ? "OPEN" : "CLOSE");

  } else if (c0 == 'b' || c0 == 'B') {
    if (!g_manualMode) { Serial.println(F("enter manual mode with 'm' first")); return; }
    const int v = cmd.substring(1).toInt();
    servoGateB.write(v ? GATE_OPEN_ANGLE : GATE_CLOSE_ANGLE);
    Serial.printf("gate B -> %s\n", v ? "OPEN" : "CLOSE");

  } else if (c0 == 'd' || c0 == 'D') {
    if (!g_manualMode) { Serial.println(F("enter manual mode with 'm' first")); return; }
    int a = cmd.substring(1).toInt();
    if (a < 0)   a = 0;
    if (a > 180) a = 180;
    servoSort.write(sorterAngle(a));
    Serial.printf("rocker -> %d deg\n", sorterAngle(a));

  } else {
    Serial.println(F("unknown cmd, send 'h' for help"));
  }
}

// 按键: 短按 暂停/继续, 长按 3s 计数清零
static void handleButton() {
  static bool     lastRaw   = true;
  static uint32_t pressAt   = 0;
  static bool     longFired = false;

  const bool raw = digitalRead(PIN_BUTTON);   // 按下 = LOW
  if (raw != lastRaw) {
    vTaskDelay(pdMS_TO_TICKS(20));            // 简易消抖
    if (digitalRead(PIN_BUTTON) != raw) return;
    lastRaw = raw;
    if (raw == LOW) { pressAt = millis(); longFired = false; }
    else {
      if (!longFired) {                       // 短按
        if (!g_manualMode) {
          g_running = !g_running;
          g_state   = g_running ? STATE_IDLE : STATE_PAUSED;
          Serial.printf("button: sorting %s\n", g_running ? "RESUMED" : "PAUSED");
          beep(g_running ? 1 : 2, 50, 2600);
        }
      }
    }
  }
  if (raw == LOW && !longFired && (uint32_t)(millis() - pressAt) > T_BUTTON_LONGPRESS) {
    longFired = true;
    if (xSemaphoreTake(countMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      for (int i = 0; i < COLOR_COUNT; i++) g_count[i] = 0;
      g_unknown = 0;
      g_total   = 0;
      xSemaphoreGive(countMutex);
    }
    Serial.println(F("button: counters cleared"));
    beep(3, 60, 3000);
  }
}

// 原始光谱数据流 (调光 / 标定 / 调阈值用)
static void rawStreamTick() {
  static uint32_t last = 0;
  if (!g_rawStream) return;
  if ((uint32_t)(millis() - last) < 300) return;
  last = millis();

  uint16_t f8[SPEC_BANDS], c = 0, n = 0;
  // 给足超时: drawOled 推一整帧 (1024B) 要几十毫秒,
  // 而 readAllChannels() 内部还要等两个 TINT ≈ 70ms
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1500)) != pdTRUE) return;
  const bool ok = readAs7341Once(f8, &c, &n);
  xSemaphoreGive(i2cMutex);
  if (!ok) { Serial.println(F("RAW: sensor read failed")); return; }

  float r, g, b, sat = 0.0f;
  spectrumToRgb(f8, &r, &g, &b);
  const float hue = computeHue(r, g, b, &sat);

  const char* tag;
  if (c < MIN_CLEAR_LEVEL)      tag = "(empty)";     // 检测仓空 -> 用这个值定 MIN_CLEAR_LEVEL
  else if (sat < MIN_SATURATION) tag = "(white/bg)"; // 有反射但没颜色
  else {
    const int cls = g_calibrated ? classifySpectrum(f8, hue, sat, nullptr)
                                 : classifyColor(hue, sat);
    tag = (cls >= 0) ? kColorName[cls] : "UNKNOWN";
  }

  Serial.printf("RAW F1..F8 %5u %5u %5u %5u %5u %5u %5u %5u | CLR %5u NIR %5u | hue=%6.1f sat=%.2f | %s\n",
                f8[0], f8[1], f8[2], f8[3], f8[4], f8[5], f8[6], f8[7],
                c, n, hue, sat, tag);
}

static void TaskUI(void* pv) {
  (void)pv;
  Serial.println(F("[UI] task started"));

  for (;;) {
    drawOled();
    handleSerial();
    handleButton();
    rawStreamTick();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ============================================================
//  初始化
// ============================================================
static void loadCalibration() {
  prefs.begin(NVS_NAMESPACE, true);
  const bool saved = prefs.getBool("cal", false);
  if (saved) {
    // 必须核对读回来的长度: 若 NVS 里是旧版本/半个 blob, 直接把
    // g_calibrated 置真会让参考光谱尾部全是 0.0, 从而静默判错颜色
    const size_t ns = prefs.getBytes("spec", g_specRef, sizeof(g_specRef));
    const size_t nh = prefs.getBytes("hue",  g_hueRef,  sizeof(g_hueRef));
    g_calibrated = (ns == sizeof(g_specRef)) && (nh == sizeof(g_hueRef));
    if (!g_calibrated) {
      Serial.printf("[CAL] stored blob size mismatch (spec %u/%u, hue %u/%u), discarded\n",
                    (unsigned)ns, (unsigned)sizeof(g_specRef),
                    (unsigned)nh, (unsigned)sizeof(g_hueRef));
    }
  }
  prefs.end();

  if (!g_calibrated) {
    for (int i = 0; i < COLOR_COUNT; i++) {
      g_hueRef[i] = hueMid(kDefaultHueMin[i], kDefaultHueMax[i]);
      for (int j = 0; j < SPEC_BANDS; j++) g_specRef[i][j] = 0.0f;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);      // 防止 readStringUntil 在没有换行时把 TaskUI 卡 1 秒
  delay(300);
  Serial.println();
  Serial.printf("=== Candy Sorter v%s booting ===\n", FW_VERSION);
  Serial.printf("chamber %dx%dx%d mm (candy %dx%d mm)\n",
                CHAMBER_W_MM, CHAMBER_D_MM, CHAMBER_H_MM,
                CANDY_DIAMETER_MM, CANDY_THICKNESS_MM);

  // ---- GPIO ----
  pinMode(PIN_IR_SENSOR, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED_ONBOARD, OUTPUT);
  digitalWrite(PIN_LED_ONBOARD, HIGH);        // 点亮, 表示已上电

#if USE_EXTERNAL_LED
  pinMode(PIN_LIGHT_LED, OUTPUT);
  digitalWrite(PIN_LIGHT_LED, HIGH);          // 外接白光 LED 常亮
#endif

#if ENABLE_VIB_UPGRADE
  pinMode(PIN_VIB_MOTOR, OUTPUT);
  digitalWrite(PIN_VIB_MOTOR, LOW);
#endif

  // ---- TCRT5000 自检 ----
  // GPIO34 是"仅输入"脚, 没有内部上拉. 模块没接好时引脚会悬空乱读到 LOW,
  // 固件就会"凭空确认"糖豆到位, 然后放行空拍、照样计数. 上电时检测仓必然是
  // 空的, 所以这里先验一次: 必须读到 HIGH, 否则直接报接线错误.
  {
    bool high = false;
    for (int i = 0; i < 50; i++) {            // 最多等 1 s
      if (digitalRead(PIN_IR_SENSOR) == HIGH) { high = true; break; }
      delay(20);
    }
    if (!high) {
      Serial.println(F("WARN: TCRT5000 DO reads LOW with an empty chamber!"));
      Serial.println(F("      check DO->GPIO34, module VCC/GND, and the on-board pot;"));
      Serial.println(F("      a 10k pull-up from GPIO34 to 3V3 also fixes a floating pin."));
      g_sensorFault = true;
      g_state = STATE_NO_SENSOR;
    } else {
      Serial.println(F("TCRT5000 idle level OK (HIGH)"));
    }
  }

  // ---- 状态灯 ----
  strip.begin();
  strip.setBrightness(60);
  setStrip(-1);

  // ---- I2C ----
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  // ---- 同步原语 (必须在创建任务之前) ----
  i2cMutex      = xSemaphoreCreateMutex();
  countMutex    = xSemaphoreCreateMutex();
  semCandyOnB   = xSemaphoreCreateBinary();
  semNextCandy  = xSemaphoreCreateBinary();
  queueColor    = xQueueCreate(QUEUE_LEN_COLOR, sizeof(int));

  if (!i2cMutex || !countMutex || !semCandyOnB || !semNextCandy || !queueColor) {
    Serial.println(F("FATAL: RTOS object creation failed"));
    while (true) { digitalWrite(PIN_LED_ONBOARD, !digitalRead(PIN_LED_ONBOARD)); delay(120); }
  }
  xSemaphoreGive(semNextCandy);   // 初始放行第一颗

  // ---- 标定数据 ----
  loadCalibration();

  // ---- OLED ----
  u8g2.begin();

  // ---- AS7341 光谱传感器 ----
  if (!as7341.begin()) {
    Serial.println(F("ERROR: AS7341 not found! check wiring (SDA=21 SCL=22, addr 0x39)"));
    g_sensorFault = true;
    g_state = STATE_NO_SENSOR;
    // 仍然继续运行, 便于用 OLED/串口/手动模式排查
  } else {
    as7341.setATIME(AS7341_ATIME);
    as7341.setASTEP(AS7341_ASTEP);
    as7341.setGain(AS7341_GAIN);
    as7341.enableLED(true);                 // 打开模块自带的白光 LED
    as7341.setLEDCurrent(AS7341_LED_MA);
    Serial.printf("AS7341 OK  TINT=%ld ms  LED=%u mA\n",
                  as7341.getTINT(), as7341.getLEDCurrent());
  }

  Serial.printf("calibrated=%d  MIN_CLEAR=%d  MIN_SAT=%.2f  MAX_SPEC_DIST=%.4f\n",
                (int)g_calibrated, MIN_CLEAR_LEVEL, MIN_SATURATION, MAX_SPECTRAL_DIST);
  printHelp();
  beep(1, 80, 2000);

  // ---- 创建任务 (机械固定核1, I2C/显示固定核0) ----
  xTaskCreatePinnedToCore(TaskGate,   "gate",   STACK_GATE,   nullptr,
                          TASK_PRIO_GATE,   nullptr, TASK_GATE_CORE);
  xTaskCreatePinnedToCore(TaskSorter, "sorter", STACK_SORTER, nullptr,
                          TASK_PRIO_SORTER, nullptr, TASK_SORTER_CORE);
  xTaskCreatePinnedToCore(TaskColor,  "color",  STACK_COLOR,  nullptr,
                          TASK_PRIO_COLOR,  nullptr, TASK_COLOR_CORE);
  xTaskCreatePinnedToCore(TaskUI,     "ui",     STACK_UI,     nullptr,
                          TASK_PRIO_UI,     nullptr, TASK_UI_CORE);

  // 只有一切正常时才把状态置为 IDLE, 不要覆盖上面自检报出的 NO_SENSOR
  if (g_state == STATE_BOOT) g_state = STATE_IDLE;
  Serial.println(F("=== all tasks running ==="));
}

// loop() 留空 —— 所有逻辑都在 FreeRTOS 任务里
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
