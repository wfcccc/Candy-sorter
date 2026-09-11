// ============================================================
//  糖豆分拣机 (Rainbow Candy Sorter)
//  主控: ESP32  |  框架: Arduino + FreeRTOS
// ------------------------------------------------------------
//  数据流 (生产者-消费者流水线, 由 FreeRTOS 调度):
//
//   [TaskFeeder]        [TaskColor]         [TaskSorter]        [TaskUI]
//    振动送料      ──▶   读取RGB判色    ──▶   转导流槽+放料   ──▶   OLED/灯带
//    IR检测到位         中值滤波+色相        计数/回位
//        │                    ▲                   │
//        └── semCandyPresent ─┘                   │
//        ◀──────────── semCycleDone ──────────────┘
//
//  同步原语:
//    semCandyPresent  二值信号量: 送料到位 -> 通知判色任务
//    semCycleDone     二值信号量: 本轮分拣完成 -> 允许送下一颗 (节拍控制)
//    queueColor       队列      : 判色结果 -> 分拣任务
//    i2cMutex         互斥量    : TCS34725 与 OLED 共用 I2C 总线
//    countMutex       互斥量    : 保护计数变量
//
//  烧录后串口(115200)命令:
//    c <0-5>  把当前传感器下的糖豆标定为第 n 种颜色
//    s        保存标定到 Flash (NVS)
//    r        清除标定, 恢复默认色相区间
//    p        打印当前状态与计数
//    z        计数清零
// ============================================================

#include <Arduino.h>
#include <Wire.h>

// ---- 第三方库 ----
#include <Adafruit_TCS34725.h>
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
Adafruit_TCS34725 tcs(TCS_INTEGRATION, TCS_GAIN);

Servo servoDiverter;
Servo servoFeeder;
Servo servoRelease;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Adafruit_NeoPixel strip(1, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
Preferences prefs;

// ---- 同步原语 ----
static SemaphoreHandle_t i2cMutex    = nullptr;
static SemaphoreHandle_t countMutex  = nullptr;
static SemaphoreHandle_t semCandyPresent = nullptr;
static SemaphoreHandle_t semCycleDone    = nullptr;
static QueueHandle_t     queueColor      = nullptr;

// ---- 共享状态 ----
typedef enum {
  STATE_BOOT = 0,
  STATE_IDLE,
  STATE_FEEDING,
  STATE_SENSING,
  STATE_SORTING,
  STATE_JAM,
  STATE_NO_SENSOR
} SystemState;

static volatile SystemState g_state = STATE_BOOT;

static uint32_t g_count[COLOR_COUNT] = {0};  // 各颜色计数 (受 countMutex 保护)
static uint32_t g_unknown   = 0;             // 未识别计数
static uint32_t g_total     = 0;             // 总计数
static volatile uint32_t g_jamCount = 0;     // 卡料次数

// ---- 标定数据 ----
static float g_hueRef[COLOR_COUNT];          // 每种颜色的标定色相中心
static bool  g_calibrated = false;           // 是否已标定

// ---- 最近一次判色结果 (给 UI 显示) ----
static volatile int  g_lastColor = -1;
static volatile float g_lastHue  = -1.0f;
static volatile float g_lastSat  =  0.0f;

// ---- 前向声明 ----
static void TaskFeeder(void* pv);
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

// 色相环上的最短角距离 (0-180)
static inline float hueDistance(float a, float b) {
  float d = fabsf(a - b);
  return (d > 180.0f) ? (360.0f - d) : d;
}

// 由 RGB 计算 HSV 色相与饱和度
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

// 判色: 返回 0..COLOR_COUNT-1, 或 -1 表示未知
static int classifyColor(float hue, float sat) {
  if (sat < MIN_SATURATION) return -1;

  if (g_calibrated) {
    // 已标定: 取色相最接近的参考色
    int   best  = -1;
    float bestD = 1e9f;
    for (int i = 0; i < COLOR_COUNT; i++) {
      const float d = hueDistance(hue, g_hueRef[i]);
      if (d < bestD) { bestD = d; best = i; }
    }
    return (bestD <= 55.0f) ? best : -1;
  }

  // 未标定: 用默认色相区间
  for (int i = 0; i < COLOR_COUNT; i++) {
    const float lo = kDefaultHueMin[i];
    const float hi = kDefaultHueMax[i];
    if (lo < hi) { if (hue >= lo && hue < hi) return i; }
    else         { if (hue >= lo || hue < hi) return i; }  // 红色跨 0 度
  }
  return -1;
}

// 取中位数 (原地排序)
static uint16_t medianU16(uint16_t* a, int n) {
  for (int i = 1; i < n; i++) {
    const uint16_t key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
  return a[n / 2];
}

// 采多次取中位数, 抑制反光毛刺; 返回 true 表示亮度足够(有糖豆)
static bool readColorMedian(uint16_t* outR, uint16_t* outG, uint16_t* outB, uint16_t* outC) {
  uint16_t rs[COLOR_SAMPLE_TIMES], gs[COLOR_SAMPLE_TIMES];
  uint16_t bs[COLOR_SAMPLE_TIMES], cs[COLOR_SAMPLE_TIMES];

  for (int i = 0; i < COLOR_SAMPLE_TIMES; i++) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(300)) != pdTRUE) return false;
    tcs.getRawData(&rs[i], &gs[i], &bs[i], &cs[i]);
    xSemaphoreGive(i2cMutex);
    vTaskDelay(pdMS_TO_TICKS(8));
  }

  *outR = medianU16(rs, COLOR_SAMPLE_TIMES);
  *outG = medianU16(gs, COLOR_SAMPLE_TIMES);
  *outB = medianU16(bs, COLOR_SAMPLE_TIMES);
  *outC = medianU16(cs, COLOR_SAMPLE_TIMES);

  return (*outC >= MIN_CLEAR_LEVEL);
}

// 状态灯颜色
static void setStrip(int color) {
  uint8_t r = 40, g = 40, b = 40;
  switch (color) {
    case 0: r = 255; g =   0; b =   0; break;  // RED
    case 1: r = 255; g =  80; b =   0; break;  // ORANGE
    case 2: r = 255; g = 200; b =   0; break;  // YELLOW
    case 3: r =   0; g = 200; b =   0; break;  // GREEN
    case 4: r =   0; g =  80; b = 255; break;  // BLUE
    case 5: r = 160; g =   0; b = 255; break;  // PURPLE
    default: break;                            // UNKNOWN -> 灰
  }
  strip.setPixelColor(0, strip.Color(r, g, b));
  strip.show();
}

// 读取并分类一次 (供判色任务与标定功能复用)
static int sampleAndClassify(float* hueOut, float* satOut) {
  uint16_t r, g, b, c;
  if (!readColorMedian(&r, &g, &b, &c)) {
    if (hueOut) *hueOut = -1.0f;
    if (satOut) *satOut =  0.0f;
    return -1;
  }
  const float hue = computeHue((float)r, (float)g, (float)b, satOut);
  if (hueOut) *hueOut = hue;
  return classifyColor(hue, satOut ? *satOut : 0.0f);
}

// ============================================================
//  Task 1: 送料 (振动 + 防架桥搅拌 + 到位检测)
// ============================================================
static void TaskFeeder(void* pv) {
  (void)pv;

  pinMode(PIN_VIB_MOTOR, OUTPUT);
  digitalWrite(PIN_VIB_MOTOR, LOW);

  servoFeeder.attach(PIN_SERVO_FEEDER, 500, 2400);
  servoFeeder.write(FEEDER_IDLE_ANGLE);

  servoRelease.attach(PIN_SERVO_RELEASE, 500, 2400);
  servoRelease.write(RELEASE_CLOSE_ANGLE);

  Serial.println(F("[Feeder] task started"));

  for (;;) {
    // ---- 等机构空闲 (上一颗已分拣完成) ----
    if (xSemaphoreTake(semCycleDone, pdMS_TO_TICKS(15000)) != pdTRUE) {
      // 上一颗卡住了, 报警并继续
      g_state = STATE_JAM;
      g_jamCount++;
      beep(3);
      vTaskDelay(pdMS_TO_TICKS(1500));
      continue;
    }

    g_state = STATE_FEEDING;

    // ---- 搅拌桨周期性摆动, 防止糖豆在料斗架桥 ----
    servoFeeder.write(FEEDER_STEP_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(280));
    servoFeeder.write(FEEDER_IDLE_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(120));

    // ---- 振动送料, 直到红外检测到糖豆到位 ----
    bool found = false;
    const uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < T_VIB_TIMEOUT) {
      if (digitalRead(PIN_IR_SENSOR) == LOW) { found = true; break; }
      digitalWrite(PIN_VIB_MOTOR, HIGH);
      vTaskDelay(pdMS_TO_TICKS(T_VIB_ON));
      digitalWrite(PIN_VIB_MOTOR, LOW);
      vTaskDelay(pdMS_TO_TICKS(T_VIB_OFF));
    }
    digitalWrite(PIN_VIB_MOTOR, LOW);

    if (!found) {
      // ---- 送料超时: 可能卡料 ----
      g_state = STATE_JAM;
      g_jamCount++;
      Serial.println(F("[Feeder] TIMEOUT: no candy detected (jam?)"));
      beep(3);
      vTaskDelay(pdMS_TO_TICKS(2000));
      // 归还节拍信号量, 允许重试
      xSemaphoreGive(semCycleDone);
      continue;
    }

    // ---- 等糖豆在检测位停稳 ----
    vTaskDelay(pdMS_TO_TICKS(T_SETTLE_AFTER_FEED));

    if (digitalRead(PIN_IR_SENSOR) != LOW) {
      Serial.println(F("[Feeder] candy slipped away, retry"));
      xSemaphoreGive(semCycleDone);
      vTaskDelay(pdMS_TO_TICKS(300));
      continue;
    }

    // ---- 通知判色任务 ----
    g_state = STATE_SENSING;
    xSemaphoreGive(semCandyPresent);
    // 注意: 此处不等待, 循环顶部再去 take(semCycleDone)
  }
}

// ============================================================
//  Task 2: 判色 (采样 -> 中值滤波 -> 分类 -> 入队)
// ============================================================
static void TaskColor(void* pv) {
  (void)pv;
  Serial.println(F("[Color] task started"));

  for (;;) {
    if (xSemaphoreTake(semCandyPresent, pdMS_TO_TICKS(20000)) != pdTRUE) {
      continue;  // 长时间没糖豆, 继续等
    }

    // 静置, 等振动完全停止再采光
    vTaskDelay(pdMS_TO_TICKS(T_COLOR_SETTLE));

    float hue = -1.0f, sat = 0.0f;
    const int color = sampleAndClassify(&hue, &sat);

    g_lastColor = color;
    g_lastHue   = hue;
    g_lastSat   = sat;

    Serial.printf("[Color] hue=%.1f sat=%.2f -> %s\n",
                  hue, sat, (color >= 0) ? kColorName[color] : "UNKNOWN");

    // 无论识别成功与否都要入队, 否则糖豆会永远堵在检测位
    (void)xQueueSend(queueColor, &color, pdMS_TO_TICKS(1000));
  }
}

// ============================================================
//  Task 3: 分拣 (转导流槽 -> 开挡板放料 -> 回位 -> 计数)
//    优先级最高, 保证动作时序不被其他任务打断
// ============================================================
static void TaskSorter(void* pv) {
  (void)pv;

  // 导流槽上电回中位
  servoDiverter.attach(PIN_SERVO_DIVERTER, 500, 2400);
  servoDiverter.write(DIVERTER_HOME_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(500));

  Serial.println(F("[Sorter] task started"));

  for (;;) {
    int color = -1;
    if (xQueueReceive(queueColor, &color, pdMS_TO_TICKS(20000)) != pdTRUE) {
      continue;
    }

    g_state = STATE_SORTING;

    // ---- 1. 导流槽转到目标容器 (未知颜色走中位/废料位) ----
    const int angle = (color >= 0) ? kBinAngle[color] : DIVERTER_HOME_ANGLE;
    servoDiverter.write(angle);
    vTaskDelay(pdMS_TO_TICKS(T_DIVERTER_MOVE));

    // ---- 2. 打开释放挡板, 糖豆落料 ----
    setStrip(color);
    servoRelease.write(RELEASE_OPEN_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(T_RELEASE_HOLD));

    // ---- 3. 关闭挡板 ----
    servoRelease.write(RELEASE_CLOSE_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(T_RELEASE_RECOVER));

    // ---- 4. 导流槽回中位 ----
    servoDiverter.write(DIVERTER_HOME_ANGLE);

    // ---- 5. 计数 (加锁保护) ----
    if (xSemaphoreTake(countMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (color >= 0) g_count[color]++;
      else            g_unknown++;
      g_total++;
      xSemaphoreGive(countMutex);
    }

    g_state = STATE_IDLE;

    // ---- 6. 放行下一颗送料 ----
    xSemaphoreGive(semCycleDone);
  }
}

// ============================================================
//  Task 4: 人机界面 (OLED + 状态灯 + 串口命令)
//    优先级最低, 抢占随时可被打断
// ============================================================
static void drawOled() {
  char line[24];

  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // 标题行
  u8g2.drawStr(0, 9, "CANDY SORTER");
  const char* st = "BOOT";
  switch (g_state) {
    case STATE_IDLE:    st = "IDLE";     break;
    case STATE_FEEDING: st = "FEEDING";  break;
    case STATE_SENSING: st = "SENSING";  break;
    case STATE_SORTING: st = "SORTING";  break;
    case STATE_JAM:     st = "JAM!";     break;
    case STATE_NO_SENSOR: st = "NO SENSOR"; break;
    default: break;
  }
  snprintf(line, sizeof(line), "%s", st);
  u8g2.drawStr(92, 9, line);

  u8g2.drawHLine(0, 12, 128);

  // 各颜色计数 (两列三行)
  for (int i = 0; i < COLOR_COUNT; i++) {
    const int col = i / 3;
    const int row = i % 3;
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

  // 底部: 总数 / 未知 / 最近色相
  if (xSemaphoreTake(countMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    const uint32_t tot = g_total, unk = g_unknown;
    xSemaphoreGive(countMutex);
    snprintf(line, sizeof(line), "TOT%lu UNK%lu J%lu", (unsigned long)tot,
             (unsigned long)unk, (unsigned long)g_jamCount);
  } else {
    snprintf(line, sizeof(line), "---");
  }
  u8g2.drawStr(0, 62, line);

  u8g2.sendBuffer();
  xSemaphoreGive(i2cMutex);
}

// 处理串口标定命令
static void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  const char c0 = cmd.charAt(0);

  if (c0 == 'c' || c0 == 'C') {
    // 标定: c <0-5>
    const int idx = cmd.substring(1).toInt();
    if (idx < 0 || idx >= COLOR_COUNT) {
      Serial.println(F("usage: c <0-5>"));
      return;
    }
    float hue = -1.0f, sat = 0.0f;
    uint16_t r, g, b, cc;
    if (!readColorMedian(&r, &g, &b, &cc)) {
      Serial.println(F("ERR: no candy / too dark"));
      return;
    }
    hue = computeHue((float)r, (float)g, (float)b, &sat);
    g_hueRef[idx]  = hue;
    g_calibrated   = true;
    Serial.printf("CAL %s: hue=%.1f sat=%.2f (R%u G%u B%u C%u)\n",
                  kColorName[idx], hue, sat, r, g, b, cc);
    Serial.println(F("send 's' to save to flash"));
    beep(1, 50, 3000);

  } else if (c0 == 's' || c0 == 'S') {
    prefs.begin(NVS_NAMESPACE, false);
    prefs.putBytes("hue", g_hueRef, sizeof(g_hueRef));
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
      g_hueRef[i] = (kDefaultHueMin[i] + kDefaultHueMax[i]) * 0.5f;
    }
    Serial.println(F("CAL cleared, default ranges restored"));
    beep(1, 200, 1200);

  } else if (c0 == 'p' || c0 == 'P') {
    Serial.printf("state=%d total=%lu unknown=%lu jam=%lu calibrated=%d\n",
                  (int)g_state, (unsigned long)g_total,
                  (unsigned long)g_unknown, (unsigned long)g_jamCount,
                  (int)g_calibrated);
    for (int i = 0; i < COLOR_COUNT; i++) {
      Serial.printf("  %-6s %lu\n", kColorName[i], (unsigned long)g_count[i]);
    }

  } else if (c0 == 'z' || c0 == 'Z') {
    if (xSemaphoreTake(countMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      for (int i = 0; i < COLOR_COUNT; i++) g_count[i] = 0;
      g_unknown = 0;
      g_total = 0;
      xSemaphoreGive(countMutex);
    }
    Serial.println(F("counters cleared"));
  }
}

static void TaskUI(void* pv) {
  (void)pv;
  Serial.println(F("[UI] task started"));

  for (;;) {
    drawOled();
    handleSerial();
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// ============================================================
//  初始化
// ============================================================
static void loadCalibration() {
  prefs.begin(NVS_NAMESPACE, true);
  const bool saved = prefs.getBool("cal", false);
  if (saved) {
    prefs.getBytes("hue", g_hueRef, sizeof(g_hueRef));
    g_calibrated = true;
  }
  prefs.end();

  if (!g_calibrated) {
    for (int i = 0; i < COLOR_COUNT; i++) {
      g_hueRef[i] = (kDefaultHueMin[i] + kDefaultHueMax[i]) * 0.5f;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== Candy Sorter booting ==="));

  // ---- GPIO ----
  pinMode(PIN_IR_SENSOR, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED_ONBOARD, OUTPUT);
  digitalWrite(PIN_LED_ONBOARD, HIGH);   // 点亮, 表示已上电

  // ---- 状态灯 ----
  strip.begin();
  strip.setBrightness(60);
  setStrip(-1);

  // ---- I2C ----
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);

  // ---- 同步原语 (必须在创建任务之前) ----
  i2cMutex         = xSemaphoreCreateMutex();
  countMutex       = xSemaphoreCreateMutex();
  semCandyPresent  = xSemaphoreCreateBinary();
  semCycleDone     = xSemaphoreCreateBinary();
  queueColor       = xQueueCreate(QUEUE_LEN_COLOR, sizeof(int));

  if (!i2cMutex || !countMutex || !semCandyPresent || !semCycleDone || !queueColor) {
    Serial.println(F("FATAL: RTOS object creation failed"));
    while (true) { digitalWrite(PIN_LED_ONBOARD, !digitalRead(PIN_LED_ONBOARD)); delay(120); }
  }
  // 初始放行第一颗送料
  xSemaphoreGive(semCycleDone);

  // ---- 标定数据 ----
  loadCalibration();

  // ---- OLED ----
  u8g2.begin();

  // ---- 颜色传感器 ----
  if (!tcs.begin()) {
    Serial.println(F("ERROR: TCS34725 not found! check wiring (SDA=21 SCL=22)"));
    g_state = STATE_NO_SENSOR;
    // 仍然继续运行, 便于用 OLED/串口排查
  } else {
    tcs.setInterrupt(false);            // LED 常亮
    tcs.setIntegrationTime(TCS_INTEGRATION);
    tcs.setGain(TCS_GAIN);
    Serial.println(F("TCS34725 OK"));
  }

  Serial.printf("calibrated=%d\n", (int)g_calibrated);
  Serial.println(F("cmds: c<n> calibrate, s save, r reset, p print, z zero"));
  beep(1, 80, 2000);

  // ---- 创建任务 (固定到不同核心, 避免互相抢占) ----
  xTaskCreatePinnedToCore(TaskSorter, "sorter", STACK_SORTER, nullptr,
                          TASK_PRIO_SORTER, nullptr, TASK_SORTER_CORE);
  xTaskCreatePinnedToCore(TaskColor,  "color",  STACK_COLOR,  nullptr,
                          TASK_PRIO_COLOR,  nullptr, TASK_COLOR_CORE);
  xTaskCreatePinnedToCore(TaskFeeder, "feeder", STACK_FEEDER, nullptr,
                          TASK_PRIO_FEEDER, nullptr, TASK_FEEDER_CORE);
  xTaskCreatePinnedToCore(TaskUI,     "ui",     STACK_UI,     nullptr,
                          TASK_PRIO_UI,     nullptr, TASK_UI_CORE);

  g_state = STATE_IDLE;
  Serial.println(F("=== all tasks running ==="));
}

// loop() 留空 —— 所有逻辑都在 FreeRTOS 任务里
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
