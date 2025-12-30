/* 2025.10.09
*   k1tada@largefield.net
*/
#include <M5Unified.h>
#include <driver/twai.h>

#define VERSION "Ver. 1.1.2"

#define CAN_TX_PIN  gpio_num_t(32) // Core2 Port A
#define CAN_RX_PIN  gpio_num_t(33) // Core2 Port A
// #define CAN_TX_PIN  gpio_num_t(26) // Core2 Port B
// #define CAN_RX_PIN  gpio_num_t(36) // Core2 Port B

//* Setting */
#define SPEED_ADJ   1.035f // 車速の調整値
#define FINAL_GEAR  5.545f // ファイナルギア比
#define TIRE_LENGTH 1.795f // タイヤ外周長

#define PEEK_LENGTH 80 // PeekHold Times

static const uint8_t PIDs[][2] = {
    /* {pID, Timing(X10 mSec)} */
    {0x05, 80}, // 冷却水温(摂氏) 1{x1 -40}
    {0x03, 30}, // 1:始動直後 2:定常動作 4:補正制御 8:異常 16:部分異常
    // {0x06, 10}, // 短期燃料調整量(%)バンク１ 1{x0.78125 -100}
    // {0x07, 10}, // 長期燃料調整量(%)バンク１ 1{x0.78125 -100}
    {0x0B, 10}, // 吸気圧力(kPa) 1{x1 +0}
    {0x0C,  8}, // エンジン回転数(rpm) 2{x0.25 +0}
    {0x0D,  8}, // 車速(Km/h) 1{x1 +0}
    // {0x0E, 10}, // 点火進角調整値(deg) 1{x0.5 -64}
    // {0x0F, 10}, // 吸気温度(摂氏) 1{x1 -40}
    {0x10, 10}, // エアフロー量(grams/sec) 2{x0.01 +0}
    // {0x1F, 10}, // エンジン始動からの時間(秒) 2{x1 +0}
    {0x33, 80}, // 大気圧(kPa) 1{x1 +0}
    {0x42, 80}, // コントローラ供給電圧(V) 2{x0.001 +0}
};

//* RX-Data Queue */
#define QUEUE_LENGTH 16
static QueueHandle_t xQueue;

//* M5GFX */
static M5GFX lcd;
static M5Canvas canvas(&lcd);  // オフスクリーン描画用
static M5Canvas base(&canvas); // ベースフォーム

//* CAN (twai) */
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
// static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_filter_config_t f_config = {
    .acceptance_code = (uint32_t)(0x7E8 << 21), // ODB2 retune ID
    .acceptance_mask = ~(0x7FF << 21),
    .single_filter = true
};

//* Global */
static bool _display_ = false;
int peek_rpm = 0;
int peek_count = PEEK_LENGTH;

static void base_drawing() {
    // 背景画面スプライト作成
    int h;
    uint16_t color;
    // タコメータ背景
    base.fillRect(0, 30, 270, 4, TFT_GREEN);
    base.fillRect(270, 30, 50, 4, TFT_RED);
    color = TFT_GREEN;
    for (int i = 0; i <= 320; i = i + 18) {
        if (i >= 270) color = TFT_RED; // レッドゾーン7500rpm
        if (i % 36 == 0) {
            h = 30; // 目盛1000
        } else {
            h = 15; // 目盛500
        }
        base.drawFastVLine(i, 30, h, color); // 目盛線描画
    }
    // base.drawFastHLine(0, 130, 10, TFT_YELLOW);
    base.setFont(&fonts::FreeMono12pt7b);
    base.setTextSize(1);
    base.setCursor(25, 220);
    base.print("kPa");
    base.setCursor(255, 160);
    base.print("km/h");
    base.setCursor(260, 80);
    base.printf("rpm");
    // ギア
    base.fillRoundRect(85, 70, 45, 45, 8, TFT_WHITE);
}

static void OBD_display(uint16_t *value) {
    uint16_t color;
    // 背景データをコピー
    base.pushSprite(0, 0);
    // データー描画
    // 稼働時間
    value[0x1F] = (uint16_t)(millis() / 1000);
    canvas.setFont(&fonts::lgfxJapanGothic_24);
    canvas.setTextSize(1);
    canvas.setCursor(220, 0);
    canvas.printf("%02d:%02d:%02d", (value[0x1F] / 3600), ((value[0x1F] % 3600) / 60), ((value[0x1F] % 3600) % 60));
    // 燃調ステータス
    canvas.setFont(&fonts::lgfxJapanGothic_24);
    canvas.setTextSize(1);
    canvas.setCursor(0, 0);
    switch (value[0x03]) {
    case 0x100:
        canvas.setTextColor(TFT_BLACK, TFT_CYAN);
        canvas.print(" Open-loop ");
        canvas.fillRect(90, 205, 230, 2, TFT_BLUE);
        break;
    case 0x200:
        canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        canvas.print(" Feedback ");
        canvas.fillRect(90, 205, 230, 2, TFT_GREEN);
        break;
    case 0x400:
        canvas.setTextColor(TFT_BLACK, TFT_YELLOW);
        canvas.print(" Fuel Cut ");
        canvas.fillRect(90, 205, 230, 2, TFT_YELLOW);
        break;
    case 0x800:
        canvas.setTextColor(TFT_WHITE, TFT_RED);
        canvas.print("-CL-System Fault ");
        canvas.fillRect(90, 205, 230, 2, TFT_RED);
        break;
    case 0x1000:
        canvas.setTextColor(TFT_WHITE, TFT_RED);
        canvas.print("-OL-System Fault ");
        canvas.fillRect(90, 205, 230, 2, TFT_RED);
        break;
    default:
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        canvas.printf(" Code : %04X ", value[0x03]);
        canvas.fillRect(90, 205, 230, 2, TFT_WHITE);
        break;
    }
    // 水温・電圧・吸気圧
    canvas.setFont(&fonts::lgfxJapanGothic_28);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    if (value[0x05] > 144) {
        canvas.setTextColor(TFT_WHITE, TFT_RED);
    }
    canvas.setCursor(100, 215);
    canvas.printf("%3d℃", (value[0x05] - 40));
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);

    if (value[0x42] > 13200) {
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    canvas.setCursor(220, 215);
    canvas.printf("%5.2fＶ", (value[0x42] / 1000.0f));

    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    int inmaniPress = value[0x0B] - value[0x33];
    canvas.setCursor(10, 190);
    canvas.printf("%4d", inmaniPress);

    // 吸気圧メーター
    if (inmaniPress > 0) {
        canvas.drawRect(25, 80, 40, 100, TFT_YELLOW);
        canvas.fillRect(25, 180, 40, (int)(inmaniPress * -1.0f), TFT_YELLOW);
        canvas.fillTriangle(65, 80, 35, 180, 65, 180, TFT_BLACK);
    } else {
        canvas.drawRect(25, 80, 40, 100, TFT_CYAN);
        canvas.fillRect(25, 80, 40, (int)(inmaniPress * -1.3f), TFT_CYAN);
        canvas.fillTriangle(35, 80, 65, 80, 65, 180, TFT_BLACK);
    }
    // MAF・燃調トリム
    // canvas.drawFastVLine(2, 130, (value[0x06] - 128) * -1, TFT_YELLOW);
    // canvas.drawFastVLine(8, 130, (value[0x07] - 128) * -1, TFT_YELLOW);
    canvas.fillRect(10, 180, 5, (int)(value[0x10] * -0.02f), TFT_GREEN);
    // 車速
    int speed = round(value[0x0D] * SPEED_ADJ);
    canvas.setFont(&fonts::Font7);
    canvas.setTextSize(1.7, 1.5);
    canvas.setTextDatum(textdatum_t::bottom_right);
    canvas.drawNumber(speed, 250, 195);
    canvas.setTextDatum(textdatum_t::top_left);
    // 回転数
    int rpm = value[0x0C] / 4;
    canvas.setFont(&fonts::lgfxJapanGothic_40);
    canvas.setTextSize(1);
    canvas.setCursor(180, 70);
    canvas.printf("%4d", rpm);
    // ピークグラフ
    --peek_count;
    if (peek_count < 0 || peek_rpm < rpm) {
        peek_rpm = rpm;
        peek_count = PEEK_LENGTH;
    }
    int graph_peek_rpm = (int)(peek_rpm / 50 * 1.8f);
    canvas.fillRect(0, 36, graph_peek_rpm, 24, TFT_DARKGREEN);
    canvas.fillRect(graph_peek_rpm, 36, -1, 24, TFT_YELLOW);
    // タコメーターグラフ
    color = TFT_GREEN;
    if (rpm >= 7500) color = TFT_RED;
    canvas.fillRect(0, 36, (int)(rpm / 50 * 1.8f), 24, color);
    canvas.setFont(&fonts::FreeSansOblique12pt7b);
    canvas.setTextColor(TFT_DARKGRAY);
    // canvas.setTextSize(0.4f);
    for (int i = 1; i <= 6; ++i) {
        canvas.drawString(String(i), i * 36 + 1, 40); // 目盛
    }
    // ピーク回転数
    canvas.setFont(&fonts::lgfxJapanGothic_32);
    canvas.setTextColor(TFT_YELLOW);
    canvas.setTextSize(1);
    canvas.setCursor(253, 35);
    canvas.printf("%4d", peek_rpm);

    // ギア
    float gear_ratio = rpm / ((speed * 1000 / 60) * (FINAL_GEAR / TIRE_LENGTH));
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextColor(TFT_BLACK);
    canvas.setTextSize(1.2f);
    if (gear_ratio < 0.5) {
        canvas.drawCentreString("-", 107, 70);
    } else if (gear_ratio < 0.82f) {
        canvas.drawCentreString("5", 107, 70);
    } else if (gear_ratio < 1.1f) {
        canvas.drawCentreString("4", 107, 70);
    } else if (gear_ratio < 1.5f) {
        canvas.drawCentreString("3", 107, 70);
    } else if (gear_ratio < 2.4f) {
        canvas.drawCentreString("2", 107, 70);
    } else if (gear_ratio < 3.5f) {
        canvas.drawCentreString("1", 107, 70);
    } else {
        canvas.drawCentreString("-", 107, 70);
    }
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    // 描画画面表示
    canvas.pushSprite(0, 0);
}

void ODB_DISP_Loop(void *arg) {
    static uint16_t value[0x60] = {};
    uint8_t queue_data[8];
    while(1) {
        // queueからデータを取り出し、valueに保存
        while(xQueueReceive(xQueue, &queue_data, 0)) {
            if (queue_data[0] == 3) {
                value[queue_data[2]] = queue_data[3];
            } else {
                value[queue_data[2]] = queue_data[3] * 256 + queue_data[4];
            }
        }
        if (_display_) OBD_display(value);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

static void OBD_Query_Send(uint8_t Query_ID) {
    // Query message data make
    twai_message_t tx_message = {
        .identifier       = 0x7DF, // OBD2 Query message
        .data_length_code = 8,
        .data             = { 2, 1, Query_ID, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA }
    };
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        // Serial.printf(">> PID:0x%02X, Message queued for transmission\n", Query_ID);
    } else {
        // Serial.printf(">> PID:0x%02X, Failed to queue message for transmission\n", Query_ID);
    }
}

void CAN_TX_Loop(void *arg) {
    unsigned long previousMillis[sizeof(PIDs) / sizeof(PIDs[0])] = {};
    unsigned long currentMillis;
    while (1) {
        currentMillis = millis();
        for (int i = 0; i < (sizeof(PIDs) / sizeof(PIDs[0])); i++ ) {
            if (currentMillis - previousMillis[i] >= PIDs[i][1] * 10) {
                previousMillis[i] = currentMillis;
                // Serial.printf("Send: 0x%02X\n", PIDs[i][0]);
                OBD_Query_Send(PIDs[i][0]);
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }
    vTaskDelete(NULL);
}

void CAN_RX_Loop(void *arg) {
    twai_message_t rx_message;
    uint16_t p_val;
    char rx_data[24];
    while (1) {
        while (twai_receive(&rx_message, portMAX_DELAY) == ESP_OK) {
            xQueueSend(xQueue, &rx_message.data, 0);
            // sprintf(rx_data, "%02X %02X %02X %02X %02X %02X %02X %02X",
            //     rx_message.data[0], rx_message.data[1], rx_message.data[2], rx_message.data[3],
            //     rx_message.data[4], rx_message.data[5], rx_message.data[6], rx_message.data[7]);
            // Serial.printf("> received 0x%03X [%d] %s\r\n", rx_message.identifier, rx_message.data_length_code, rx_data);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}

void UI_Loop(void *arg) {
    uint8_t lcdBrightness = 192;
    lcd.setBrightness(lcdBrightness);
    while(1) {
        M5.update();

        // 外部電源喪失で電源断
        if (M5.Power.getBatteryCurrent() < -30) {
            _display_ = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            lcd.clear();
            lcd.setFont(&fonts::lgfxJapanGothic_40);
            lcd.drawCentreString("Power Off", 160, 100);
            vTaskDelay(pdMS_TO_TICKS(3000));
            M5.Power.powerOff();
        }

        // Bボタンで画面の明るさを切り替える
        if (M5.BtnB.wasPressed()) {
            lcdBrightness = lcdBrightness + 128;
            lcd.setBrightness(lcdBrightness);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

    }
    vTaskDelete(NULL);
}

static void splash() {
    lcd.clear();
    lcd.setFont(&fonts::lgfxJapanGothic_40);
    lcd.drawCentreString("ＯＢＤⅡ", 160,  40);
    lcd.drawCentreString("Monitor", 160,  80);
    lcd.setFont(&fonts::lgfxJapanGothicP_20);
    lcd.drawCentreString(VERSION, 160, 120);
    lcd.drawCentreString("development by", 160, 170);
    lcd.drawCentreString("k1tada@largefield.net", 160, 200);
}

void setup() {
    auto cfg = M5.config();
    cfg.internal_imu = false;
    cfg.internal_mic = false;
    cfg.internal_spk = false;
    cfg.internal_rtc = false;
    cfg.output_power = false;
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);

    M5.Power.setLed(0);  // PowerLED off

    // Queue
    xQueue = xQueueCreate(QUEUE_LENGTH, 8);

    // LCD Display
    lcd.init();
    lcd.setRotation(1);
    lcd.setColor(8);
    canvas.setColorDepth(8);
    canvas.createSprite(lcd.width(), lcd.height());
    base.setColorDepth(8);
    base.createSprite(lcd.width(), lcd.height());

    // 起動スプラッシュ
    splash();

    // 背景作成
    base_drawing();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    Serial.println("Driver installed");
    ESP_ERROR_CHECK(twai_start());

    // CAN TX/RX 開始
    xTaskCreatePinnedToCore(CAN_RX_Loop, "RX_Loop", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(CAN_TX_Loop, "TX_Loop", 4096, NULL, 5, NULL, 1);

    delay(3000);
    _display_ = true;

    // UI/Display 開始
    xTaskCreatePinnedToCore(ODB_DISP_Loop, "DISP_Loop", 4096, NULL, 8, NULL, 0);
    xTaskCreatePinnedToCore(UI_Loop, "UI_Loop", 2048, NULL, 5, NULL, 0);
}

void loop() {
    delay(1000);
}
