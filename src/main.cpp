#include <FFat.h>
#include <lvgl.h>
#include <tuyacpp/scanner.hpp>
#include <USB.h>
#include <USBMSC.h>
#include <WiFi.h>

#include "pin_config.h"

#define BLOCK_SIZE 4096

USBCDC cdc;
USBMSC msc;
EspClass esp;
const esp_partition_t* partition;

#ifdef BOARD_WAVESHARE_1_69
#include <Arduino_GFX_Library.h>
#include <CST816S.h>

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

CST816S touch(IIC_SDA, IIC_SCL, TP_RST, TP_INT);
#endif

uint32_t screenWidth;
uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;


#define LVGL_PORT_TASK_MAX_DELAY_MS             (500)       // The maximum delay of the LVGL timer task, in milliseconds
#define LVGL_PORT_TASK_MIN_DELAY_MS             (2)         // The minimum delay of the LVGL timer task, in milliseconds
#define LVGL_PORT_TASK_STACK_SIZE               (6 * 1024)  // The stack size of the LVGL timer task, in bytes
#define LVGL_PORT_TASK_PRIORITY                 (2)         // The priority of the LVGL timer task

static SemaphoreHandle_t lvgl_mux = nullptr;
static TaskHandle_t lvgl_task_handle = nullptr;

#ifdef BOARD_WAVESHARE_4_3
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <SD.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

#define LVGL_PORT_BUFFER_MALLOC_CAPS            (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)       // Allocate LVGL buffer in SRAM
#define LVGL_PORT_BUFFER_NUM                    (2)

#define LVGL_PORT_DISP_BUFFER_NUM           (2)

static Board board;
static void *lvgl_buf[LVGL_PORT_BUFFER_NUM] = {};

IRAM_ATTR bool onLcdVsyncCallback(void *user_data)
{
    BaseType_t need_yield = pdFALSE;
    TaskHandle_t task_handle = (TaskHandle_t)user_data;
    // Notify that the current LCD frame buffer has been transmitted
    xTaskNotifyFromISR(task_handle, ULONG_MAX, eNoAction, &need_yield);
    return (need_yield == pdTRUE);
}

static void rounder_callback(lv_disp_drv_t *drv, lv_area_t *area)
{
    LCD *lcd = board.getLCD();
    uint8_t x_align = lcd->getBasicAttributes().basic_bus_spec.x_coord_align;
    uint8_t y_align = lcd->getBasicAttributes().basic_bus_spec.y_coord_align;

    if (x_align > 1) {
        // round the start of coordinate down to the nearest aligned value
        area->x1 &= ~(x_align - 1);
        // round the end of coordinate up to the nearest aligned value
        area->x2 = (area->x2 & ~(x_align - 1)) + x_align - 1;
    }

    if (y_align > 1) {
        // round the start of coordinate down to the nearest aligned value
        area->y1 &= ~(y_align - 1);
        // round the end of coordinate up to the nearest aligned value
        area->y2 = (area->y2 & ~(y_align - 1)) + y_align - 1;
    }
}
#endif // BOARD_WAVESHARE_4_3

static bool lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE);
}

static void lvgl_port_task(void *arg)
{
    uint32_t task_delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
    while (1) {
        if (lvgl_port_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            xSemaphoreGiveRecursive(lvgl_mux);
        }
        if (task_delay_ms > LVGL_PORT_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_PORT_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_PORT_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

tuya::Loop l;
std::unique_ptr<tuya::Scanner> s;

static lv_obj_t *battery_label;
static lv_obj_t *wifi_label;
static bool fs_mounted = false;
static bool devices_loaded = false;
static bool wifi_config_loaded = false;
static ordered_json devices_data;
static ordered_json wifi_config;
static std::string wifi_ssid;
static std::string wifi_password;

static uint32_t loop_counter = 0;

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
#ifdef BOARD_WAVESHARE_1_69
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
  
  #if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
  #else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
  #endif

#endif
#ifdef BOARD_WAVESHARE_4_3
    /* Action after last area refresh */
    if (lv_disp_flush_is_last(disp)) {
        /* Switch the current LCD frame buffer to `color_map` */
        board.getLCD()->switchFrameBufferTo(color_p);

        /* Waiting for the last frame buffer to complete transmission */
        ulTaskNotifyValueClear(NULL, ULONG_MAX);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
#endif
  
    lv_disp_flush_ready(disp);
  }

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
#ifdef BOARD_WAVESHARE_1_69
  if (touch.available()) {
    data->state = LV_INDEV_STATE_PR;

    data->point.x = touch.data.x;
    data->point.y = touch.data.y;

    std::cout << "Data x: " << touch.data.x << ", Data y: " << touch.data.y << std::endl;

    tone(BUZZER, 1000);
    delay(20);
    noTone(BUZZER);
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
#endif

#ifdef BOARD_WAVESHARE_4_3
  Touch *tp = board.getTouch();
  TouchPoint point;

  /* Read data from touch controller */
  int read_touch_result = tp->readPoints(&point, 1, 0);
  if (read_touch_result > 0) {
      data->point.x = point.x;
      data->point.y = point.y;
      data->state = LV_INDEV_STATE_PRESSED;
  } else {
      data->state = LV_INDEV_STATE_RELEASED;
  }
#endif
}

void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static int write_to_cdc(void *cookie, const char *data, int size) {
  cdc.write(data, size);
  return size;
}

void setup() {
  static lv_disp_drv_t disp_drv;

#ifdef BOARD_WAVESHARE_4_3
  board.init();

  if(!board.begin()) {
    std::cout << "Failed to initialize board" << std::endl;
    while(1) {
      delay(1000);
    }
  }

  SPI.setHwCs(false);
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_SS);
  // NOTE: IO10 is automatically used for SS, need to disable this to avoid blue tint on display
  if (!SD.begin(6)) {
      std::cout << "Card Mount Failed" << std::endl;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
      std::cout << "No SD card attached" << std::endl;
  }

  std::cout << "SD Card Type: ";
  if (cardType == CARD_MMC) {
      std::cout << "MMC" << std::endl;
  } else if (cardType == CARD_SD) {
      std::cout << "SDSC" << std::endl;
  } else if (cardType == CARD_SDHC) {
      std::cout << "SDHC" << std::endl;
  } else {
      std::cout << "UNKNOWN" << std::endl;
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  std::cout << "SD Card Size: " << cardSize << "MB" << std::endl;
#endif

  cdc.begin();
  cdc.setDebugOutput(true);

  fclose(stdout);
  stdout = funopen(nullptr, nullptr, &write_to_cdc, nullptr, nullptr);
  fclose(stderr);
  stderr = funopen(nullptr, nullptr, &write_to_cdc, nullptr, nullptr);

  partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, NULL);
  fs_mounted = FFat.begin();

  msc.onRead([](uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
    std::cout << "USB MSC read: LBA " << lba << std::endl;
    esp.partitionRead(partition, offset + lba * BLOCK_SIZE, (uint32_t *) buffer, bufsize);
    return (int32_t) bufsize;
  });
  msc.onWrite([](uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
    std::cout << "USB MSC write: LBA " << lba << std::endl;
    esp.partitionEraseRange(partition, offset + lba * BLOCK_SIZE, bufsize);
    esp.partitionWrite(partition, offset + lba * BLOCK_SIZE, (uint32_t *) buffer, bufsize);
    return (int32_t) bufsize;
  });
  if (partition == NULL) {
    std::cout << "Partition not found" << std::endl;
  } else {
    msc.mediaPresent(true);
    msc.begin(partition->size, BLOCK_SIZE);
  }

  USB.serialNumber("");
  USB.begin();

  lv_init();
  lv_disp_drv_init(&disp_drv);
#ifdef BOARD_WAVESHARE_1_69
  pinMode(BUZZER, OUTPUT);
  pinMode(SYS_EN, OUTPUT);
  pinMode(VOLTAGE_DIVIDER, INPUT);

  tone(BUZZER, 1000);
  delay(200);
  noTone(BUZZER);
  digitalWrite(SYS_EN, HIGH);

  touch.begin();

  gfx->begin();
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * screenHeight / 4);
#endif

#ifdef BOARD_WAVESHARE_4_3
  auto lcd = board.getLCD();
  screenWidth = lcd->getFrameWidth();
  screenHeight = lcd->getFrameHeight();

  for (int i = 0; (i < LVGL_PORT_DISP_BUFFER_NUM); i++) {
      lvgl_buf[i] = lcd->getFrameBufferByIndex(i);
  }

  lv_disp_draw_buf_init(&draw_buf, lvgl_buf[0], lvgl_buf[1], screenWidth * screenHeight);

  disp_drv.direct_mode = 1;
  // Only available when the coordinate alignment is enabled
  if ((lcd->getBasicAttributes().basic_bus_spec.x_coord_align > 1) ||
          (lcd->getBasicAttributes().basic_bus_spec.y_coord_align > 1)) {
      disp_drv.rounder_cb = rounder_callback;
  }
#endif

  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  lvgl_mux = xSemaphoreCreateRecursiveMutex();
  BaseType_t ret = xTaskCreatePinnedToCore(lvgl_port_task, "lvgl", LVGL_PORT_TASK_STACK_SIZE, NULL,
                   LVGL_PORT_TASK_PRIORITY, &lvgl_task_handle, ARDUINO_RUNNING_CORE);

#ifdef BOARD_WAVESHARE_4_3                   
  lcd->attachRefreshFinishCallback(onLcdVsyncCallback, (void *)lvgl_task_handle);
#endif

  if (FFat.exists("/devices.json")) {
    File f = FFat.open("/devices.json");
    try {
      devices_data = ordered_json::parse(f.readString());
      devices_loaded = true;
    } catch (const std::exception& e) {
      std::cout << "Error parsing devices.json: " << e.what() << std::endl;
    }
    f.close();
  }

  if (FFat.exists("/wifi.json")) {
    File f = FFat.open("/wifi.json");
    try {
      wifi_config = ordered_json::parse(f.readString());
      wifi_ssid = wifi_config["ssid"].get<std::string>();
      wifi_password = wifi_config["password"].get<std::string>();
      wifi_config_loaded = true;
    } catch (const std::exception& e) {
      std::cout << "Error parsing wifi.json: " << e.what() << std::endl;
    }
    f.close();
  }

  s = std::make_unique<tuya::Scanner>(l, devices_data);

  lvgl_port_lock(-1);

  lv_obj_t * cont_col = lv_obj_create(lv_scr_act());
  lv_obj_align(cont_col, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_size(cont_col, screenWidth, screenHeight);
  lv_obj_set_flex_flow(cont_col, LV_FLEX_FLOW_COLUMN);

  for (const auto& ip : s->getDevices()) {
    const auto& dev = s->getDevice(ip);
    lv_obj_t *btn = lv_btn_create(cont_col);
    lv_obj_t *label_btn = lv_label_create(btn);
    lv_label_set_text(label_btn, dev->name().c_str());
    lv_obj_add_event_cb(btn, [](lv_event_t *event) {
      tuya::Device *d = static_cast<tuya::Device *>(event->user_data);
      if (event->code == LV_EVENT_CLICKED) {
        std::cout << "Button " << d->name() << std::endl;
        d->toggle();
      }
    }, LV_EVENT_CLICKED, dev.get());
  }

  battery_label = lv_label_create(cont_col);
  lv_label_set_text(battery_label, "Initializing...");

  wifi_label = lv_label_create(cont_col);
  lv_label_set_text(wifi_label, "Initializing...");

  if (!fs_mounted || !devices_loaded || !wifi_config_loaded) {
    lv_obj_t* warning_label = lv_label_create(cont_col);
    lv_label_set_text(warning_label, "Warning: Storage not\nformatted or data missing");
    lv_obj_t* format_button = lv_btn_create(cont_col);
    lv_obj_t* format_label = lv_label_create(format_button);
    lv_label_set_text(format_label, "Format storage");
    lv_obj_add_event_cb(format_button, [](lv_event_t *event) {
      if (event->code == LV_EVENT_CLICKED) {
        FFat.end();
        FFat.format();
        ESP.restart();
      }
    }, LV_EVENT_CLICKED, NULL);
  }

  xSemaphoreGiveRecursive(lvgl_mux);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  if (wifi_config_loaded) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
  }
}

void loop() {
  uint32_t t0 = millis();
  uint32_t dt;
  if (loop_counter++ % 50 == 0) {
#ifdef BOARD_WAVESHARE_1_69
    uint16_t adc_val = analogRead(VOLTAGE_DIVIDER);
    float voltage = (float)adc_val * (VREF / 4095.0);
    float vbat = voltage * ((R1 + R2) / R2);
    String voltage_str = "Battery voltage: " + String(vbat) + " V";
    lv_label_set_text(battery_label, voltage_str.c_str());
#endif

    switch (WiFi.status()) {
      case WL_CONNECTED:
        lv_label_set_text(wifi_label, "WiFi connected");
        break;
      case WL_NO_SHIELD:
      case WL_IDLE_STATUS:
      case WL_NO_SSID_AVAIL:
      case WL_SCAN_COMPLETED:
      case WL_CONNECT_FAILED:
      case WL_CONNECTION_LOST:
      case WL_DISCONNECTED:
        lv_label_set_text(wifi_label, "WiFi disconnected");
        break;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    l.loop(10);
  }

  dt = millis() - t0;
  if (dt < 10) {
    delay(10 - dt);
  }
}
