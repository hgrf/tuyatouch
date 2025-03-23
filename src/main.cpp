#include <Arduino_GFX_Library.h>
#include <CST816S.h>
#include <lvgl.h>
#include <tuyacpp/scanner.hpp>
#include <WiFi.h>

#include "pin_config.h"
#include "wifi_config.h"
#include "devices.json.h"

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

CST816S touch(IIC_SDA, IIC_SCL, TP_RST, TP_INT);

uint32_t screenWidth;
uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;

tuya::Loop l;
std::unique_ptr<tuya::Scanner> s;

static lv_obj_t *battery_label;
static lv_obj_t *wifi_label;

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
  
  #if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
  #else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
  #endif
  
    lv_disp_flush_ready(disp);
  }

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
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
}

void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void setup() {
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

  lv_init();

  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(screenWidth * screenHeight / 4 * sizeof(lv_color_t), MALLOC_CAP_DMA);

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * screenHeight / 4);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
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

  std::string data((char *) devices_json, devices_json_len);
  s = std::make_unique<tuya::Scanner>(l, ordered_json::parse(data));

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

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  std::cout << "Connecting to the WiFi network" << std::endl;
}

void loop() {
  uint16_t adc_val = analogRead(VOLTAGE_DIVIDER);
  float voltage = (float)adc_val * (VREF / 4095.0);
  float vbat = voltage * ((R1 + R2) / R2);
  String voltage_str = "Battery voltage: " + String(vbat) + " V";
  lv_label_set_text(battery_label, voltage_str.c_str());

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

  l.loop(10);
  lv_timer_handler();
}
