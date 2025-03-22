#include <Arduino_DriveBus_Library.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <tuyacpp/scanner.hpp>
#include <WiFi.h>

#include "pin_config.h"
#include "wifi_config.h"
#include "devices.json.h"

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

void Arduino_IIC_Touch_Interrupt(void);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
std::unique_ptr<Arduino_IIC> CST816T(new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS, TP_RST, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  CST816T->IIC_Interrupt_Flag = true;
}

uint32_t screenWidth;
uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;

tuya::Loop l;
std::unique_ptr<tuya::Scanner> s;

static lv_obj_t *battery_label;

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
  int32_t touchX = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  if (CST816T->IIC_Interrupt_Flag == true) {
    CST816T->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;

    /* Set the coordinates with some debounce */
    if (touchX >= 0 && touchY >= 0) {
      data->point.x = touchX;
      data->point.y = touchY;

      std::cout << "Data x: " << touchX << ", Data y: " << touchY << std::endl;
    }
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

  while (CST816T->begin() == false) {
    std::cout << "CST816T initialization fail" << std::endl;
    delay(1000);
  }
  std::cout << "CST816T initialization successfully" << std::endl;

  CST816T->IIC_Write_Device_State(CST816T->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
                                  CST816T->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);

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

  while(WiFi.status() != WL_CONNECTED){
      delay(100);
  }

  std::cout << "Connected to the WiFi network" << std::endl;
  std::cout << "IP Address: " << WiFi.localIP().toString().c_str() << std::endl;
}

void loop() {
  uint16_t adc_val = analogRead(VOLTAGE_DIVIDER);
  float voltage = (float)adc_val * (VREF / 4095.0);
  float vbat = voltage * ((R1 + R2) / R2);
  String voltage_str = "Battery voltage: " + String(vbat) + " V";
  lv_label_set_text(battery_label, voltage_str.c_str());

  l.loop(100);
  lv_timer_handler();
}
