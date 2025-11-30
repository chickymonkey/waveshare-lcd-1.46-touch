#include "spd2010_touch.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"


namespace esphome {
namespace spd2010_touch {

static const char *const TAG = "spd2010_touch";
static const uint8_t SPD2010_ADDR = 0x53;

// Register map
static const uint16_t REG_CLEAR_INT = 0x0002;
static const uint16_t REG_CPU_START = 0x0004;
static const uint16_t REG_START_TOUCH = 0x0046;
static const uint16_t REG_POINT_MODE = 0x0050;
static const uint16_t REG_STATUS = 0x0020;
static const uint16_t REG_HDP = 0x0300;
static const uint16_t REG_HDP_STATUS = 0x02FC;
static const uint16_t REG_FW_VERSION = 0x0026;

// Helper for hex dump
static std::string format_hex(const uint8_t *data, size_t len) {
  char buf[256];
  size_t pos = 0;
  for (size_t i = 0; i < len && pos < sizeof(buf) - 3; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", data[i]);
  }
  return std::string(buf);
}

void SPD2010Touch::setup() {

  if (this->rst_pin_) {
    this->rst_pin_->setup();

    // Active-low reset
    this->rst_pin_->digital_write(false); // Hold reset
    vTaskDelay(pdMS_TO_TICKS(50));        // 50 ms
    this->rst_pin_->digital_write(true);  // Release reset
    vTaskDelay(pdMS_TO_TICKS(50));        // Boot time
    ESP_LOGI(TAG, "SPD2010 reset complete");
    ESP_LOGI(TAG, "Reset pin state: %d", this->rst_pin_->digital_read());

  }

  // Debug: Check SPD2010 state after reset
  vTaskDelay(pdMS_TO_TICKS(200));  // Give SPD2010 time to boot
  uint8_t status_buf[4];
  if (read_bytes16_(REG_STATUS, status_buf, 4)) {
    ESP_LOGI(TAG, "SPD2010 Status: %02X %02X %02X %02X",
             status_buf[0], status_buf[1], status_buf[2], status_buf[3]);
    ESP_LOGI(TAG, "BIOS=%d CPU=%d RUN=%d PT=%d",
             (status_buf[1] & 0x40) ? 1 : 0,
             (status_buf[1] & 0x20) ? 1 : 0,
             (status_buf[1] & 0x08) ? 1 : 0,
             (status_buf[0] & 0x01) ? 1 : 0);
  } else {
    ESP_LOGE(TAG, "Failed to read REG_STATUS");
  }

  // Read FW version for diagnostics
  read_fw_version_();


  // Initial command sequence
  ESP_LOGI(TAG, "Sending initial SPD2010 commands...");
  write_cmd_(REG_CLEAR_INT, 0x0001);
  vTaskDelay(pdMS_TO_TICKS(50));
  write_cmd_(REG_CPU_START, 0x0001);
  vTaskDelay(pdMS_TO_TICKS(50));
  write_cmd_(REG_POINT_MODE, 0x0000);
  vTaskDelay(pdMS_TO_TICKS(50));
  write_cmd_(REG_START_TOUCH, 0x0000);
  vTaskDelay(pdMS_TO_TICKS(50));
  clear_int_sequence_();
  vTaskDelay(pdMS_TO_TICKS(100));

  // Test read
  ESP_LOGI(TAG, "Testing read from REG_STATUS...");
  uint8_t dummy[4] = {0};
  bool ok = read_bytes16_(REG_STATUS, dummy, 4);
  ESP_LOGI(TAG, "Initial status read: %s", ok ? "OK" : "FAIL");

  this->register_lvgl_indev_();
}


// Write command: reg big-endian, data little-endian
bool SPD2010Touch::write_cmd_(uint16_t reg, uint16_t val) {
  uint8_t buf[4] = {
    static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
    static_cast<uint8_t>(val & 0xFF), static_cast<uint8_t>(val >> 8)
  };

  for (int attempt = 0; attempt < 3; attempt++) {
    if (this->write(buf, 4)) {
      ESP_LOGD(TAG, "WRITE CMD success: reg=0x%04X val=0x%04X (attempt %d)", reg, val, attempt + 1);
      vTaskDelay(pdMS_TO_TICKS(200));
      return true;
    }
    ESP_LOGW(TAG, "WRITE CMD failed, retrying (attempt %d)...", attempt + 1);
    vTaskDelay(pdMS_TO_TICKS(20));;
  }

  ESP_LOGE(TAG, "WRITE CMD failed after retries: reg=0x%04X", reg);
  return false;
}

// Read with repeated start
bool SPD2010Touch::read_bytes16_(uint16_t reg, uint8_t *buf, size_t len) {
  uint8_t addr[2] = {
    static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF)
  };

  for (int attempt = 0; attempt < 3; attempt++) {
    if (this->write_read(addr, 2, buf, len)) {
      ESP_LOGD(TAG, "READ success: reg=0x%04X len=%d (attempt %d)", reg, (int)len, attempt + 1);
      ESP_LOGD(TAG, "READ data: %s", format_hex(buf, len).c_str());
      return true;
    }
    ESP_LOGW(TAG, "READ failed, retrying (attempt %d)...", attempt + 1);
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  ESP_LOGE(TAG, "READ failed after retries: reg=0x%04X", reg);
  return false;
}

void SPD2010Touch::clear_int_sequence_() {
  ESP_LOGD(TAG, "Executing Clear INT sequence...");
  write_cmd_(REG_CLEAR_INT, 0x0001);
  esp_rom_delay_us(200);
  write_cmd_(REG_CLEAR_INT, 0x0000);
  esp_rom_delay_us(200);
}

void SPD2010Touch::read_fw_version_() {
  uint8_t buf[18] = {0};
  if (read_bytes16_(REG_FW_VERSION, buf, sizeof(buf))) {
    uint16_t DVer = (buf[5] << 8) | buf[4];
    ESP_LOGI(TAG, "SPD2010 FW Version: %u", DVer);
  } else {
    ESP_LOGW(TAG, "Failed to read FW version");
  }
}

void SPD2010Touch::register_lvgl_indev_() {
  lv_indev_drv_init(&this->indev_drv_);
  this->indev_drv_.type = LV_INDEV_TYPE_POINTER;
  this->indev_drv_.read_cb = &SPD2010Touch::lvgl_read_cb_;
  this->indev_drv_.user_data = this;
  this->indev_ = lv_indev_drv_register(&this->indev_drv_);
  ESP_LOGI(TAG, "LVGL input device registered");
}

void SPD2010Touch::loop() {
  // LVGL handles polling
}

void SPD2010Touch::dump_config() {
  ESP_LOGCONFIG(TAG, "SPD2010 Touch (I2C addr=0x%02X)", SPD2010_ADDR);
  ESP_LOGCONFIG(TAG, "Screen size: %ux%u", this->w_, this->h_);
  ESP_LOGCONFIG(TAG, "Orientation: swap_xy=%s mirror_x=%s mirror_y=%s",
                this->swap_xy_ ? "true" : "false",
                this->mirror_x_ ? "true" : "false",
                this->mirror_y_ ? "true" : "false");
}

void SPD2010Touch::lvgl_read_cb_(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  auto *self = reinterpret_cast<SPD2010Touch *>(drv->user_data);
  uint32_t now = millis();
  if (self->poll_interval_ms_ && (now - self->last_poll_ms_ < self->poll_interval_ms_)) {
    data->state = self->last_pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->point.x = self->last_x_;
    data->point.y = self->last_y_;
    return;
  }
  self->last_poll_ms_ = now;

  uint16_t x, y;
  uint8_t w;
  bool pressed;
  if (!self->tp_read_data_(x, y, w, pressed)) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  uint16_t rx = x, ry = y;
  if (self->swap_xy_) std::swap(rx, ry);
  if (self->mirror_x_) rx = self->w_ - 1 - rx;
  if (self->mirror_y_) ry = self->h_ - 1 - ry;

  data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  if (pressed) {
    data->point.x = rx;
    data->point.y = ry;
    self->last_x_ = rx;
    self->last_y_ = ry;
  }
  self->last_pressed_ = pressed;
}

// Enhanced tp_read_data_ with BIOS/CPU state handling and HDP loop
bool SPD2010Touch::tp_read_data_(uint16_t &x, uint16_t &y, uint8_t &weight, bool &pressed) {
  uint8_t status_buf[4];
  if (!read_bytes16_(REG_STATUS, status_buf, 4)) {
    ESP_LOGE(TAG, "Failed to read status");
    return false;
  }

  bool pt_exist = status_buf[0] & 0x01;
  bool gesture = status_buf[0] & 0x02;
  bool aux = status_buf[0] & 0x08;
  bool tic_in_bios = status_buf[1] & 0x40;
  bool tic_in_cpu = status_buf[1] & 0x20;
  bool cpu_run = status_buf[1] & 0x08;
  uint16_t read_len = (status_buf[3] << 8) | status_buf[2];

  if (tic_in_bios) {
    ESP_LOGW(TAG, "SPD2010 in BIOS mode, sending CPU start...");
    clear_int_sequence_();
    write_cmd_(REG_CPU_START, 0x0001);
    return false;
  }
  if (tic_in_cpu) {
    ESP_LOGW(TAG, "SPD2010 in CPU mode, sending start sequence...");
    write_cmd_(REG_POINT_MODE, 0x0000);
    write_cmd_(REG_START_TOUCH, 0x0000);
    clear_int_sequence_();
    return false;
  }
  if (cpu_run && read_len == 0) {
    clear_int_sequence_();
    return false;
  }
  if (!pt_exist && !gesture) {
    if (cpu_run && aux) clear_int_sequence_();
    pressed = false;
    return true;
  }

  // Read HDP packet
  uint8_t buf[64] = {0};
  if (!read_bytes16_(REG_HDP, buf, read_len)) {
    ESP_LOGE(TAG, "Failed to read HDP packet");
    return false;
  }

  x = (((buf[7] & 0xF0) << 4) | buf[5]);
  y = (((buf[7] & 0x0F) << 8) | buf[6]);
  weight = buf[8];
  pressed = (weight > 0);

  // HDP done-check loop
  while (true) {
    uint8_t hdp_status_buf[8] = {0};
    if (!read_bytes16_(REG_HDP_STATUS, hdp_status_buf, sizeof(hdp_status_buf))) break;
    uint8_t status = hdp_status_buf[5];
    uint16_t next_len = (hdp_status_buf[2] | (hdp_status_buf[3] << 8));
    if (status == 0x82) {
      clear_int_sequence_();
      break;
    } else if (status == 0x00 && next_len > 0) {
      uint8_t remain_buf[32] = {0};
      read_bytes16_(REG_HDP, remain_buf, next_len);
      continue;
    } else break;
  }

  ESP_LOGD(TAG, "Touch: X=%d, Y=%d, Weight=%d, Pressed=%d", x, y, weight, pressed);
  return true;
}

}  // namespace spd2010_touch
}  // namespace esphome