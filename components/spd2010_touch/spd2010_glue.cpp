#include "spd2010_glue.h"
#include "esphome/core/log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "lvgl.h" 

#include <vector>
#include <algorithm>

namespace esphome {
namespace spd2010_glue {

static const char *const TAG = "spd2010_glue";

// Forward declaration so lvgl_read_cb_() can call it
static uint8_t parse_hdp_first_points_(const uint8_t *d,
                                       size_t len,
                                       esp_lcd_touch_point_data_t *out,
                                       uint8_t max_points);


// ===== SPD2010 raw register helpers (16-bit address) =====

// Write 16-bit register value (little-endian data after big-endian reg address)
inline esp_err_t spd_write_reg16_(i2c_master_dev_handle_t dev, uint16_t reg, uint16_t val_le) {
  uint8_t buf[4] = {
    (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),        // reg high, reg low
    (uint8_t)(val_le & 0xFF), (uint8_t)(val_le >> 8)   // value low, value high (little-endian)
  };
  return i2c_master_transmit(dev, buf, sizeof(buf), -1);
}

// Read register block (write 2-byte reg, read 'len' bytes)
inline esp_err_t spd_read_reg_(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t *out, size_t len) {
  uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
  // write reg address then read
  esp_err_t err = i2c_master_transmit(dev, addr, sizeof(addr), -1);
  if (err != ESP_OK) return err;
  return i2c_master_receive(dev, out, len, -1);
}

// --- Register addresses & meanings ---
static constexpr uint16_t REG_TP_STATUS_LEN   = 0x2000; // 4 bytes: status_low, status_high, len_lo, len_hi
static constexpr uint16_t REG_HDP             = 0x0003; // HDP payload (length = st.read_len)
static constexpr uint16_t REG_HDP_STATUS      = 0xFC02; // 8 bytes: next_len_lo, next_len_hi, ..., status @ [5]
static constexpr uint16_t REG_CLEAR_INT       = 0x0200; // write 0x0001
static constexpr uint16_t REG_CPU_START       = 0x0400; // write 0x0001
static constexpr uint16_t REG_POINT_MODE      = 0x5000; // write 0x0000
static constexpr uint16_t REG_START           = 0x4600; // write 0x0000

// --- Bit helpers: sample_data[0] == status_low, sample_data[1] == status_high ---
inline bool pt_exist   (uint8_t sl) { return (sl & 0x01) != 0; }
inline bool gesture    (uint8_t sl) { return (sl & 0x02) != 0; }
inline bool aux        (uint8_t sl) { return (sl & 0x08) != 0; }

inline bool tic_busy   (uint8_t sh) { return (sh & 0x80) != 0; }
inline bool tic_in_bios(uint8_t sh) { return (sh & 0x40) != 0; }
inline bool tic_in_cpu (uint8_t sh) { return (sh & 0x20) != 0; }
inline bool tint_low   (uint8_t sh) { return (sh & 0x10) != 0; }
inline bool cpu_run    (uint8_t sh) { return (sh & 0x08) != 0; }


struct tp_status_t {
  struct {
    uint8_t pt_exist : 1;
    uint8_t gesture  : 1;
    uint8_t reserved : 1;
    uint8_t aux      : 1;
  } status_low;
  struct {
    uint8_t tic_busy   : 1;
    uint8_t tic_in_bios: 1;
    uint8_t tic_in_cpu : 1;
    uint8_t tint_low   : 1;
    uint8_t cpu_run    : 1;
  } status_high;
  uint16_t read_len;
};

// Read TP status + total length
esp_err_t read_tp_status_length_(i2c_master_dev_handle_t dev, tp_status_t *st) {
  uint8_t d[4] = {0};
  ESP_RETURN_ON_ERROR(spd_read_reg_(dev, REG_TP_STATUS_LEN, d, sizeof(d)), TAG, "read TP status len failed");
  st->status_low.pt_exist    = (d[0] & 0x01);
  st->status_low.gesture     = (d[0] & 0x02);
  st->status_low.aux         = (d[0] & 0x08);

  st->status_high.tic_busy   = ((d[1] & 0x80) >> 7);
  st->status_high.tic_in_bios= ((d[1] & 0x40) >> 6);
  st->status_high.tic_in_cpu = ((d[1] & 0x20) >> 5);
  st->status_high.tint_low   = ((d[1] & 0x10) >> 4);
  st->status_high.cpu_run    = ((d[1] & 0x08) >> 3);
  // little-endian length
  st->read_len = (uint16_t)d[3] << 8 | (uint16_t)d[2];
  return ESP_OK;
}

// Read HDP status (8 bytes), status lives at [5], next packet length lives at [2..3]
struct tp_hdp_status_t {
  uint8_t status;
  uint16_t next_packet_len;
};
esp_err_t read_tp_hdp_status_(i2c_master_dev_handle_t dev, tp_hdp_status_t *hs) {
  uint8_t d[8] = {0};
  ESP_RETURN_ON_ERROR(spd_read_reg_(dev, REG_HDP_STATUS, d, sizeof(d)), TAG, "read HDP status failed");
  hs->status = d[5];
  hs->next_packet_len = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
  return ESP_OK;
}

// Read HDP remain data (next_packet_len bytes)
esp_err_t read_hdp_remain_(i2c_master_dev_handle_t dev, uint16_t len) {
  if (len == 0) return ESP_OK;
  //  reads from REG_HDP (0x0003) again for remain data
  std::vector<uint8_t> tmp(len);
  return spd_read_reg_(dev, REG_HDP, tmp.data(), tmp.size());
}

// Clear INT (write 0x0001), with small settling delay
inline esp_err_t clear_int_(i2c_master_dev_handle_t dev) {
  ESP_RETURN_ON_ERROR(spd_write_reg16_(dev, REG_CLEAR_INT, 0x0001), TAG, "clear INT");
  esp_rom_delay_us(200);
  return ESP_OK;
}

// CPU start and point-mode/start helpers
inline esp_err_t cpu_start_(i2c_master_dev_handle_t dev) {
  ESP_RETURN_ON_ERROR(spd_write_reg16_(dev, REG_CPU_START, 0x0001), TAG, "CPU start");
  esp_rom_delay_us(200);
  return ESP_OK;
}
inline esp_err_t point_mode_(i2c_master_dev_handle_t dev) {
  ESP_RETURN_ON_ERROR(spd_write_reg16_(dev, REG_POINT_MODE, 0x0000), TAG, "point mode");
  esp_rom_delay_us(200);
  return ESP_OK;
}
inline esp_err_t start_(i2c_master_dev_handle_t dev) {
  ESP_RETURN_ON_ERROR(spd_write_reg16_(dev, REG_START, 0x0000), TAG, "start");
  esp_rom_delay_us(200);
  return ESP_OK;
}

// Post-read IRQ service

esp_err_t post_read_irq_service_(i2c_master_dev_handle_t dev, bool *cleared) {
  tp_status_t st{};
  ESP_RETURN_ON_ERROR(read_tp_status_length_(dev, &st), TAG, "read_tp_status_length failed");

  // Handle controller states
  if (st.status_high.tic_in_bios) {
    ESP_RETURN_ON_ERROR(clear_int_(dev), TAG, "clear INT");
    ESP_RETURN_ON_ERROR(cpu_start_(dev), TAG, "CPU start");
    return ESP_OK;
  }
  if (st.status_high.tic_in_cpu) {
    ESP_RETURN_ON_ERROR(point_mode_(dev), TAG, "point mode");
    ESP_RETURN_ON_ERROR(start_(dev), TAG, "start");
    ESP_RETURN_ON_ERROR(clear_int_(dev), TAG, "clear INT");
    return ESP_OK;
  }
  if (st.status_high.cpu_run && st.read_len == 0) {
    ESP_RETURN_ON_ERROR(clear_int_(dev), TAG, "clear INT");
    return ESP_OK;
  }

  // If touch data or gesture flagged, DO NOT read REG_HDP here
  // The esp_lcd_touch driver has already read the frame in esp_lcd_touch_read_data().
  if (st.status_low.pt_exist || st.status_low.gesture) {
    tp_hdp_status_t hs{};
    // Loop until CONTINUE (0x00 with next_packet_len) is drained
    // and we see DONE (0x82), then clear INT.
    while (true) {
      ESP_RETURN_ON_ERROR(read_tp_hdp_status_(dev, &hs), TAG, "read HDP status failed");
      if (hs.status == 0x82) {
        ESP_RETURN_ON_ERROR(clear_int_(dev), TAG, "clear INT");
        if (cleared) *cleared = true;
        break;
      } else if (hs.status == 0x00 && hs.next_packet_len > 0) {
        // Drain the remaining chunk (the main frame was already consumed by the driver)
        ESP_RETURN_ON_ERROR(read_hdp_remain_(dev, hs.next_packet_len), TAG, "read remain");
        // Continue the loop to check if more remain and eventually hit 0x82
      } else {
        // Unknown/idle status: break and avoid tight loop
        break;
      }
    }
    return ESP_OK;
  }


  if (st.status_high.cpu_run && st.status_low.aux) {
    ESP_RETURN_ON_ERROR(clear_int_(dev), TAG, "clear INT");
  }

  return ESP_OK;
}

void Spd2010LvglGlue::begin() {
  if (started_) { ESP_LOGI(TAG, "Touch already started; skipping."); return; }

  // --- New I²C master bus API (IDF 5.x) ---
  i2c_master_bus_config_t bus_cfg{};
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.i2c_port   = I2C_NUM_0;
  bus_cfg.sda_io_num = (gpio_num_t)11;   // GPIO11
  bus_cfg.scl_io_num = (gpio_num_t)10;   // GPIO10
  bus_cfg.flags.enable_internal_pullup = true;

  i2c_master_bus_handle_t i2c_bus = nullptr;
  esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_new_master_bus failed: %d", (int)err);
    return;
  }

  // This verifies the touch controller at 0x53 ACKs before we install the panel IO.
  esp_err_t probe = i2c_master_probe(i2c_bus, 0x53, -1);
  ESP_LOGI(TAG, "SPD2010 probe 0x53: %s", esp_err_to_name(probe));
  // Optional: bail early if not present
  if (probe != ESP_OK) {
    // Helpful hint to yourself during bring-up; you can relax this later.
    ESP_LOGE(TAG, "SPD2010 not responding at 0x53. Check wiring/pull-ups.");
    return;
  }

  // Install LCD touch IO on this I2C bus (v2 IO)
  esp_lcd_panel_io_handle_t io = nullptr;
  esp_lcd_panel_io_i2c_config_t io_cfg = ESP_LCD_TOUCH_IO_I2C_SPD2010_CONFIG();
  io_cfg.scl_speed_hz = 400000;              
  io_cfg.flags.disable_control_phase = 0;     // enable a simple control phase
  io_cfg.control_phase_bytes = 1;
  io_cfg.dc_bit_offset = 0;
  io_cfg.lcd_cmd_bits = 8;
  io_cfg.lcd_param_bits = 8;

  err = esp_lcd_new_panel_io_i2c(i2c_bus, &io_cfg, &io);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_lcd_new_panel_io_i2c failed: %d", (int)err);
    return;
  }

  // Create SPD2010 touch handle
  esp_lcd_touch_config_t tp_cfg = {
    .x_max = w_, .y_max = h_,
    .rst_gpio_num = GPIO_NUM_NC,
    .int_gpio_num = (gpio_num_t)int_gpio_,
    .levels = { .reset = 0, .interrupt = 0 },
    .flags = {
      .swap_xy = (uint8_t)(swap_xy_ ? 1 : 0),
      .mirror_x = (uint8_t)(mirror_x_ ? 1 : 0),
      .mirror_y = (uint8_t)(mirror_y_ ? 1 : 0),
    },
  };
  esp_lcd_touch_handle_t tp = nullptr;
  err = esp_lcd_touch_new_i2c_spd2010(io, &tp_cfg, &tp);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "touch_new_i2c_spd2010 failed: %d", (int)err);
    return;
  }

  // Save handles
  this->tp_      = tp;
  this->io_      = io;
  this->i2c_bus_ = i2c_bus;

  // Add SPD2010 raw device handle on bus (0x53) so we can issue reg ops
  i2c_device_config_t spd_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address  = 0x53,
    .scl_speed_hz    = 400000,              // use your chosen rate (100k/400k)
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(this->i2c_bus_, &spd_cfg, &this->spd_dev_));
  ESP_LOGI(TAG, "SPD2010 raw dev added at 0x53");

  // Put controller in point mode and start sampling before LVGL begins polling
  (void)point_mode_(this->spd_dev_);
  (void)start_(this->spd_dev_);

  // Optional: PCA9554 on the same bus
  if (this->pca9554_init(0x20) == ESP_OK) {
    (void)this->pca9554_set_pin_mode(1, /*output=*/true);
    (void)this->pca9554_write_pin(1, true);
  }

  // INT pin as input
  gpio_config_t io_gpio{};
  io_gpio.pin_bit_mask = 1ULL << int_gpio_;
  io_gpio.mode = GPIO_MODE_INPUT;
  io_gpio.pull_up_en = GPIO_PULLUP_ENABLE;
  io_gpio.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_gpio.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_gpio);

  // Register LVGL input device - deferred to loop
  //this->register_lvgl_indev_();
  
  // after this->register_lvgl_indev_();
  for (int n = 0; n < 5; ++n) {
    esp_lcd_touch_read_data(this->tp_);
    uint8_t cnt = 0;
    esp_lcd_touch_point_data_t p[1] = {0};
    esp_lcd_touch_get_data(this->tp_, p, &cnt, 1);
    if (cnt > 0) {
      ESP_LOGI(TAG, "During setup Touch: [%u, %u], strength=%u", p[0].x, p[0].y, p[0].strength);
    } else {
      ESP_LOGI(TAG, "No touch during setup");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  //late setup_priority
  started_ = true;
}


void Spd2010LvglGlue::setup() {
  // Intentionally empty: we’ll call begin() later from on_boot.
  ESP_LOGI(TAG, "Deferring SPD2010 init to on_boot/begin()");
}


void Spd2010LvglGlue::dump_config() {
  ESP_LOGCONFIG(TAG, "SPD2010 LVGL glue (I2C addr=0x53)");
  ESP_LOGCONFIG(TAG, "Screen: %ux%u, swap_xy=%s, mirror_x=%s, mirror_y=%s, INT=GPIO%d",
                w_, h_,
                swap_xy_ ? "true" : "false",
                mirror_x_ ? "true" : "false",
                mirror_y_ ? "true" : "false",
                int_gpio_);
}

void Spd2010LvglGlue::register_lvgl_indev_() {
  // LVGL v9: no driver struct; create indev directly and configure via setters
  this->indev_ = lv_indev_create();
  lv_indev_set_type(this->indev_, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(this->indev_, &Spd2010LvglGlue::lvgl_read_cb_);
  lv_indev_set_user_data(this->indev_, this);
  lv_indev_set_display(this->indev_, lv_display_get_default());

  ESP_LOGI(TAG, "LVGL indev registered");
}

void Spd2010LvglGlue::loop() {
  if (!indev_registered_) {
    
    if (lv_display_get_default()) {
      register_lvgl_indev_();
      indev_registered_ = true;
    }
  }
  
  // Lightweight, time-gated polling
  const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  if (now_ms - last_touch_poll_ms_ < kTouchPollMs) return;
  last_touch_poll_ms_ = now_ms;

  // Only sample when IRQ is asserted (low)
  const int irq_lvl = gpio_get_level(static_cast<gpio_num_t>(int_gpio_));
  if (irq_lvl != 0 || !spd_dev_) return;

  // 1) Read status to get length
  tp_status_t st{};
  if (read_tp_status_length_(spd_dev_, &st) != ESP_OK) return;

  // If no points/gesture, release
  if (!(st.status_low.pt_exist || st.status_low.gesture) || st.read_len == 0) {
    last_pressed_ = false;
    return;
  }

  // 2) Read HDP frame (clamp to buffer)
  const size_t to_read = std::min<size_t>(st.read_len, kMaxHdpBytes);
  if (spd_read_reg_(spd_dev_, REG_HDP, hdp_buf_, to_read) != ESP_OK) return;

  // 3) Parse into a local point (no heap allocs)
  esp_lcd_touch_point_data_t points[5] = {{0}};
  uint8_t cnt = parse_hdp_first_points_(hdp_buf_, to_read, points, 5);

  if (cnt > 0) {
    last_pressed_ = true;

    // --- ORIENTATION TRANSFORM, read config from yaml---
    uint16_t x = points[0].x;
    uint16_t y = points[0].y;

    if (swap_xy_) {
      uint16_t tmp = x; x = y; y = tmp;
    }
    if (mirror_x_) {
      x = (w_ > 0) ? (w_ - 1 - x) : x;
    }
    if (mirror_y_) {
    y = (h_ > 0) ? (h_ - 1 - y) : y;
    }

    // Cache transformed values

    last_x_ = x;
    last_y_ = y;
  } else {
    last_pressed_ = false;
  }

  // 4) Post-read service (drain remain + clear INT)
  bool cleared = false;
  (void)post_read_irq_service_(spd_dev_, &cleared);

  // Optional: stuck IRQ detector
  update_irq_stuck_detector_(now_ms, gpio_get_level(static_cast<gpio_num_t>(int_gpio_)));

  
}

void Spd2010LvglGlue::lvgl_read_cb_(lv_indev_t *indev, lv_indev_data_t *data) {
  auto *self = reinterpret_cast<Spd2010LvglGlue *>(lv_indev_get_user_data(indev));
  if (!self) { data->state = LV_INDEV_STATE_RELEASED; data->point.x = data->point.y = 0; return; }

  const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  if (now_ms - self->last_poll_ms_ < kPollMs) {
    data->state   = self->last_pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->point.x = self->last_x_; 
    data->point.y = self->last_y_;
    //return;
  }
  self->last_poll_ms_ = now_ms;

  // Sample IRQ
  int irq_before = gpio_get_level(static_cast<gpio_num_t>(self->int_gpio_));
  ESP_LOGV(TAG, "INT level (pre): %d (0=asserted, 1=idle)", irq_before);

  uint8_t cnt = 0;
  esp_lcd_touch_point_data_t points[5] = {0};

  if (irq_before == 0 && self->spd_dev_) {
    // 1) Read status to get frame length
    tp_status_t st{};
    if (read_tp_status_length_(self->spd_dev_, &st) == ESP_OK) {
      if (st.read_len > 0 && (st.status_low.pt_exist || st.status_low.gesture)) {
        // 2) Read HDP frame

        std::vector<uint8_t> hdp(st.read_len);

        // TEMP LOG
        //ESP_LOGD(TAG, "status: pt=%d gest=%d len=%u",
        //         (int)st.status_low.pt_exist, (int)st.status_low.gesture, (unsigned)st.read_len);

        if (spd_read_reg_(self->spd_dev_, REG_HDP, hdp.data(), hdp.size()) == ESP_OK) {
          // 3) Parse to LVGL point(s)
          cnt = parse_hdp_first_points_(hdp.data(), hdp.size(), points, 5);

          if (cnt > 0) {
            // --- ORIENTATION TRANSFORM (add this block) ---
            uint16_t x = points[0].x;
            uint16_t y = points[0].y;

            // Apply YAML-configured transforms locally
            if (self->swap_xy_) {
              uint16_t tmp = x; x = y; y = tmp;
            }
            if (self->mirror_x_) {
              x = (self->w_ > 0) ? (self->w_ - 1 - x) : x;
              ESP_LOGD(TAG, "Final touch point x=%u",
                     cnt, points[0].x, points[0].y, points[0].strength);
            }
            if (self->mirror_y_) {
              y = (self->h_ > 0) ? (self->h_ - 1 - y) : y;
            }

            // Then use the transformed coordinates
            data->state   = LV_INDEV_STATE_PRESSED;
            data->point.x = x;
            data->point.y = y;

            self->last_pressed_ = true;
            self->last_x_ = x;
            self->last_y_ = y;

            ESP_LOGD(TAG, "Final touch point cnt=%u, x=%u y=%u strength=%u",
                       cnt, x, y, points[0].strength);
          } else {
            // gesture-only or zero-weight -> release
            data->state   = LV_INDEV_STATE_RELEASED;
            data->point.x = self->last_x_; data->point.y = self->last_y_;
            self->last_pressed_ = false;

            //ESP_LOGD(TAG, "cnt=0 (gesture-only or zero-weight)");
          }
        } else {
          // HDP read failed; keep last state
          data->state   = self->last_pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
          data->point.x = self->last_x_; data->point.y = self->last_y_;
          ESP_LOGW(TAG, "HDP read failed");
        }
      } else {
        // Status shows no points/gesture => release
        data->state   = LV_INDEV_STATE_RELEASED;
        data->point.x = self->last_x_; data->point.y = self->last_y_;
        self->last_pressed_ = false;
        //ESP_LOGD(TAG, "Status no points (len=%u)", st.read_len);
      }
    } else {
      ESP_LOGW(TAG, "Status read failed");
    }

    // 4) Run the controller status/clear loop (remain chunks + Clear-INT)
    bool cleared = false;
    (void)post_read_irq_service_(self->spd_dev_, &cleared);
    int irq_after = gpio_get_level(static_cast<gpio_num_t>(self->int_gpio_));
    ESP_LOGV(TAG, "INT level (post): %d (0=asserted, 1=idle), cleared=%d",
             irq_after, static_cast<int>(cleared));
    self->update_irq_stuck_detector_(now_ms, irq_after);

  } else {
    // No IRQ asserted: fall back to previous cache so LVGL stays responsive
    data->state   = self->last_pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->point.x = self->last_x_; data->point.y = self->last_y_;
    //ESP_LOGD(TAG, "cnt=0 (IRQ not asserted)");
  }
}


// Add PCA9554 device on the same bus (optional)
esp_err_t Spd2010LvglGlue::pca9554_init(uint8_t i2c_addr) {
  pca_addr_ = i2c_addr;

  if (!i2c_bus_) return ESP_ERR_INVALID_STATE;

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address  = pca_addr_,
      .scl_speed_hz    = 400000, // same as your bus (400kHz)
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &pca_dev_), TAG, "add_device failed");

  ESP_LOGI(TAG, "PCA9554 added at 0x%02X", pca_addr_);
  return ESP_OK;
}

// Helpers to read/write a PCA9554 register
static esp_err_t pca_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val) {
  uint8_t cmd = reg;
  ESP_RETURN_ON_ERROR(i2c_master_transmit(dev, &cmd, 1, -1), "pca", "tx reg failed");
  return i2c_master_receive(dev, val, 1, -1);
}

static esp_err_t pca_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return i2c_master_transmit(dev, buf, 2, -1);
}

// reg addresses from datasheet: IN=0x00, OUT=0x01, POL=0x02, CONF=0x03
// CONF bit=1 -> input, bit=0 -> output
esp_err_t Spd2010LvglGlue::pca9554_set_pin_mode(uint8_t pin, bool output) {
  if (!pca_dev_) return ESP_ERR_INVALID_STATE;
  if (pin > 7)    return ESP_ERR_INVALID_ARG;

  uint8_t cfg = 0xFF;
  ESP_RETURN_ON_ERROR(pca_read_reg(pca_dev_, 0x03, &cfg), TAG, "read CFG failed");

  if (output) cfg &= ~(1u << pin); else cfg |= (1u << pin);

  return pca_write_reg(pca_dev_, 0x03, cfg);
}

// OUT bit=1 -> drive high, bit=0 -> low (only if configured as output)
esp_err_t Spd2010LvglGlue::pca9554_write_pin(uint8_t pin, bool level) {
  if (!pca_dev_) return ESP_ERR_INVALID_STATE;
  if (pin > 7)    return ESP_ERR_INVALID_ARG;

  uint8_t out = 0x00;
  ESP_RETURN_ON_ERROR(pca_read_reg(pca_dev_, 0x01, &out), TAG, "read OUT failed");

  if (level) out |=  (1u << pin); else out &= ~(1u << pin);

  return pca_write_reg(pca_dev_, 0x01, out);
}

//small stuck‑IRQ detector
void Spd2010LvglGlue::update_irq_stuck_detector_(uint32_t now_ms, int irq_level) {
  if (irq_level != this->irq_last_level_) {
    // Edge detected; reset timer
    this->irq_last_level_   = irq_level;
    this->irq_last_change_ms_ = now_ms;
    ESP_LOGV(TAG, "INT edge -> level=%d at %u ms", irq_level, now_ms);
    return;
  }

  // If line is asserted-low continuously beyond threshold, warn (and optionally clear)
  if (irq_level == 0) {
    uint32_t stuck_ms = now_ms - this->irq_last_change_ms_;
    if (stuck_ms > this->irq_stuck_threshold_ms_) {
      ESP_LOGW(TAG, "IRQ appears stuck low for %u ms; attempting force-clear", stuck_ms);
      if (this->spd_dev_) {
        // Force a Clear-INT
        (void)clear_int_(this->spd_dev_);
        vTaskDelay(pdMS_TO_TICKS(1));
        int irq_after = gpio_get_level((gpio_num_t) this->int_gpio_);
        ESP_LOGW(TAG, "IRQ after force-clear: %d", irq_after);
        // Reset timer if recovered
        if (irq_after != irq_level) {
          this->irq_last_level_    = irq_after;
          this->irq_last_change_ms_= now_ms;
        }
      }
    }
  }
}


// Parse SPD2010 HDP frame into up to N points.
// Returns number of points parsed.

static uint8_t parse_hdp_first_points_(const uint8_t *d, size_t len,
                                       esp_lcd_touch_point_data_t *out, uint8_t max_points) {
  if (!d || len < 10 || max_points == 0) return 0;

  const uint8_t check_id = d[4];
  if (check_id == 0xF6) {
    // gesture-only packet -> no points for LVGL
    return 0;
  }

  const size_t raw_count = (len - 4) / 6;
  const uint8_t count = static_cast<uint8_t>(std::min(raw_count, static_cast<size_t>(max_points)));

  for (uint8_t i = 0; i < count; i++) {
    const size_t off = i * 6;
    const uint16_t x = (static_cast<uint16_t>(d[7 + off] & 0xF0) << 4) | d[5 + off];
    const uint16_t y = (static_cast<uint16_t>(d[7 + off] & 0x0F) << 8) | d[6 + off];
    const uint8_t  w = d[8 + off];

    out[i].x        = x;
    out[i].y        = y;
    out[i].strength = w;
  }
  return count;
}

}  // namespace spd2010_glue
}  // namespace esphome