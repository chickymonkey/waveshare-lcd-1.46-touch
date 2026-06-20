
#pragma once
#include "esphome/core/component.h"
#include "sdkconfig.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_spd2010.h"
#include "lvgl.h"

namespace esphome {
namespace spd2010_glue {

class Spd2010LvglGlue : public Component {
 public:
  // Config setters
 
  // Call this from on_boot (late) to initialize touch
  void begin();
  void set_screen_size(uint16_t w, uint16_t h) { w_ = w; h_ = h; }
  void set_swap_xy(bool b) { swap_xy_ = b; }
  void set_mirror_x(bool b) { mirror_x_ = b; }
  void set_mirror_y(bool b) { mirror_y_ = b; }
  void set_int_gpio(int gpio) { int_gpio_ = gpio; }

  void setup() override;
  void dump_config() override;
  void loop() override;


  float get_setup_priority() const override { 
    // run setup() at the very end (but we’ll keep setup() empty)
    return esphome::setup_priority::LATE; 
  }
  
  //  PCA9554 helpers (for backlight, GPIO expand, etc.) ---
  esp_err_t pca9554_init(uint8_t i2c_addr = 0x20);
  esp_err_t pca9554_set_pin_mode(uint8_t pin, bool output);     // true=output, false=input
  esp_err_t pca9554_write_pin(uint8_t pin, bool level);         // only meaningful if output

 protected:
  void register_lvgl_indev_();
  static void lvgl_read_cb_(lv_indev_t *indev, lv_indev_data_t *data);

  // NEW: stuck-IRQ helper declaration
  void update_irq_stuck_detector_(uint32_t now_ms, int irq_level);

  // Bus / IO / Touch
  i2c_master_bus_handle_t i2c_bus_{nullptr};
  esp_lcd_panel_io_handle_t io_{nullptr};
  esp_lcd_touch_handle_t tp_{nullptr};

  // Optional PCA9554 device handle (new I2C master driver)
  i2c_master_dev_handle_t pca_dev_{nullptr};
  uint8_t pca_addr_{0x20};

  // raw SPD2010 device handle (0x53) on the same I²C bus
  i2c_master_dev_handle_t spd_dev_{nullptr};

  // LVGL indev (LVGL v9: no driver struct, just the indev handle)
  lv_indev_t *indev_{nullptr};

  // Screen & orientation
  uint16_t w_{412}, h_{412};
  bool swap_xy_{false}, mirror_x_{false}, mirror_y_{false};
  int int_gpio_{4};

  // Last state cache
  static constexpr uint32_t kPollMs = 20;
  uint32_t last_poll_ms_{0};
  bool last_pressed_{false};
  uint16_t last_x_{0}, last_y_{0};
  
  // IRQ stuck detection
  uint32_t irq_last_change_ms_{0};
  int      irq_last_level_{1};         // assume idle-high at boot
  uint32_t irq_stuck_threshold_ms_{400}; // e.g., warn if low > 400 ms
  bool indev_registered_{false};
  
  //--- Below for less resource usage.
  // Touch sample cadence (poll from loop(), not inside LVGL cb)
  static constexpr uint32_t kTouchPollMs = 10;
  
  // Upper bound on SPD2010 HDP frame we’ll parse in-place
  static constexpr size_t kMaxHdpBytes = 128;   // clamp, drain remainder if larger

  uint32_t last_touch_poll_ms_{0};
  uint8_t hdp_buf_[kMaxHdpBytes];               // pre-allocated parsing buffer
  //late setup_priority
  bool started_{false};
};

} // namespace spd2010_glue
} // namespace esphome