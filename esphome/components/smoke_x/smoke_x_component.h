#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>
#include <string>

namespace esphome {
namespace smoke_x {

// Forward declarations
class LoRaSX1262;

struct ProbeData {
  bool attached{false};
  float temperature{0.0f};
  bool alarm{false};
  int alarm_max{0};
  int alarm_min{0};
};

class SmokeXComponent : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration methods
  void set_lora_rst_pin(uint8_t pin) { lora_rst_pin_ = pin; }
  void set_lora_busy_pin(uint8_t pin) { lora_busy_pin_ = pin; }
  void set_lora_dio1_pin(uint8_t pin) { lora_dio1_pin_ = pin; }
  void set_sync_frequency(uint32_t freq) { sync_frequency_ = freq; }
  void set_num_probes(uint8_t num) { num_probes_ = num; }
  void set_enabled(bool enabled);
  bool is_enabled() const { return enabled_; }

  // Sensor registration methods
  void register_temperature_sensor(sensor::Sensor *sensor, uint8_t probe_index);
  void register_alarm_max_sensor(sensor::Sensor *sensor, uint8_t probe_index);
  void register_alarm_min_sensor(sensor::Sensor *sensor, uint8_t probe_index);
  void register_device_id_sensor(text_sensor::TextSensor *sensor) { device_id_sensor_ = sensor; }
  void register_units_sensor(text_sensor::TextSensor *sensor) { units_sensor_ = sensor; }
  void register_probe_attached_sensor(binary_sensor::BinarySensor *sensor, uint8_t probe_index);
  void register_probe_alarm_sensor(binary_sensor::BinarySensor *sensor, uint8_t probe_index);
  void register_billows_attached_sensor(binary_sensor::BinarySensor *sensor) { billows_attached_sensor_ = sensor; }
  void register_synced_sensor(binary_sensor::BinarySensor *sensor) { synced_sensor_ = sensor; }

  // Manual sync trigger
  void trigger_sync();

 protected:
  // LoRa pins
  uint8_t lora_rst_pin_{12};
  uint8_t lora_busy_pin_{13};
  uint8_t lora_dio1_pin_{14};
  
  // Configuration
  uint32_t sync_frequency_{920000000};
  uint32_t operating_frequency_{0};
  uint8_t num_probes_{4};
  bool enabled_{false};
  
  // State
  bool configured_{false};
  bool sync_received_{false};
  std::string device_id_;
  std::string units_{"°F"};
  bool billows_attached_{false};
  bool new_alarm_{false};
  ProbeData probes_[4];
  
  // Sensors
  sensor::Sensor *temperature_sensors_[4]{nullptr};
  sensor::Sensor *alarm_max_sensors_[4]{nullptr};
  sensor::Sensor *alarm_min_sensors_[4]{nullptr};
  text_sensor::TextSensor *device_id_sensor_{nullptr};
  text_sensor::TextSensor *units_sensor_{nullptr};
  binary_sensor::BinarySensor *probe_attached_sensors_[4]{nullptr};
  binary_sensor::BinarySensor *probe_alarm_sensors_[4]{nullptr};
  binary_sensor::BinarySensor *billows_attached_sensor_{nullptr};
  binary_sensor::BinarySensor *synced_sensor_{nullptr};
  
  // LoRa instance
  LoRaSX1262 *lora_{nullptr};
  
  // Timing
  uint32_t last_sync_attempt_{0};
  uint32_t last_data_received_{0};
  uint32_t sync_switch_interval_{3300};  // milliseconds
  
  // Methods
  void init_lora();
  void deinit_lora();
  void handle_lora_rx();
  void handle_sync_message(const char *msg, int len);
  void handle_state_message(const char *msg, int len);
  void send_sync_ack();
  void update_sensors();
  void set_frequency(uint32_t freq);
  unsigned int count_commas(const char *msg, int len);
};

// Simple LoRa SX1262 wrapper for ESPHome
class LoRaSX1262 {
 public:
  LoRaSX1262(SPIClass *spi, uint8_t cs_pin, uint8_t rst_pin, uint8_t busy_pin, uint8_t dio1_pin);
  
  bool begin(uint32_t frequency);
  void set_frequency(uint32_t frequency);
  void set_spreading_factor(uint8_t sf);
  void set_bandwidth(uint32_t bw);
  void set_coding_rate(uint8_t cr);
  void set_preamble_length(uint16_t length);
  void set_sync_word(uint8_t sw);
  void set_crc(bool enable);
  void set_tx_power(int8_t power);
  void sleep();
  void standby();
  
  void start_receive();
  bool is_receiving();
  int receive_packet(uint8_t *buffer, size_t size);
  void send_packet(const uint8_t *buffer, size_t size);
  
  int get_rssi() { return last_rssi_; }
  float get_snr() { return last_snr_; }
  
 protected:
  SPIClass *spi_;
  uint8_t cs_pin_;
  uint8_t rst_pin_;
  uint8_t busy_pin_;
  uint8_t dio1_pin_;
  
  int last_rssi_{0};
  float last_snr_{0.0f};
  
  void reset();
  void wait_busy();
  void write_command(uint8_t cmd, const uint8_t *data, size_t len);
  void read_command(uint8_t cmd, uint8_t *data, size_t len);
  void write_register(uint16_t address, const uint8_t *data, size_t len);
  void read_register(uint16_t address, uint8_t *data, size_t len);
};

}  // namespace smoke_x
}  // namespace esphome
