#include "smoke_x_component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#include <cstdlib>

namespace esphome {
namespace smoke_x {

static const char *TAG = "smoke_x";

// SX1262 Commands
static const uint8_t SX126X_CMD_SET_SLEEP = 0x84;
static const uint8_t SX126X_CMD_SET_STANDBY = 0x80;
static const uint8_t SX126X_CMD_SET_FS = 0xC1;
static const uint8_t SX126X_CMD_SET_TX = 0x83;
static const uint8_t SX126X_CMD_SET_RX = 0x82;
static const uint8_t SX126X_CMD_SET_RF_FREQUENCY = 0x86;
static const uint8_t SX126X_CMD_SET_PACKET_TYPE = 0x8A;
static const uint8_t SX126X_CMD_GET_PACKET_TYPE = 0x11;
static const uint8_t SX126X_CMD_SET_TX_PARAMS = 0x8E;
static const uint8_t SX126X_CMD_SET_MODULATION_PARAMS = 0x8B;
static const uint8_t SX126X_CMD_SET_PACKET_PARAMS = 0x8C;
static const uint8_t SX126X_CMD_GET_IRQ_STATUS = 0x12;
static const uint8_t SX126X_CMD_CLR_IRQ_STATUS = 0x02;
static const uint8_t SX126X_CMD_SET_DIO_IRQ_PARAMS = 0x08;
static const uint8_t SX126X_CMD_GET_RX_BUFFER_STATUS = 0x13;
static const uint8_t SX126X_CMD_READ_BUFFER = 0x1E;
static const uint8_t SX126X_CMD_WRITE_BUFFER = 0x0E;
static const uint8_t SX126X_CMD_GET_PACKET_STATUS = 0x14;
static const uint8_t SX126X_CMD_SET_REGULATOR_MODE = 0x96;
static const uint8_t SX126X_CMD_SET_DIO3_AS_TCXO_CTRL = 0x97;
static const uint8_t SX126X_CMD_CALIBRATE = 0x89;
static const uint8_t SX126X_CMD_SET_PA_CONFIG = 0x95;
static const uint8_t SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0;

// Constants
static const uint32_t SMOKE_X2_SYNC_FREQ = 920000000;
static const uint32_t SMOKE_X4_SYNC_FREQ = 915000000;
static const uint32_t SMOKE_X_RF_MIN = 902000000;
static const uint32_t SMOKE_X_RF_MAX = 928000000;
static const int NUM_COMMAS_SYNC_MSG = 6;
static const int NUM_COMMAS_X2_STATE_MSG = 16;
static const int NUM_COMMAS_X4_STATE_MSG = 26;

// ==================== SmokeXComponent Implementation ====================

void SmokeXComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Smoke X Component...");
  
  // Initialize LoRa if enabled
  if (enabled_) {
    init_lora();
  } else {
    ESP_LOGI(TAG, "Smoke X is disabled at boot");
  }
  
  // Set initial state
  configured_ = false;
  sync_received_ = false;
  device_id_.clear();
  
  // Initialize probe data
  for (int i = 0; i < 4; i++) {
    probes_[i] = ProbeData();
  }
  
  // Update sensors with initial state
  update_sensors();
  
  ESP_LOGCONFIG(TAG, "Smoke X Component setup complete");
}

void SmokeXComponent::loop() {
  uint32_t now = millis();
  
  // Handle LoRa reception when enabled
  if (enabled_) {
    handle_lora_rx();
  }
  
  // If not configured, switch between sync frequencies
  if (enabled_ && !configured_ && !sync_received_) {
    if (now - last_sync_attempt_ > sync_switch_interval_) {
      last_sync_attempt_ = now;
      
      // Switch frequency
      uint32_t new_freq = (sync_frequency_ == SMOKE_X2_SYNC_FREQ) ? SMOKE_X4_SYNC_FREQ : SMOKE_X2_SYNC_FREQ;
      sync_frequency_ = new_freq;
      set_frequency(new_freq);
      
      ESP_LOGD(TAG, "Switching sync frequency to %u MHz", new_freq / 1000000);
    }
  }
  
  // Check for timeout if configured
  if (enabled_ && configured_ && (now - last_data_received_ > 30000)) {
    ESP_LOGW(TAG, "No data received for 30 seconds");
  }
}

void SmokeXComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Smoke X:");
  ESP_LOGCONFIG(TAG, "  RST Pin: %u", lora_rst_pin_);
  ESP_LOGCONFIG(TAG, "  BUSY Pin: %u", lora_busy_pin_);
  ESP_LOGCONFIG(TAG, "  DIO1 Pin: %u", lora_dio1_pin_);
  ESP_LOGCONFIG(TAG, "  Sync Frequency: %u MHz", sync_frequency_ / 1000000);
  ESP_LOGCONFIG(TAG, "  Num Probes: %u", num_probes_);
  
  if (configured_) {
    ESP_LOGCONFIG(TAG, "  Device ID: %s", device_id_.c_str());
    ESP_LOGCONFIG(TAG, "  Operating Frequency: %u MHz", operating_frequency_ / 1000000);
  } else {
    ESP_LOGCONFIG(TAG, "  Status: Not configured (waiting for sync)");
  }
}

void SmokeXComponent::init_lora() {
  ESP_LOGD(TAG, "Initializing LoRa SX1262...");
  
  // Create LoRa instance
  lora_ = new LoRaSX1262(
    &this->spi_,
    this->cs_->get_pin(),
    lora_rst_pin_,
    lora_busy_pin_,
    lora_dio1_pin_
  );
  
  // Initialize with sync frequency
  if (!lora_->begin(sync_frequency_)) {
    ESP_LOGE(TAG, "Failed to initialize LoRa");
    return;
  }
  
  // Configure LoRa parameters for Smoke X
  lora_->set_spreading_factor(9);      // SF9
  lora_->set_bandwidth(125000);        // 125 kHz
  lora_->set_coding_rate(5);           // 4/5
  lora_->set_preamble_length(10);      // 10 symbols
  lora_->set_sync_word(0x12);          // Private network
  lora_->set_crc(true);
  lora_->set_tx_power(22);             // 22 dBm
  
  // Start receiving
  lora_->start_receive();
  
  ESP_LOGD(TAG, "LoRa initialized successfully");
}

void SmokeXComponent::deinit_lora() {
  if (!lora_) return;
  ESP_LOGD(TAG, "Deinitializing LoRa SX1262 (sleep)");
  lora_->sleep();
}

void SmokeXComponent::handle_lora_rx() {
  if (!lora_ || !lora_->is_receiving()) {
    return;
  }
  
  uint8_t buffer[255];
  int len = lora_->receive_packet(buffer, sizeof(buffer));
  
  if (len > 0) {
    buffer[len] = '\0';  // Null terminate
    
    ESP_LOGD(TAG, "Received packet (%d bytes): %s", len, buffer);
    ESP_LOGD(TAG, "RSSI: %d dBm, SNR: %.1f dB", lora_->get_rssi(), lora_->get_snr());
    
    // Process message based on comma count
    unsigned int comma_count = count_commas((char *)buffer, len);
    
    switch (comma_count) {
      case NUM_COMMAS_SYNC_MSG:
        if (!configured_ && !sync_received_) {
          handle_sync_message((char *)buffer, len);
        } else {
          ESP_LOGW(TAG, "Received unexpected sync message (ignored)");
        }
        break;
        
      case NUM_COMMAS_X2_STATE_MSG:
        if (sync_received_) {
          if (!configured_) {
            num_probes_ = 2;
            configured_ = true;
            ESP_LOGI(TAG, "Detected Smoke X2 (2 probes)");
          }
          handle_state_message((char *)buffer, len);
        }
        break;
        
      case NUM_COMMAS_X4_STATE_MSG:
        if (sync_received_) {
          if (!configured_) {
            num_probes_ = 4;
            configured_ = true;
            ESP_LOGI(TAG, "Detected Smoke X4 (4 probes)");
          }
          handle_state_message((char *)buffer, len);
        }
        break;
        
      default:
        ESP_LOGW(TAG, "Received unrecognized message format (%u commas)", comma_count);
        break;
    }
    
    // Restart receiving
    lora_->start_receive();
  }
}

void SmokeXComponent::handle_sync_message(const char *msg, int len) {
  ESP_LOGI(TAG, "Processing sync message: %s", msg);
  
  // Parse sync message: "020001,|dhHWl,160,32,69,54,"
  char *msg_copy = strdup(msg);
  char *token = strtok(msg_copy, ",");
  
  if (!token) {
    free(msg_copy);
    return;
  }
  
  // Skip first field
  token = strtok(nullptr, ",");
  if (!token) {
    free(msg_copy);
    return;
  }
  
  // Get device ID
  device_id_ = std::string(token);
  ESP_LOGI(TAG, "Device ID: %s", device_id_.c_str());
  
  // Parse frequency bytes
  uint8_t freq_bytes[4];
  for (int i = 0; i < 4; i++) {
    token = strtok(nullptr, ",");
    if (!token) {
      free(msg_copy);
      return;
    }
    freq_bytes[i] = (uint8_t)atoi(token);
  }
  
  free(msg_copy);
  
  // Reconstruct frequency
  operating_frequency_ = *(uint32_t *)freq_bytes;
  
  if (operating_frequency_ >= SMOKE_X_RF_MIN && operating_frequency_ <= SMOKE_X_RF_MAX) {
    sync_received_ = true;
    set_frequency(operating_frequency_);
    send_sync_ack();
    
    ESP_LOGI(TAG, "Sync successful, operating frequency: %u MHz", operating_frequency_ / 1000000);
  } else {
    ESP_LOGE(TAG, "Invalid frequency in sync message: %u", operating_frequency_);
  }
}

void SmokeXComponent::handle_state_message(const char *msg, int len) {
  last_data_received_ = millis();
  
  // Parse state message
  char *msg_copy = strdup(msg);
  char *token = strtok(msg_copy, ",");
  
  // Skip device ID
  token = strtok(nullptr, ",");
  // Skip unknown field
  token = strtok(nullptr, ",");
  
  // Get units
  token = strtok(nullptr, ",");
  if (token) {
    units_ = (atoi(token) == 1) ? "°F" : "°C";
  }
  
  // Get new alarm status
  token = strtok(nullptr, ",");
  if (token) {
    new_alarm_ = (atoi(token) != 0);
  }
  
  // Parse probe data
  for (unsigned int i = 0; i < num_probes_; i++) {
    // Probe attached
    token = strtok(nullptr, ",");
    if (token) {
      probes_[i].attached = (atoi(token) != 3);
    }
    
    // Temperature
    token = strtok(nullptr, ",");
    if (token) {
      probes_[i].temperature = atof(token) / 10.0f;
    }
    
    // Alarm
    token = strtok(nullptr, ",");
    if (token) {
      probes_[i].alarm = (atoi(token) != 0);
    }
    
    // Alarm max
    token = strtok(nullptr, ",");
    if (token) {
      probes_[i].alarm_max = atoi(token);
    }
    
    // Alarm min
    token = strtok(nullptr, ",");
    if (token) {
      probes_[i].alarm_min = atoi(token);
    }
  }
  
  // Billows attached
  token = strtok(nullptr, ",");
  if (token) {
    billows_attached_ = (atoi(token) != 0);
  }
  
  free(msg_copy);
  
  // Update all sensors
  update_sensors();
  
  ESP_LOGD(TAG, "State updated - Units: %s, Billows: %s", 
           units_.c_str(), billows_attached_ ? "Yes" : "No");
}

void SmokeXComponent::send_sync_ack() {
  char response[32];
  snprintf(response, sizeof(response), "%s,SUCCESS,", device_id_.c_str());
  
  ESP_LOGI(TAG, "Sending sync acknowledgement: %s", response);
  
  lora_->send_packet((uint8_t *)response, strlen(response));
  
  // Resume receiving after transmission
  lora_->start_receive();
}

void SmokeXComponent::update_sensors() {
  // Update device ID
  if (device_id_sensor_ && !device_id_.empty()) {
    device_id_sensor_->publish_state(device_id_);
  }
  
  // Update units
  if (units_sensor_) {
    units_sensor_->publish_state(units_);
  }
  
  // Update synced status
  if (synced_sensor_) {
    synced_sensor_->publish_state(configured_);
  }
  
  // Update billows status
  if (billows_attached_sensor_) {
    billows_attached_sensor_->publish_state(billows_attached_);
  }
  
  // Update probe sensors
  for (int i = 0; i < 4; i++) {
    if (temperature_sensors_[i] && probes_[i].attached) {
      temperature_sensors_[i]->publish_state(probes_[i].temperature);
    }
    
    if (alarm_max_sensors_[i] && probes_[i].attached) {
      alarm_max_sensors_[i]->publish_state(probes_[i].alarm_max);
    }
    
    if (alarm_min_sensors_[i] && probes_[i].attached) {
      alarm_min_sensors_[i]->publish_state(probes_[i].alarm_min);
    }
    
    if (probe_attached_sensors_[i]) {
      probe_attached_sensors_[i]->publish_state(probes_[i].attached);
    }
    
    if (probe_alarm_sensors_[i]) {
      probe_alarm_sensors_[i]->publish_state(probes_[i].alarm);
    }
  }
}

void SmokeXComponent::set_frequency(uint32_t freq) {
  if (lora_) {
    lora_->set_frequency(freq);
    lora_->start_receive();
  }
}

unsigned int SmokeXComponent::count_commas(const char *msg, int len) {
  unsigned int count = 0;
  for (int i = 0; i < len; i++) {
    if (msg[i] == ',') count++;
  }
  return count;
}

void SmokeXComponent::trigger_sync() {
  ESP_LOGI(TAG, "Manual sync triggered");
  configured_ = false;
  sync_received_ = false;
  device_id_.clear();
  operating_frequency_ = 0;
  sync_frequency_ = SMOKE_X2_SYNC_FREQ;
  if (enabled_) {
    set_frequency(sync_frequency_);
  }
  last_sync_attempt_ = millis();
}

void SmokeXComponent::set_enabled(bool enabled) {
  if (enabled_ == enabled) return;
  enabled_ = enabled;
  if (enabled_) {
    ESP_LOGI(TAG, "Smoke X enabled");
    init_lora();
  } else {
    ESP_LOGI(TAG, "Smoke X disabled");
    deinit_lora();
  }
}

// Sensor registration methods
void SmokeXComponent::register_temperature_sensor(sensor::Sensor *sensor, uint8_t probe_index) {
  if (probe_index < 4) {
    temperature_sensors_[probe_index] = sensor;
  }
}

void SmokeXComponent::register_alarm_max_sensor(sensor::Sensor *sensor, uint8_t probe_index) {
  if (probe_index < 4) {
    alarm_max_sensors_[probe_index] = sensor;
  }
}

void SmokeXComponent::register_alarm_min_sensor(sensor::Sensor *sensor, uint8_t probe_index) {
  if (probe_index < 4) {
    alarm_min_sensors_[probe_index] = sensor;
  }
}

void SmokeXComponent::register_probe_attached_sensor(binary_sensor::BinarySensor *sensor, uint8_t probe_index) {
  if (probe_index < 4) {
    probe_attached_sensors_[probe_index] = sensor;
  }
}

void SmokeXComponent::register_probe_alarm_sensor(binary_sensor::BinarySensor *sensor, uint8_t probe_index) {
  if (probe_index < 4) {
    probe_alarm_sensors_[probe_index] = sensor;
  }
}

// ==================== LoRaSX1262 Implementation ====================

LoRaSX1262::LoRaSX1262(SPIClass *spi, uint8_t cs_pin, uint8_t rst_pin, uint8_t busy_pin, uint8_t dio1_pin)
    : spi_(spi), cs_pin_(cs_pin), rst_pin_(rst_pin), busy_pin_(busy_pin), dio1_pin_(dio1_pin) {
  
  pinMode(cs_pin_, OUTPUT);
  digitalWrite(cs_pin_, HIGH);
  
  pinMode(rst_pin_, OUTPUT);
  pinMode(busy_pin_, INPUT);
  pinMode(dio1_pin_, INPUT);
}

bool LoRaSX1262::begin(uint32_t frequency) {
  // Reset the chip
  reset();
  
  // Set to standby
  uint8_t standby_config = 0x00;  // STDBY_RC
  write_command(SX126X_CMD_SET_STANDBY, &standby_config, 1);
  delay(1);
  
  // Set regulator mode to LDO
  uint8_t reg_mode = 0x01;
  write_command(SX126X_CMD_SET_REGULATOR_MODE, &reg_mode, 1);
  
  // Set TCXO voltage (3.3V, 5ms timeout)
  uint8_t tcxo_params[4] = {0x07, 0x00, 0x01, 0x40};  // 3.3V, 5ms
  write_command(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, tcxo_params, 4);
  delay(10);
  
  // Calibrate
  uint8_t calib_param = 0x7F;  // Calibrate all blocks
  write_command(SX126X_CMD_CALIBRATE, &calib_param, 1);
  delay(5);
  
  // Set packet type to LoRa
  uint8_t packet_type = 0x01;  // PACKET_TYPE_LORA
  write_command(SX126X_CMD_SET_PACKET_TYPE, &packet_type, 1);
  
  // Set frequency
  set_frequency(frequency);
  
  // Set PA config
  uint8_t pa_config[4] = {0x04, 0x07, 0x00, 0x01};  // +22dBm, PA_LUT
  write_command(SX126X_CMD_SET_PA_CONFIG, pa_config, 4);
  
  return true;
}

void LoRaSX1262::set_frequency(uint32_t frequency) {
  uint32_t freq_reg = (uint32_t)((double)frequency / (double)32000000 * (double)(1 << 25));
  uint8_t freq_params[4] = {
    (uint8_t)((freq_reg >> 24) & 0xFF),
    (uint8_t)((freq_reg >> 16) & 0xFF),
    (uint8_t)((freq_reg >> 8) & 0xFF),
    (uint8_t)(freq_reg & 0xFF)
  };
  write_command(SX126X_CMD_SET_RF_FREQUENCY, freq_params, 4);
}

void LoRaSX1262::set_spreading_factor(uint8_t sf) {
  // This is set along with bandwidth in set_modulation_params
}

void LoRaSX1262::set_bandwidth(uint32_t bw) {
  // Convert bandwidth to register value
  uint8_t bw_reg;
  if (bw <= 7800) bw_reg = 0x00;        // 7.8 kHz
  else if (bw <= 10400) bw_reg = 0x08;  // 10.4 kHz
  else if (bw <= 15600) bw_reg = 0x01;  // 15.6 kHz
  else if (bw <= 20800) bw_reg = 0x09;  // 20.8 kHz
  else if (bw <= 31250) bw_reg = 0x02;  // 31.25 kHz
  else if (bw <= 41700) bw_reg = 0x0A;  // 41.7 kHz
  else if (bw <= 62500) bw_reg = 0x03;  // 62.5 kHz
  else if (bw <= 125000) bw_reg = 0x04; // 125 kHz
  else if (bw <= 250000) bw_reg = 0x05; // 250 kHz
  else bw_reg = 0x06;                   // 500 kHz
  
  // Set modulation parameters (SF=9, BW, CR=4/5, LDRO=auto)
  uint8_t mod_params[8] = {
    0x09,     // SF9
    bw_reg,   // Bandwidth
    0x01,     // CR 4/5
    0x00,     // Low data rate optimize OFF
    0x00, 0x00, 0x00, 0x00  // Reserved
  };
  write_command(SX126X_CMD_SET_MODULATION_PARAMS, mod_params, 8);
}

void LoRaSX1262::set_coding_rate(uint8_t cr) {
  // Coding rate is set in modulation parameters
  // This would require reading current params and updating
}

void LoRaSX1262::set_preamble_length(uint16_t length) {
  // Set in packet parameters
}

void LoRaSX1262::set_sync_word(uint8_t sw) {
  uint8_t sync_word_bytes[2] = {(uint8_t)(sw >> 4), (uint8_t)(sw << 4)};
  write_register(0x0740, sync_word_bytes, 2);
}

void LoRaSX1262::set_crc(bool enable) {
  // CRC is set in packet parameters
}

void LoRaSX1262::set_tx_power(int8_t power) {
  uint8_t tx_params[2] = {(uint8_t)power, 0x04};  // Power, ramp time 200us
  write_command(SX126X_CMD_SET_TX_PARAMS, tx_params, 2);
}

void LoRaSX1262::sleep() {
  uint8_t sleep_params = 0x04; // warm start, retain RTC and registers
  write_command(SX126X_CMD_SET_SLEEP, &sleep_params, 1);
}

void LoRaSX1262::standby() {
  uint8_t standby_config = 0x00;  // STDBY_RC
  write_command(SX126X_CMD_SET_STANDBY, &standby_config, 1);
}

void LoRaSX1262::start_receive() {
  // Clear IRQ status
  uint8_t irq_mask[2] = {0xFF, 0xFF};
  write_command(SX126X_CMD_CLR_IRQ_STATUS, irq_mask, 2);
  
  // Set DIO IRQ parameters
  uint8_t dio_params[8] = {
    0x00, 0x02,  // IRQ mask: RxDone
    0x00, 0x02,  // DIO1 mask: RxDone
    0x00, 0x00,  // DIO2 mask: none
    0x00, 0x00   // DIO3 mask: none
  };
  write_command(SX126X_CMD_SET_DIO_IRQ_PARAMS, dio_params, 8);
  
  // Set packet parameters
  uint8_t packet_params[9] = {
    0x00, 0x0A,  // Preamble length: 10
    0x00,        // Header type: explicit
    0xFF,        // Payload length: 255
    0x01,        // CRC: enabled
    0x00,        // Invert IQ: standard
    0x00, 0x00, 0x00  // Reserved
  };
  write_command(SX126X_CMD_SET_PACKET_PARAMS, packet_params, 9);
  
  // Start receiving (continuous mode)
  uint8_t rx_params[3] = {0xFF, 0xFF, 0xFF};  // Timeout disabled
  write_command(SX126X_CMD_SET_RX, rx_params, 3);
}

bool LoRaSX1262::is_receiving() {
  // Check if DIO1 is high (packet received)
  return digitalRead(dio1_pin_) == HIGH;
}

int LoRaSX1262::receive_packet(uint8_t *buffer, size_t size) {
  // Get IRQ status
  uint8_t irq_status[2];
  read_command(SX126X_CMD_GET_IRQ_STATUS, irq_status, 2);
  
  // Check if RxDone flag is set
  if (!(irq_status[1] & 0x02)) {
    return 0;  // No packet received
  }
  
  // Get RX buffer status
  uint8_t rx_status[2];
  read_command(SX126X_CMD_GET_RX_BUFFER_STATUS, rx_status, 2);
  
  uint8_t payload_length = rx_status[0];
  uint8_t rx_start_buffer_pointer = rx_status[1];
  
  if (payload_length > size) {
    payload_length = size;
  }
  
  // Read buffer
  uint8_t cmd[2] = {rx_start_buffer_pointer, 0x00};
  digitalWrite(cs_pin_, LOW);
  spi_->transfer(SX126X_CMD_READ_BUFFER);
  spi_->transfer(cmd[0]);
  spi_->transfer(cmd[1]);
  for (int i = 0; i < payload_length; i++) {
    buffer[i] = spi_->transfer(0x00);
  }
  digitalWrite(cs_pin_, HIGH);
  
  // Get packet status (RSSI, SNR)
  uint8_t packet_status[3];
  read_command(SX126X_CMD_GET_PACKET_STATUS, packet_status, 3);
  
  last_rssi_ = -packet_status[0] / 2;
  last_snr_ = (int8_t)packet_status[1] / 4.0f;
  
  // Clear IRQ status
  uint8_t irq_mask[2] = {0xFF, 0xFF};
  write_command(SX126X_CMD_CLR_IRQ_STATUS, irq_mask, 2);
  
  return payload_length;
}

void LoRaSX1262::send_packet(const uint8_t *buffer, size_t size) {
  // Set to standby
  uint8_t standby_config = 0x00;
  write_command(SX126X_CMD_SET_STANDBY, &standby_config, 1);
  
  // Write buffer
  digitalWrite(cs_pin_, LOW);
  spi_->transfer(SX126X_CMD_WRITE_BUFFER);
  spi_->transfer(0x00);  // offset
  for (size_t i = 0; i < size; i++) {
    spi_->transfer(buffer[i]);
  }
  digitalWrite(cs_pin_, HIGH);
  
  // Set packet params with actual payload length
  uint8_t packet_params[9] = {
    0x00, 0x0A,  // Preamble length: 10
    0x00,        // Header type: explicit
    (uint8_t)size, // Payload length
    0x01,        // CRC: enabled
    0x00,        // Invert IQ: standard
    0x00, 0x00, 0x00  // Reserved
  };
  write_command(SX126X_CMD_SET_PACKET_PARAMS, packet_params, 9);
  
  // Clear and set IRQ for TxDone
  uint8_t irq_mask[2] = {0xFF, 0xFF};
  write_command(SX126X_CMD_CLR_IRQ_STATUS, irq_mask, 2);
  
  uint8_t dio_params[8] = {
    0x00, 0x01,  // IRQ mask: TxDone
    0x00, 0x01,  // DIO1 mask: TxDone
    0x00, 0x00,  // DIO2 mask: none
    0x00, 0x00   // DIO3 mask: none
  };
  write_command(SX126X_CMD_SET_DIO_IRQ_PARAMS, dio_params, 8);
  
  // Start transmission
  uint8_t tx_params[3] = {0x00, 0x00, 0x00};  // No timeout
  write_command(SX126X_CMD_SET_TX, tx_params, 3);
  
  // Wait for transmission to complete
  uint32_t start = millis();
  while (digitalRead(dio1_pin_) == LOW) {
    if (millis() - start > 1000) {
      ESP_LOGW(TAG, "TX timeout");
      break;
    }
    delay(1);
  }
  
  // Clear IRQ
  write_command(SX126X_CMD_CLR_IRQ_STATUS, irq_mask, 2);
}

void LoRaSX1262::reset() {
  digitalWrite(rst_pin_, LOW);
  delay(10);
  digitalWrite(rst_pin_, HIGH);
  delay(20);
}

void LoRaSX1262::wait_busy() {
  while (digitalRead(busy_pin_) == HIGH) {
    delayMicroseconds(10);
  }
}

void LoRaSX1262::write_command(uint8_t cmd, const uint8_t *data, size_t len) {
  wait_busy();
  
  digitalWrite(cs_pin_, LOW);
  spi_->transfer(cmd);
  for (size_t i = 0; i < len; i++) {
    spi_->transfer(data[i]);
  }
  digitalWrite(cs_pin_, HIGH);
}

void LoRaSX1262::read_command(uint8_t cmd, uint8_t *data, size_t len) {
  wait_busy();
  
  digitalWrite(cs_pin_, LOW);
  spi_->transfer(cmd);
  spi_->transfer(0x00);  // NOP
  for (size_t i = 0; i < len; i++) {
    data[i] = spi_->transfer(0x00);
  }
  digitalWrite(cs_pin_, HIGH);
}

void LoRaSX1262::write_register(uint16_t address, const uint8_t *data, size_t len) {
  wait_busy();
  
  digitalWrite(cs_pin_, LOW);
  spi_->transfer(0x0D);  // Write register command
  spi_->transfer((address >> 8) & 0xFF);
  spi_->transfer(address & 0xFF);
  for (size_t i = 0; i < len; i++) {
    spi_->transfer(data[i]);
  }
  digitalWrite(cs_pin_, HIGH);
}

void LoRaSX1262::read_register(uint16_t address, uint8_t *data, size_t len) {
  wait_busy();
  
  digitalWrite(cs_pin_, LOW);
  spi_->transfer(0x1D);  // Read register command
  spi_->transfer((address >> 8) & 0xFF);
  spi_->transfer(address & 0xFF);
  spi_->transfer(0x00);  // NOP
  for (size_t i = 0; i < len; i++) {
    data[i] = spi_->transfer(0x00);
  }
  digitalWrite(cs_pin_, HIGH);
}

}  // namespace smoke_x
}  // namespace esphome
