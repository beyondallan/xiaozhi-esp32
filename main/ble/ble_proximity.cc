#include "ble_proximity.h"

#include <cmath>
#include <cstring>
#include <algorithm>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"

static const char* TAG = "BleProximity";

// BLE Advertising Data constants
// Advertisement format: Company ID (2B) + Device Type (1B) + Device UUID (16B) = 19 bytes
// Total with AD header: 1 (length) + 1 (type) + 19 (data) = 21 bytes
static constexpr int kManufacturerDataLen = 19;
static constexpr uint8_t kAdvTypeManufacturer = 0xFF;  // Manufacturer Specific Data AD type

// BLE Advertisement parameters
static esp_ble_adv_params_t s_adv_params = {
    .adv_int_min = 0x20,      // 20ms
    .adv_int_max = 0x40,      // 40ms
    .adv_type = ADV_TYPE_NONCONN_IND,  // Non-connectable undirected advertising
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr = {0},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Scan parameters
static esp_ble_scan_params_t s_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,    // 50ms
    .scan_window = 0x30,      // 30ms
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};

BleProximity::BleProximity() = default;
BleProximity::~BleProximity() { Stop(); }

BleProximity& BleProximity::GetInstance() {
    static BleProximity instance;
    return instance;
}

void BleProximity::RegisterProximityCallback(ProximityCallback callback) {
    callback_ = std::move(callback);
}

esp_err_t BleProximity::Start(const Config& config) {
    if (running_) {
        ESP_LOGW(TAG, "Already running, stop first");
        return ESP_OK;
    }

    if (config.device_uuid.empty() || config.device_uuid.length() != 32) {
        ESP_LOGE(TAG, "Invalid device UUID: must be 32 hex chars");
        return ESP_ERR_INVALID_ARG;
    }

    config_ = config;

    // Initialize BLE controller if not already done
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means already initialized
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GAP callback
    ret = esp_ble_gap_register_callback(GapEventCallback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure scan parameters
    ret = esp_ble_gap_set_scan_params(&s_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Set scan params failed: %s", esp_err_to_name(ret));
        return ret;
    }

    running_ = true;

    // Start advertising and scanning (triggered after GAP ready event)
    ret = StartAdvertising();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
    }

    ret = StartScanning();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Start scanning failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "BLE proximity started: device_uuid=%.8s... type=%d company_id=0x%04X",
             config_.device_uuid.c_str(), config_.device_type, config_.company_id);

    return ESP_OK;
}

void BleProximity::Stop() {
    if (!running_) return;

    running_ = false;

    esp_ble_gap_stop_advertising();
    esp_ble_gap_stop_scanning();

    std::lock_guard<std::mutex> lock(devices_mutex_);
    tracked_devices_.clear();

    ESP_LOGI(TAG, "BLE proximity stopped");
}

esp_err_t BleProximity::StartAdvertising() {
    // Build manufacturer specific data:
    // [Company ID (2B LE)] [Device Type (1B)] [Device UUID (16B)]
    uint8_t mfr_data[kManufacturerDataLen];

    // Company ID (little-endian)
    mfr_data[0] = config_.company_id & 0xFF;
    mfr_data[1] = (config_.company_id >> 8) & 0xFF;

    // Device type
    mfr_data[2] = config_.device_type;

    // Device UUID (16 bytes from hex string)
    for (int i = 0; i < 16; i++) {
        char hex[3] = {config_.device_uuid[i * 2], config_.device_uuid[i * 2 + 1], '\0'};
        mfr_data[3 + i] = (uint8_t)strtol(hex, nullptr, 16);
    }

    // Build advertising data
    esp_ble_adv_data_t adv_data = {};
    adv_data.set_scan_rsp = false;
    adv_data.include_name = false;
    adv_data.include_txpower = false;
    adv_data.manufacturer_len = kManufacturerDataLen;
    adv_data.p_manufacturer_data = mfr_data;

    esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret != ESP_OK) {
        return ret;
    }

    // Start advertising after config data callback
    return ESP_OK;
}

esp_err_t BleProximity::StartScanning() {
    // Stop first, then restart with new parameters
    esp_ble_gap_stop_scanning();

    // Start scanning — 0 means continuous
    return esp_ble_gap_start_scanning(0);
}

void BleProximity::GapEventCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    auto& instance = GetInstance();

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        if (instance.running_) {
            esp_ble_gap_start_advertising(&s_adv_params);
            ESP_LOGI(TAG, "Advertising started");
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        if (!instance.running_) break;
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) break;

        // Check for manufacturer specific data
        uint8_t* adv_data = param->scan_rst.ble_adv;
        int adv_data_len = param->scan_rst.adv_data_len;

        // Parse AD structures to find manufacturer specific data
        int offset = 0;
        while (offset < adv_data_len) {
            int length = adv_data[offset];
            if (length == 0) break;
            offset++;

            if (offset >= adv_data_len) break;
            uint8_t ad_type = adv_data[offset];
            offset++;

            if (ad_type == kAdvTypeManufacturer && length >= 2) {
                int mfr_data_len = length - 1;  // minus the type byte
                instance.ProcessScanResult(
                    "",  // We'll extract device UUID from manufacturer data
                    &adv_data[offset],
                    mfr_data_len,
                    param->scan_rst.rssi
                );
            }
            offset += length - 1;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan stopped");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising stopped");
        break;

    default:
        break;
    }
}

void BleProximity::ProcessScanResult(const std::string& device_addr,
                                     const uint8_t* manufacturer_data,
                                     int manufacturer_data_len,
                                     int rssi) {
    // Validate manufacturer data length
    if (manufacturer_data_len < kManufacturerDataLen) return;

    // Parse Company ID (little-endian)
    uint16_t company_id = manufacturer_data[0] | (manufacturer_data[1] << 8);
    if (company_id != config_.company_id) return;

    // Parse Device Type
    uint8_t device_type = manufacturer_data[2];
    if (device_type != config_.device_type) return;

    // Extract Device UUID (16 bytes → 32 hex chars)
    char uuid_hex[33];
    for (int i = 0; i < 16; i++) {
        snprintf(&uuid_hex[i * 2], 3, "%02x", manufacturer_data[3 + i]);
    }
    uuid_hex[32] = '\0';
    std::string device_uuid(uuid_hex);

    // Don't track ourselves
    if (device_uuid == config_.device_uuid) return;

    float distance = EstimateDistance(rssi);
    int64_t now = GetCurrentTimeMs();

    ESP_LOGD(TAG, "Detected device: uuid=%.8s... rssi=%d dist=%.1fm",
             device_uuid.c_str(), rssi, distance);

    std::lock_guard<std::mutex> lock(devices_mutex_);

    auto it = tracked_devices_.find(device_uuid);
    if (it == tracked_devices_.end()) {
        TrackedDevice device;
        device.device_uuid = device_uuid;
        device.last_rssi = rssi;
        device.last_distance = distance;

        // Initialize debounce state based on current RSSI
        if (rssi >= config_.enter_rssi_threshold) {
            device.first_seen_near_ms = now;
        } else if (rssi <= config_.leave_rssi_threshold) {
            device.first_seen_far_ms = now;
        }

        tracked_devices_[device_uuid] = device;
        it = tracked_devices_.find(device_uuid);
    } else {
        it->second.last_rssi = rssi;
        it->second.last_distance = distance;
    }

    CheckDebounce(it->second);
}

void BleProximity::CheckDebounce(TrackedDevice& device) {
    int64_t now = GetCurrentTimeMs();

    if (!device.is_near) {
        // Device is currently "far" — check if it should transition to "near"
        if (device.last_rssi >= config_.enter_rssi_threshold) {
            if (device.first_seen_near_ms == 0) {
                device.first_seen_near_ms = now;
            } else if (now - device.first_seen_near_ms >= config_.enter_duration_ms) {
                // Debounce passed — transition to near
                device.is_near = true;
                device.first_seen_far_ms = 0;

                // Report enter event (if not already reported)
                if (!device.enter_reported) {
                    device.enter_reported = true;
                    device.leave_reported = false;  // Reset leave flag
                    ReportEvent(device, true);
                }
            }
        } else if (device.last_rssi < config_.leave_rssi_threshold) {
            // RSSI dropped back below threshold before debounce completed
            device.first_seen_near_ms = 0;
        }
    } else {
        // Device is currently "near" — check if it should transition to "far"
        if (device.last_rssi <= config_.leave_rssi_threshold) {
            if (device.first_seen_far_ms == 0) {
                device.first_seen_far_ms = now;
            } else if (now - device.first_seen_far_ms >= config_.leave_duration_ms) {
                // Debounce passed — transition to far
                device.is_near = false;
                device.first_seen_near_ms = 0;

                // Report leave event (if not already reported)
                if (!device.leave_reported) {
                    device.leave_reported = true;
                    device.enter_reported = false;  // Reset enter flag
                    ReportEvent(device, false);
                }
            }
        } else if (device.last_rssi > config_.enter_rssi_threshold) {
            // RSSI went back above threshold before debounce completed
            device.first_seen_far_ms = 0;
        }
    }
}

void BleProximity::ReportEvent(TrackedDevice& device, bool is_enter) {
    // Rate limit check
    if (!CheckRateLimit()) {
        ESP_LOGW(TAG, "Rate limit: dropping %s event for device %.8s...",
                 is_enter ? "enter" : "leave", device.device_uuid.c_str());
        return;
    }

    if (callback_) {
        ProximityEvent event;
        event.device_id = device.device_uuid;
        event.rssi = device.last_rssi;
        event.distance = device.last_distance;
        event.is_enter = is_enter;

        ESP_LOGI(TAG, "Proximity %s: device=%.8s... rssi=%d dist=%.1fm",
                 is_enter ? "ENTER" : "LEAVE",
                 device.device_uuid.c_str(),
                 device.last_rssi,
                 device.last_distance);

        callback_(event);
    }
}

bool BleProximity::CheckRateLimit() {
    int64_t now = GetCurrentTimeMs();
    if (now - last_report_time_ms_ < kMinReportIntervalMs) {
        return false;
    }
    last_report_time_ms_ = now;
    return true;
}

float BleProximity::EstimateDistance(int rssi) const {
    // Log-distance path loss model:
    // distance = 10 ^ ((tx_power - rssi) / (10 * n))
    // where tx_power = RSSI at 1m, n = path loss exponent
    if (rssi >= 0) return 0.0f;

    float ratio = (float)(kTxPowerAt1m - rssi) / (10.0f * kPathLossExponent);
    float distance = powf(10.0f, ratio);

    // Clamp to reasonable range
    return std::max(0.1f, std::min(distance, 50.0f));
}

int64_t BleProximity::GetCurrentTimeMs() const {
    return esp_timer_get_time() / 1000;  // esp_timer returns microseconds
}
