#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <mutex>

#include "esp_err.h"
#include "esp_timer.h"

/**
 * @brief BLE Proximity Detection Module
 *
 * Implements BLE advertising and scanning for near-field social feature.
 * When two Ti-social devices come within proximity range, this module
 * detects the event and triggers a callback for the application to
 * report to the server via WebSocket.
 *
 * Design:
 * - BLE advertising carries Company ID + Device Type + Device UUID (20 bytes)
 * - Scanning filters for the same Company ID to find Ti-social devices
 * - RSSI-based distance estimation with hysteresis thresholds
 * - Time-based debouncing to avoid flapping
 * - Rate limiting: max 1 report per second, enter/leave each once per device
 */
class BleProximity {
public:
    static BleProximity& GetInstance();

    /**
     * @brief Configuration for BLE proximity detection
     */
    struct Config {
        uint8_t device_type = 0x01;             // Ti-social device type
        std::string device_uuid;                // 16-byte device UUID (hex string, 32 chars)
        int enter_rssi_threshold = -70;         // dBm — above this = "near"
        int leave_rssi_threshold = -85;         // dBm — below this = "far"
        int enter_duration_ms = 2000;           // Debounce: must stay near for this long
        int leave_duration_ms = 5000;           // Debounce: must stay far for this long
        uint16_t company_id = 0xFFFF;           // BLE Company ID (0xFFFF = test/default)
    };

    /**
     * @brief Proximity event data
     */
    struct ProximityEvent {
        std::string device_id;      // Detected device UUID (hex)
        int rssi;                   // Current RSSI in dBm
        float distance;             // Estimated distance in meters
        bool is_enter;              // true = entering range, false = leaving
    };

    using ProximityCallback = std::function<void(const ProximityEvent& event)>;

    /**
     * @brief Initialize BLE proximity module
     * Must be called after BLE stack is available.
     */
    esp_err_t Start(const Config& config);

    /**
     * @brief Stop BLE advertising and scanning
     */
    void Stop();

    /**
     * @brief Register callback for proximity events
     */
    void RegisterProximityCallback(ProximityCallback callback);

    /**
     * @brief Check if proximity detection is active
     */
    bool IsRunning() const { return running_; }

    /**
     * @brief Get the device UUID used for advertising
     */
    const std::string& GetDeviceUuid() const { return config_.device_uuid; }

    // Delete copy/move
    BleProximity(const BleProximity&) = delete;
    BleProximity& operator=(const BleProximity&) = delete;

private:
    BleProximity();
    ~BleProximity();

    // Internal tracking for each detected device
    struct TrackedDevice {
        std::string device_uuid;
        int last_rssi = 0;
        float last_distance = 0.0f;
        int64_t first_seen_near_ms = 0;     // Timestamp when RSSI first crossed enter threshold
        int64_t first_seen_far_ms = 0;      // Timestamp when RSSI first crossed leave threshold
        bool is_near = false;               // Currently in "near" state (debounced)
        bool enter_reported = false;        // Enter event already reported (rate limit)
        bool leave_reported = false;        // Leave event already reported (rate limit)
    };

    // BLE GAP callback trampoline
    static void GapEventCallback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

    // Internal methods
    esp_err_t StartAdvertising();
    esp_err_t StartScanning();
    void ProcessScanResult(const std::string& device_addr, const uint8_t* manufacturer_data,
                          int manufacturer_data_len, int rssi);
    float EstimateDistance(int rssi) const;
    int64_t GetCurrentTimeMs() const;
    void CheckDebounce(TrackedDevice& device);
    void ReportEvent(TrackedDevice& device, bool is_enter);

    // Rate limiting
    bool CheckRateLimit();

    Config config_;
    bool running_ = false;
    ProximityCallback callback_;

    std::mutex devices_mutex_;
    std::unordered_map<std::string, TrackedDevice> tracked_devices_;

    int64_t last_report_time_ms_ = 0;
    static constexpr int kMinReportIntervalMs = 1000;  // Max 1 report per second

    // RSSI to distance estimation parameters
    static constexpr int kTxPowerAt1m = -59;   // RSSI at 1 meter (typical for ESP32)
    static constexpr float kPathLossExponent = 2.5f;  // Indoor environment
};
