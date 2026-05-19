#pragma once

#ifndef MarauderCsi_h
#define MarauderCsi_h

#include <Arduino.h>

#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"

enum MarauderCsiSource : uint8_t {
  MARAUDER_CSI_SOURCE_NONE = 0,
  MARAUDER_CSI_SOURCE_PASSIVE = 1,
  MARAUDER_CSI_SOURCE_ROUTER = 2,
};

enum MarauderCsiFormat : uint8_t {
  MARAUDER_CSI_FORMAT_SUMMARY = 0,
  MARAUDER_CSI_FORMAT_RAW = 1,
};

class MarauderCsi {
  public:
    MarauderCsi();

    esp_err_t start(
        MarauderCsiSource source,
        MarauderCsiFormat format,
        uint32_t sample_divisor,
        uint8_t channel,
        const uint8_t* mac_filter = nullptr);
    void stop();

    bool active() const;
    bool routerMode() const;
    MarauderCsiSource source() const;
    MarauderCsiFormat format() const;
    uint32_t sampleDivisor() const;
    uint8_t channel() const;
    void setChannel(uint8_t channel);

    uint32_t framesSeen() const;
    uint32_t framesEmitted() const;
    uint32_t framesDropped() const;

    void printStatus() const;
    void service();

  private:
    static const uint8_t kQueueDepth = 4;
    static const uint16_t kMaxPayloadLen = 384;

    struct PendingFrame {
      MarauderCsiFormat format;
      MarauderCsiSource source;
      uint32_t sample_index;
      uint8_t channel;
      int8_t rssi;
      int8_t rate;
      int8_t sig_mode;
      int8_t mcs;
      int8_t cwb;
      int8_t smoothing;
      int8_t not_sounding;
      int8_t aggregation;
      int8_t stbc;
      int8_t fec_coding;
      int8_t sgi;
      int8_t noise_floor;
      int8_t ampdu_cnt;
      int8_t secondary_channel;
      int8_t ant;
      uint32_t timestamp;
      uint16_t sig_len;
      uint16_t len;
      uint16_t stored_len;
      bool first_word_invalid;
      bool truncated;
      int32_t amp_mean_x100;
      int32_t amp_std_x100;
      uint8_t mac[6];
      int8_t payload[kMaxPayloadLen];
    };

    static void wifiCsiRxCallback(void* ctx, wifi_csi_info_t* info);
    static const char* sourceName(MarauderCsiSource source);
    static const char* formatName(MarauderCsiFormat format);

    void resetCounters();
    bool enqueueFrame(const wifi_csi_info_t* info, uint32_t sample_index);
    void emitSummary(const PendingFrame& frame);
    void emitRaw(const PendingFrame& frame);
    void clearQueue();

    volatile bool active_ = false;
    volatile uint32_t frames_seen_ = 0;
    volatile uint32_t frames_emitted_ = 0;
    volatile uint32_t frames_dropped_ = 0;

    MarauderCsiSource source_ = MARAUDER_CSI_SOURCE_NONE;
    MarauderCsiFormat format_ = MARAUDER_CSI_FORMAT_SUMMARY;
    uint32_t sample_divisor_ = 1;
    uint8_t channel_ = 0;
    bool filter_mac_ = false;
    uint8_t filter_bssid_[6] = {0};
    portMUX_TYPE queue_mux_ = portMUX_INITIALIZER_UNLOCKED;
    PendingFrame queue_[kQueueDepth] = {};
    volatile uint8_t queue_head_ = 0;
    volatile uint8_t queue_tail_ = 0;
    volatile uint8_t queue_count_ = 0;
};

#endif
