#include "MarauderCsi.h"

#include <math.h>
#include <string.h>

#include "esp_mac.h"

MarauderCsi::MarauderCsi() {
}

esp_err_t MarauderCsi::start(
    MarauderCsiSource source,
    MarauderCsiFormat format,
    uint32_t sample_divisor,
    uint8_t channel,
    const uint8_t* mac_filter) {
  this->stop();

  this->source_ = source;
  this->format_ = format;
  this->sample_divisor_ = sample_divisor == 0 ? 1 : sample_divisor;
  this->channel_ = channel;
  this->filter_mac_ = mac_filter != nullptr;
  if (this->filter_mac_) {
    memcpy(this->filter_bssid_, mac_filter, sizeof(this->filter_bssid_));
  }
  else {
    memset(this->filter_bssid_, 0, sizeof(this->filter_bssid_));
  }

  this->resetCounters();
  this->clearQueue();

#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C61)
  wifi_csi_config_t csi_config = {
      .enable                   = true,
      .acquire_csi_legacy       = true,
      .acquire_csi_force_lltf   = false,
      .acquire_csi_ht20         = true,
      .acquire_csi_ht40         = true,
      .acquire_csi_vht          = false,
      .acquire_csi_su           = false,
      .acquire_csi_mu           = false,
      .acquire_csi_dcm          = false,
      .acquire_csi_beamformed   = false,
      .acquire_csi_he_stbc_mode = 2,
      .val_scale_cfg            = 0,
      .dump_ack_en              = false,
      .reserved                 = false,
  };
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
  wifi_csi_config_t csi_config = {
      .enable                 = true,
      .acquire_csi_legacy     = true,
      .acquire_csi_ht20       = true,
      .acquire_csi_ht40       = true,
      .acquire_csi_su         = false,
      .acquire_csi_mu         = false,
      .acquire_csi_dcm        = false,
      .acquire_csi_beamformed = false,
      .acquire_csi_he_stbc    = 2,
      .val_scale_cfg          = false,
      .dump_ack_en            = false,
      .reserved               = false,
  };
#else
  wifi_csi_config_t csi_config = {
      .lltf_en           = true,
      .htltf_en          = true,
      .stbc_htltf2_en    = true,
      .ltf_merge_en      = true,
      .channel_filter_en = false,
      .manu_scale        = false,
      .shift             = false,
  };
#endif

  esp_err_t status = esp_wifi_set_csi_config(&csi_config);
  if (status != ESP_OK) {
#ifdef MARAUDER_DEBUG_USB_BOOT
    Serial.print(F("CSI_DEBUG config err: "));
    Serial.println((int)status);
#endif
    this->source_ = MARAUDER_CSI_SOURCE_NONE;
    this->filter_mac_ = false;
    return status;
  }

  status = esp_wifi_set_csi_rx_cb(&MarauderCsi::wifiCsiRxCallback, this);
  if (status != ESP_OK) {
#ifdef MARAUDER_DEBUG_USB_BOOT
    Serial.print(F("CSI_DEBUG rx_cb err: "));
    Serial.println((int)status);
#endif
    this->source_ = MARAUDER_CSI_SOURCE_NONE;
    this->filter_mac_ = false;
    return status;
  }

  status = esp_wifi_set_csi(true);
  if (status == ESP_OK) {
    this->active_ = true;
  }
  else {
#ifdef MARAUDER_DEBUG_USB_BOOT
    Serial.print(F("CSI_DEBUG enable err: "));
    Serial.println((int)status);
#endif
    this->source_ = MARAUDER_CSI_SOURCE_NONE;
    this->filter_mac_ = false;
  }

  return status;
}

void MarauderCsi::stop() {
  if (!this->active_) {
    return;
  }

  esp_wifi_set_csi(false);
  esp_wifi_set_csi_rx_cb(nullptr, nullptr);

  this->active_ = false;
  this->source_ = MARAUDER_CSI_SOURCE_NONE;
  this->filter_mac_ = false;
  this->clearQueue();
}

bool MarauderCsi::active() const {
  return this->active_;
}

bool MarauderCsi::routerMode() const {
  return this->source_ == MARAUDER_CSI_SOURCE_ROUTER;
}

MarauderCsiSource MarauderCsi::source() const {
  return this->source_;
}

MarauderCsiFormat MarauderCsi::format() const {
  return this->format_;
}

uint32_t MarauderCsi::sampleDivisor() const {
  return this->sample_divisor_;
}

uint8_t MarauderCsi::channel() const {
  return this->channel_;
}

void MarauderCsi::setChannel(uint8_t channel) {
  this->channel_ = channel;
}

uint32_t MarauderCsi::framesSeen() const {
  return this->frames_seen_;
}

uint32_t MarauderCsi::framesEmitted() const {
  return this->frames_emitted_;
}

uint32_t MarauderCsi::framesDropped() const {
  return this->frames_dropped_;
}

void MarauderCsi::printStatus() const {
  Serial.print(F("CSI_STATUS ACTIVE:"));
  Serial.print(this->active_ ? 1 : 0);
  Serial.print(F(" SRC:"));
  Serial.print(MarauderCsi::sourceName(this->source_));
  Serial.print(F(" FMT:"));
  Serial.print(MarauderCsi::formatName(this->format_));
  Serial.print(F(" CH:"));
  Serial.print(this->channel_);
  Serial.print(F(" SAMPLE:"));
  Serial.print(this->sample_divisor_);
  Serial.print(F(" SEEN:"));
  Serial.print(this->frames_seen_);
  Serial.print(F(" EMIT:"));
  Serial.print(this->frames_emitted_);
  Serial.print(F(" DROP:"));
  Serial.println(this->frames_dropped_);
}

void MarauderCsi::resetCounters() {
  this->frames_seen_ = 0;
  this->frames_emitted_ = 0;
  this->frames_dropped_ = 0;
}

void MarauderCsi::clearQueue() {
  portENTER_CRITICAL(&this->queue_mux_);
  this->queue_head_ = 0;
  this->queue_tail_ = 0;
  this->queue_count_ = 0;
  portEXIT_CRITICAL(&this->queue_mux_);
}

const char* MarauderCsi::sourceName(MarauderCsiSource source) {
  if (source == MARAUDER_CSI_SOURCE_PASSIVE) {
    return "passive";
  }
  else if (source == MARAUDER_CSI_SOURCE_ROUTER) {
    return "router";
  }

  return "none";
}

const char* MarauderCsi::formatName(MarauderCsiFormat format) {
  if (format == MARAUDER_CSI_FORMAT_RAW) {
    return "raw";
  }

  return "summary";
}

void MarauderCsi::wifiCsiRxCallback(void* ctx, wifi_csi_info_t* info) {
  MarauderCsi* self = (MarauderCsi*)ctx;
  if (!self || !self->active_ || !info || !info->buf) {
    return;
  }

  if (self->filter_mac_ && memcmp(info->mac, self->filter_bssid_, sizeof(self->filter_bssid_)) != 0) {
    return;
  }

  uint32_t sample_index = ++self->frames_seen_;
  if ((self->sample_divisor_ > 1) && (((sample_index - 1) % self->sample_divisor_) != 0)) {
    self->frames_dropped_++;
    return;
  }

  if (info->rx_ctrl.channel > 0) {
    self->channel_ = info->rx_ctrl.channel;
  }

  if (!self->enqueueFrame(info, sample_index)) {
    self->frames_dropped_++;
  }
}

bool MarauderCsi::enqueueFrame(const wifi_csi_info_t* info, uint32_t sample_index) {
  PendingFrame frame = {};
  const wifi_pkt_rx_ctrl_t* rx_ctrl = &info->rx_ctrl;
  const int8_t* csi = (const int8_t*)info->buf;

  frame.format = this->format_;
  frame.source = this->source_;
  frame.sample_index = sample_index;
  frame.channel = rx_ctrl->channel > 0 ? rx_ctrl->channel : this->channel_;
  frame.rssi = rx_ctrl->rssi;
  frame.rate = rx_ctrl->rate;
  frame.sig_mode = rx_ctrl->sig_mode;
  frame.mcs = rx_ctrl->mcs;
  frame.cwb = rx_ctrl->cwb;
  frame.smoothing = rx_ctrl->smoothing;
  frame.not_sounding = rx_ctrl->not_sounding;
  frame.aggregation = rx_ctrl->aggregation;
  frame.stbc = rx_ctrl->stbc;
  frame.fec_coding = rx_ctrl->fec_coding;
  frame.sgi = rx_ctrl->sgi;
  frame.noise_floor = rx_ctrl->noise_floor;
  frame.ampdu_cnt = rx_ctrl->ampdu_cnt;
  frame.secondary_channel = rx_ctrl->secondary_channel;
  frame.ant = rx_ctrl->ant;
  frame.timestamp = rx_ctrl->timestamp;
  frame.sig_len = rx_ctrl->sig_len;
  frame.len = info->len;
  frame.first_word_invalid = info->first_word_invalid ? true : false;
  memcpy(frame.mac, info->mac, sizeof(frame.mac));

  int bins = 0;
  float sum = 0.0f;
  float sum_sq = 0.0f;

  for (int i = 0; i + 1 < info->len; i += 2) {
    float real = (float)csi[i];
    float imag = (float)csi[i + 1];
    float amp = sqrtf((real * real) + (imag * imag));
    sum += amp;
    sum_sq += amp * amp;
    bins++;
  }

  if (bins == 0 && info->len > 0) {
    float amp = fabsf((float)csi[0]);
    sum = amp;
    sum_sq = amp * amp;
    bins = 1;
  }

  float mean = bins > 0 ? sum / (float)bins : 0.0f;
  float variance = bins > 0 ? (sum_sq / (float)bins) - (mean * mean) : 0.0f;
  if (variance < 0.0f) {
    variance = 0.0f;
  }
  frame.amp_mean_x100 = (int32_t)lroundf(mean * 100.0f);
  frame.amp_std_x100 = (int32_t)lroundf(sqrtf(variance) * 100.0f);

  if (frame.format == MARAUDER_CSI_FORMAT_RAW) {
    frame.stored_len = frame.len > kMaxPayloadLen ? kMaxPayloadLen : frame.len;
    frame.truncated = frame.stored_len != frame.len;
    memcpy(frame.payload, info->buf, frame.stored_len);
  }

  bool queued = false;
  portENTER_CRITICAL(&this->queue_mux_);
  if (this->queue_count_ < kQueueDepth) {
    this->queue_[this->queue_head_] = frame;
    this->queue_head_ = (this->queue_head_ + 1) % kQueueDepth;
    this->queue_count_++;
    queued = true;
  }
  portEXIT_CRITICAL(&this->queue_mux_);

  return queued;
}

void MarauderCsi::service() {
  if (!this->active_ && this->queue_count_ == 0) {
    return;
  }

  uint8_t processed = 0;
  while (processed < 2) {
    PendingFrame frame = {};
    bool have_frame = false;

    portENTER_CRITICAL(&this->queue_mux_);
    if (this->queue_count_ > 0) {
      frame = this->queue_[this->queue_tail_];
      this->queue_tail_ = (this->queue_tail_ + 1) % kQueueDepth;
      this->queue_count_--;
      have_frame = true;
    }
    portEXIT_CRITICAL(&this->queue_mux_);

    if (!have_frame) {
      break;
    }

    if (frame.format == MARAUDER_CSI_FORMAT_RAW) {
      this->emitRaw(frame);
    }
    else {
      this->emitSummary(frame);
    }
    this->frames_emitted_++;
    processed++;
  }
}

void MarauderCsi::emitSummary(const PendingFrame& frame) {
  Serial.printf(
      "CSI_SUM CH:%d RSSI:%d MAC:" MACSTR " LEN:%d AMP_MEAN_X100:%ld AMP_STD_X100:%ld NF:%d FWI:%d SRC:%s\r\n",
      frame.channel,
      frame.rssi,
      MAC2STR(frame.mac),
      frame.len,
      (long)frame.amp_mean_x100,
      (long)frame.amp_std_x100,
      frame.noise_floor,
      frame.first_word_invalid ? 1 : 0,
      MarauderCsi::sourceName(frame.source));
}

void MarauderCsi::emitRaw(const PendingFrame& frame) {
#if defined(CONFIG_IDF_TARGET_ESP32C5) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C61)
  Serial.printf(
      "CSI_DATA,%lu," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
      (unsigned long)frame.sample_index,
      MAC2STR(frame.mac),
      frame.rssi,
      frame.rate,
      frame.noise_floor,
      0,
      0,
      frame.channel,
      frame.timestamp,
      frame.sig_len,
      0);
#else
  Serial.printf(
      "CSI_DATA,%lu," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
      (unsigned long)frame.sample_index,
      MAC2STR(frame.mac),
      frame.rssi,
      frame.rate,
      frame.sig_mode,
      frame.mcs,
      frame.cwb,
      frame.smoothing,
      frame.not_sounding,
      frame.aggregation,
      frame.stbc,
      frame.fec_coding,
      frame.sgi,
      frame.noise_floor,
      frame.ampdu_cnt,
      frame.channel,
      frame.secondary_channel,
      frame.timestamp,
      frame.ant,
      frame.sig_len,
      frame.sig_mode);
#endif

  Serial.printf(",%d,%d,\"[", frame.len, frame.first_word_invalid ? 1 : 0);
  for (uint16_t i = 0; i < frame.stored_len; i++) {
    if (i > 0) {
      Serial.print(',');
    }
    Serial.print((int)frame.payload[i]);
  }
  if (frame.truncated) {
    if (frame.stored_len > 0) {
      Serial.print(',');
    }
    Serial.print("TRUNC");
  }
  Serial.print("]\"\r\n");
}
