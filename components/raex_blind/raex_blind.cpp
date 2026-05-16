#include "raex_blind.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace raex_blind {

static const char *TAG = "raex_blind_tx";

// ---- CC1101 command strobes / register access bits ----
static const uint8_t CC1101_SRES = 0x30;        // reset strobe
static const uint8_t CC1101_STX = 0x35;         // enable TX strobe
static const uint8_t CC1101_SIDLE = 0x36;       // exit TX/RX -> IDLE strobe
static const uint8_t CC1101_PATABLE = 0x3E;     // PA table address
static const uint8_t CC1101_SRX = 0x34;         // enable RX strobe
static const uint8_t CC1101_MARCSTATE = 0x35;   // status reg (burst-bit access)
static const uint8_t CC1101_MARCSTATE_TX = 0x13;
static const uint8_t CC1101_MARCSTATE_RX = 0x0D;
static const uint8_t CC1101_READ = 0x80;
static const uint8_t CC1101_BURST = 0x40;

// Verified register set: 26 MHz xtal, 433.92 MHz, OOK, asynchronous serial TX.
// FREQ2/1/0 (0x0D-0x0F) are written separately, derived from freq_hz_.
struct CC1101Reg {
  uint8_t addr;
  uint8_t val;
};
static const CC1101Reg CC1101_CONFIG[] = {
  {0x00, 0x0D},  // IOCFG2
  {0x02, 0x0D},  // IOCFG0   - GDO0 = serial data (async TX data input pin)
  {0x03, 0x47},  // FIFOTHR
  {0x07, 0x04},  // PKTCTRL1
  {0x08, 0x32},  // PKTCTRL0 - async serial, no CRC, infinite length
  {0x0B, 0x06},  // FSCTRL1
  {0x0C, 0x00},  // FSCTRL0
  {0x10, 0x87},  // MDMCFG4
  {0x11, 0x93},  // MDMCFG3  - ~4.8 kBaud (timing resolution for OOK pulses)
  {0x12, 0x30},  // MDMCFG2  - OOK, no Manchester, no sync
  {0x13, 0x02},  // MDMCFG1
  {0x14, 0xF8},  // MDMCFG0
  {0x15, 0x47},  // DEVIATN
  {0x18, 0x18},  // MCSM0    - FS autocal on IDLE->TX
  {0x19, 0x16},  // FOCCFG
  {0x1A, 0x1C},  // BSCFG
  {0x1B, 0xC7},  // AGCCTRL2
  {0x1C, 0x00},  // AGCCTRL1
  {0x1D, 0xB2},  // AGCCTRL0
  {0x21, 0x56},  // FREND1
  {0x22, 0x11},  // FREND0   - OOK uses PATABLE index 0/1
  {0x23, 0xE9},  // FSCAL3
  {0x24, 0x2A},  // FSCAL2
  {0x25, 0x00},  // FSCAL1
  {0x26, 0x1F},  // FSCAL0
  {0x2C, 0x81},  // TEST2
  {0x2D, 0x35},  // TEST1
  {0x2E, 0x09},  // TEST0
};

// RX needs different RX-BW/AGC than the TX/base table. Applied on entering RX;
// the matching proven TX values are re-applied on entering TX so the working TX
// path is byte-for-byte unchanged. Starting points — tune per Verification.
static const CC1101Reg CC1101_RX_OVERRIDES[] = {
  {0x10, 0x27},  // MDMCFG4  - RX BW (low nibble 7 kept = TX DRATE_E)
  {0x1B, 0x04},  // AGCCTRL2 - OOK MAGN_TARGET
  {0x1C, 0x40},  // AGCCTRL1 - relative carrier-sense (suppress idle noise)
  {0x1D, 0x92},  // AGCCTRL0 - ASK/OOK decision boundary
  {0x19, 0x1D},  // FOCCFG
};
static const CC1101Reg CC1101_TX_OVERRIDES[] = {  // proven TX values for those regs
  {0x10, 0x87},  // MDMCFG4
  {0x1B, 0xC7},  // AGCCTRL2
  {0x1C, 0x00},  // AGCCTRL1
  {0x1D, 0xB2},  // AGCCTRL0
  {0x19, 0x16},  // FOCCFG
};

// ---- RX preamble-skip state-machine timing (see PROTOCOL.md; tunable) ----
// Universal anchor = the ~2540-2645us LONG SYNC (present in every remote,
// unambiguous: no data pulse exceeds ~1370us). Preamble varies per remote
// (RC-201-W: long ~320us AGC; others: none / a ~640us header) so it is NOT
// used for gating -- everything before the sync is ignored.
static const uint32_t RX_GLITCH_US = 80;        // ignore sub-glitch edges
static const uint32_t RX_DATA_MIN_US = 400;     // ~640us half-bit .. ~1300us double
static const uint32_t RX_DATA_MAX_US = 1700;
static const uint32_t RX_SYNC_MIN_US = 1800;    // the ~2640us long sync (data delimiter)
static const uint32_t RX_END_GAP_US = 5000;     // > sync, < ~25ms inter-repeat = frame end
static const uint16_t RX_DATA_MIN_EDGES = 30;   // payload ~40-110 edges; checksum is the gate
static const uint16_t RX_DATA_MAX_EDGES = 140;

// IRAM ISR preamble-skip state machine. The huge ~320us AGC preamble is ignored
// (never buffered); only the post-preamble data edges are stored
// (+us = HIGH segment, -us = LOW segment). loop() decodes on `frame_ready`.
void IRAM_ATTR RaexRxStore::gpio_intr(RaexRxStore *s) {
  if (s->tx_active)
    return;
  const uint32_t now = micros();
  const bool level = s->pin.digital_read();
  if (level == s->prev_level)
    return;  // no real transition
  const uint32_t dur = now - s->prev_us;
  const bool ended_level = s->prev_level;  // level of the segment that just ended
  s->prev_us = now;
  s->prev_level = level;
  if (dur < RX_GLITCH_US)
    return;  // glitch: do not advance state
  const int32_t v = ended_level ? (int32_t) dur : -(int32_t) dur;

  switch (s->state) {
    case RX_IDLE:  // wait for the long sync; ignore preamble / header / gaps
      // Don't start a new capture or reset the ring while a decoded frame is
      // still pending (undrained) -- closes the TOCTOU vs rx_process_; the next
      // burst repeat is captured once rx_process_ clears frame_ready.
      if (dur >= RX_SYNC_MIN_US && dur < RX_END_GAP_US && !s->frame_ready) {
        s->state = RX_DATA;
        s->write = 0;
        s->read = 0;
        s->data_count = 0;
      }
      break;

    case RX_DATA:
      if (dur >= RX_END_GAP_US) {  // ~25ms inter-repeat gap = end of burst
        if (!s->frame_ready && s->data_count >= RX_DATA_MIN_EDGES &&
            s->data_count <= RX_DATA_MAX_EDGES) {
          s->frame_ready = true;
          s->frame_ms = millis();
        }
        s->state = RX_IDLE;
      } else if (dur >= RX_SYNC_MIN_US) {  // sync (2nd half / next repeat) = boundary
        if (!s->frame_ready && s->data_count >= RX_DATA_MIN_EDGES &&
            s->data_count <= RX_DATA_MAX_EDGES) {
          s->frame_ready = true;
          s->frame_ms = millis();
        }
        if (!s->frame_ready) {  // re-anchor for the following repeat's payload
          s->write = 0;
          s->read = 0;
          s->data_count = 0;
        }
      } else if (dur >= RX_DATA_MIN_US && dur <= RX_DATA_MAX_US) {
        if (!s->frame_ready && s->data_count < RX_BUF_SIZE) {
          uint32_t w = s->write;
          s->buffer[w] = v;
          s->write = (w + 1) % RX_BUF_SIZE;
          s->data_count = s->data_count + 1;  // not ++ (volatile, C++20)
        }
      } else {
        // out-of-band blip (preamble/header/jitter mid-capture): ignore
      }
      break;

    default:  // unexpected state -> recover
      s->state = RX_IDLE;
      break;
  }
}

void RaexBlindTransmitComponent::tx_write(bool level) { this->gdo0_->digital_write(level); }

void RaexBlindTransmitComponent::cc1101_write_reg(uint8_t addr, uint8_t val) {
  this->enable();
  this->transfer_byte(addr);
  this->transfer_byte(val);
  this->disable();
}

uint8_t RaexBlindTransmitComponent::cc1101_strobe(uint8_t cmd) {
  this->enable();
  uint8_t status = this->transfer_byte(cmd);
  this->disable();
  return status;
}

uint8_t RaexBlindTransmitComponent::cc1101_read_status(uint8_t addr) {
  this->enable();
  this->transfer_byte(addr | CC1101_READ | CC1101_BURST);  // status regs need the burst bit
  uint8_t val = this->transfer_byte(0x00);
  this->disable();
  return val;
}

void RaexBlindTransmitComponent::cc1101_reset() {
  // The CC1101 performs an automatic power-on reset at boot. CS is owned by the
  // ESPHome SPI bus lock and must only move inside a balanced enable()/disable()
  // transaction, so issue a software-reset SRES strobe and let it settle rather
  // than bit-banging the manual CSn reset sequence.
  delay(10);                       // let XOSC/regulator stabilize after boot
  this->cc1101_strobe(CC1101_SRES);
  delay(2);                        // SRES recovery time
}

void RaexBlindTransmitComponent::cc1101_write_config() {
  for (auto &r : CC1101_CONFIG) {
    this->cc1101_write_reg(r.addr, r.val);
  }

  // Base frequency: FREQ = round(f * 2^16 / f_xosc), f_xosc = 26 MHz
  uint32_t freq = (uint32_t) ((double) this->freq_hz_ * 65536.0 / 26000000.0 + 0.5);
  this->cc1101_write_reg(0x0D, (freq >> 16) & 0xFF);  // FREQ2
  this->cc1101_write_reg(0x0E, (freq >> 8) & 0xFF);   // FREQ1
  this->cc1101_write_reg(0x0F, freq & 0xFF);          // FREQ0

  // PATABLE: [0] = OOK off, [1] = OOK on (~max power @433 MHz)
  this->enable();
  this->transfer_byte(CC1101_PATABLE | CC1101_BURST);
  this->transfer_byte(0x00);
  this->transfer_byte(0xC0);
  this->disable();
}

void RaexBlindTransmitComponent::cc1101_idle() { this->cc1101_strobe(CC1101_SIDLE); }

void RaexBlindTransmitComponent::cc1101_enter_tx() {
  this->cc1101_strobe(CC1101_SIDLE);
  for (auto &r : CC1101_TX_OVERRIDES)  // guarantee the proven TX register set
    this->cc1101_write_reg(r.addr, r.val);
  this->cc1101_strobe(CC1101_STX);
  // IDLE->TX includes FS autocal (~700-800 us); poll MARCSTATE rather than fixed delay.
  uint32_t start = millis();
  while (this->cc1101_read_status(CC1101_MARCSTATE) != CC1101_MARCSTATE_TX) {
    if (millis() - start > 5) {  // ~5 ms safety timeout
      ESP_LOGW(TAG, "CC1101 did not reach TX state");
      break;
    }
  }
  ESP_LOGD(TAG, "CC1101 -> TX");
}

void RaexBlindTransmitComponent::cc1101_enter_rx() {
  this->cc1101_strobe(CC1101_SIDLE);
  for (auto &r : CC1101_RX_OVERRIDES)  // OOK RX-BW / AGC
    this->cc1101_write_reg(r.addr, r.val);
  this->cc1101_strobe(CC1101_SRX);
  uint32_t start = millis();
  while (this->cc1101_read_status(CC1101_MARCSTATE) != CC1101_MARCSTATE_RX) {
    if (millis() - start > 5) {
      ESP_LOGW(TAG, "CC1101 did not reach RX state");
      break;
    }
  }
  ESP_LOGD(TAG, "CC1101 -> RX");
}

RaexMessage::RaexMessage(
  uint32_t execute_time,
  uint8_t retries_remain,
  uint16_t remote_id,
  uint8_t channel_id,
  uint8_t action_id
):
  execute_time(execute_time),
  retries_remain(retries_remain),
  remote_id(remote_id),
  channel_id(channel_id),
  action_id(action_id) {}

void RaexBlindTransmitComponent::setup() {
  this->spi_setup();
  this->gdo0_->setup();
  this->gdo0_->digital_write(false);  // async data line idle LOW

  this->cc1101_reset();
  this->cc1101_write_config();

  this->pos_restore_();  // restore last position from flash (boots uncertain)

  if (this->rx_pin_ != nullptr) {
    this->rx_pin_->setup();
    this->rx_store_.pin = this->rx_pin_->to_isr();
    this->rx_store_.prev_us = micros();
    this->rx_store_.prev_level = this->rx_pin_->digital_read();
    this->rx_pin_->attach_interrupt(&RaexRxStore::gpio_intr, &this->rx_store_,
                                    gpio::INTERRUPT_ANY_EDGE);
    this->cc1101_enter_rx();  // default state = listening
  } else {
    this->cc1101_idle();
  }

  register_service(&RaexBlindTransmitComponent::transmit, "transmit",
                  {"remote_id", "channel_id", "action"});
  register_service(&RaexBlindTransmitComponent::transmit_custom, "transmit_custom",
                  {"remote_id", "channel_id", "action", "retries"});
  register_service(&RaexBlindTransmitComponent::set_position, "set_position",
                  {"remote_id", "channel_id", "position"});
}

void RaexBlindTransmitComponent::loop() {
  if (this->rx_pin_ != nullptr)
    this->rx_process_();

  this->pos_loop_();  // time-integration engine (runs every loop, even in lockout)

  if (!pending_messages.empty()) {
    uint32_t now = millis();
    // Wrap-safe deadline checks: a signed delta survives the ~49.7-day millis()
    // rollover (all intervals here are <=5min, far under the int32 ~24.8d limit).
    if ((int32_t) (now - lockout_until) < 0) {  // still inside the TX lockout
      return;
    }

    for (auto it = pending_messages.begin(); it != pending_messages.end(); it++) {
      auto key = it->first;
      auto msg = it->second;

      if ((int32_t) (now - msg->execute_time) >= 0) {  // deadline reached
        ESP_LOGD(TAG, "Executing send for [%d,%d,%d] [retries: %d]",
          msg->remote_id, msg->channel_id, msg->action_id, msg->retries_remain);

        this->rx_store_.tx_active = true;   // deafen RX ISR across the bit-bang
        this->cc1101_enter_tx();
        txPrepare(200, CLOCK_WIDTH);
        txRaexSend(msg->remote_id, msg->channel_id, msg->action_id, CLOCK_WIDTH);
        if (this->rx_pin_ != nullptr) {
          this->cc1101_enter_rx();          // resume listening
          this->rx_store_.prev_us = micros();
          this->rx_store_.prev_level = this->rx_pin_->digital_read();
        } else {
          this->cc1101_idle();
        }
        // Reset the RX state machine so a frame interrupted by TX (or stale
        // state) is discarded before the ISR is re-armed.
        this->rx_store_.state = RaexRxStore::RX_IDLE;
        this->rx_store_.data_count = 0;
        this->rx_store_.write = 0;
        this->rx_store_.read = 0;
        this->rx_store_.frame_ready = false;
        this->rx_store_.tx_active = false;
        lockout_until = millis() + LOCKOUT_DELAY_MS;

        if (msg->retries_remain > 0) {
          msg->retries_remain--;
          msg->execute_time = now + TRANSMIT_RETRY_DELAY_MS;
          return;
        }

        // Burst done -> drop the message; it is never re-sent.
        ESP_LOGD(TAG, "Burst complete for [%d,%d,%d], removing",
          msg->remote_id, msg->channel_id, msg->action_id);
        pending_messages.erase(it);
        delete msg;
        return;
      }
    }
  }
}

void RaexBlindTransmitComponent::transmit(int32_t remote_id, int32_t channel_id, std::string action) {
  transmit_custom(remote_id, channel_id, action, TRANSMIT_RETRIES);
}

void RaexBlindTransmitComponent::transmit_custom(int32_t remote_id, int32_t channel_id, std::string action,
                                                 int32_t retries) {
  raex_action_t action_id;
  if (action.compare("OPEN") == 0) {
    action_id = TX_RAEX_ACTION_UP;
  } else if (action.compare("CLOSE") == 0) {
    action_id = TX_RAEX_ACTION_DOWN;
  } else if (action.compare("STOP") == 0) {
    action_id = TX_RAEX_ACTION_STOP;
  } else if (action.compare("PAIR") == 0) {
    action_id = TX_RAEX_ACTION_PAIR;
  } else if (action.compare("REV_DIR") == 0) {
    action_id = TX_RAEX_ACTION_REV_DIR;
  } else if (action.compare("OPEN_NUDGE") == 0) {
    action_id = TX_RAEX_ACTION_NUDGE_LEFT;
  } else if (action.compare("CLOSE_NUDGE") == 0) {
    action_id = TX_RAEX_ACTION_NUDGE_RIGHT;
  } else {
    ESP_LOGE(TAG, "Malformed payload received. Unknown action [%s]", action.c_str());
    return;
  }

  ESP_LOGD(TAG, "Enqueing: %d, %d, %s", remote_id, channel_id, action.c_str());
  this->enqueue_(remote_id, channel_id, (uint8_t) action_id, retries);
}

void RaexBlindTransmitComponent::set_position(int32_t remote_id, int32_t channel_id, int32_t position) {
  if (position < 0) position = 0;
  if (position > 100) position = 100;

  // Per PROTOCOL.md: endpoints are the plain travel commands, interior is value+11.
  uint8_t action_byte;
  if (position <= 0) {
    action_byte = TX_RAEX_ACTION_DOWN;        // 0% -> full close (0xFC)
  } else if (position >= 100) {
    action_byte = TX_RAEX_ACTION_UP;          // 100% -> full open (0xFE)
  } else {
    action_byte = (uint8_t) (position + 11);  // 1..99% -> 0x0C..0x6E
  }

  ESP_LOGD(TAG, "Enqueing position: %d, %d, %d%% -> 0x%02X", remote_id, channel_id, position, action_byte);
  this->enqueue_(remote_id, channel_id, action_byte, TRANSMIT_RETRIES);
}

void RaexBlindTransmitComponent::enqueue_(int32_t remote_id, int32_t channel_id, uint8_t action_byte,
                                          int32_t retries) {
  uint32_t now = millis();

  // Canonicalize so an aliased remote shares the primary's queue slot (matches
  // apply_state_ keying; prevents duplicate slots/TX for one physical blind).
  uint32_t key = this->canon_key_((uint16_t) remote_id, (uint8_t) channel_id);
  auto search = pending_messages.find(key);
  if (search != pending_messages.end()) {
    ESP_LOGD(TAG, "Reusing existing slot with key: %d", key);
    auto msg = search->second;
    msg->execute_time = now;
    msg->retries_remain = retries - 1;
    msg->action_id = action_byte;
  } else {
    ESP_LOGD(TAG, "Inserting new message with key: %d", key);
    auto msg = new RaexMessage(now, retries - 1, remote_id, channel_id, action_byte);
    pending_messages.insert({key, msg});
  }

  ESP_LOGD(TAG, "New queue:");
  for (auto it = pending_messages.begin(); it != pending_messages.end(); it++) {
    ESP_LOGD(TAG, " - %u: [%d, %d, %d, %u]", it->first, it->second->remote_id, it->second->channel_id,
             it->second->action_id, it->second->execute_time);
  }

  // Command path is the source of truth for our own commands; reflect intent
  // immediately (once per command, not per radio retry). ts = now (TX start).
  this->apply_state_((uint16_t) remote_id, (uint8_t) channel_id, action_byte, "TX", millis());
}

void RaexBlindTransmitComponent::rx_process_() {
  if (!this->rx_store_.frame_ready)
    return;

  // Drain the small post-sync data buffer captured by the ISR.
  uint32_t w = this->rx_store_.write;
  std::vector<int32_t> pulses;
  pulses.reserve(RX_BUF_SIZE);
  uint32_t r = this->rx_store_.read;
  while (r != w) {
    pulses.push_back((int32_t) this->rx_store_.buffer[r]);  // read through volatile
    r = (r + 1) % RX_BUF_SIZE;
  }
  this->rx_store_.read = w;
  this->rx_store_.frame_ready = false;

  if (pulses.size() < RX_DATA_MIN_EDGES)
    return;  // partial / re-anchor artifact -> silently drop

  uint32_t nowm = millis();
  uint8_t channel, action;
  uint16_t remote;
  if (!this->rx_decode_(pulses, channel, remote, action)) {
    if (nowm - this->rx_dump_ms_ >= 5000) {  // throttle: partial repeats are normal
      this->rx_dump_ms_ = nowm;
      ESP_LOGD(TAG, "RX %u edges: no valid frame", (unsigned) pulses.size());
    }
    return;
  }

  uint32_t raw = ((uint32_t) remote << 8) | channel;

  // One identity may map to one OR MANY blinds: a single target is a normal
  // alias; several targets = a group (a remote's group/ALL button listed as an
  // alias under multiple covers). Fan the command out to every registered
  // target. Checked before the trust gate (the alias identity is not a blind).
  auto ait = this->alias_map_.find(raw);
  if (ait != this->alias_map_.end()) {
    if (raw == this->rx_dedup_key_ && action == this->rx_dedup_action_ &&
        (nowm - this->rx_dedup_ms_) < RX_DEDUP_WINDOW_MS) {  // collapse the on-air burst
      ESP_LOGD(TAG, "RX alias dup suppressed [%u/%u act=%u]", remote, channel, action);
      return;
    }
    this->rx_dedup_key_ = raw;
    this->rx_dedup_action_ = action;
    this->rx_dedup_ms_ = nowm;
    for (uint32_t t : ait->second) {
      if (!this->blinds_.count(t))
        continue;  // only a registered blind has an engine/cover
      this->apply_state_((uint16_t) (t >> 8), (uint8_t) (t & 0xFF), action, "RX",
                         this->rx_store_.frame_ms);
    }
    return;
  }

  // Direct identity (the blind's own primary). Trust gate (see PROTOCOL.md RX):
  // only a registered blind moves the cover.
  if (this->blinds_.count(raw)) {
    if (raw == this->rx_dedup_key_ && action == this->rx_dedup_action_ &&
        (nowm - this->rx_dedup_ms_) < RX_DEDUP_WINDOW_MS) {  // collapses the 3x burst only
      ESP_LOGD(TAG, "RX dup suppressed [%u/%u act=%u]", remote, channel, action);
      return;
    }
    this->rx_dedup_key_ = raw;
    this->rx_dedup_action_ = action;
    this->rx_dedup_ms_ = nowm;
    // ts = ISR frame timestamp (closest proxy for the physical press time).
    this->apply_state_(remote, channel, action, "RX", this->rx_store_.frame_ms);
    return;
  }

  // Unregistered remote: discovery candidate. Never touches cover state.
  // Reported on the FIRST checksum-valid decode (the sum checksum already
  // rejects noise hard -- same gate the trust path acts on); a held button is
  // throttled to 1x / 10s per (remote,channel,action) so it can't spam.
  bool fresh = (raw != this->disc_key_ || action != this->disc_action_);
  if (fresh || (nowm - this->disc_logged_ms_) > 10000) {
    this->disc_key_ = raw;
    this->disc_action_ = action;
    this->disc_logged_ms_ = nowm;
    ESP_LOGI(TAG,
             "RX (unregistered) channel=%u id=%u.%u remoteID=%u (0x%04X) action=%u (0x%02X) "
             "cksum=OK -- not tracked; add a 'cover:' entry (or an alias) with this "
             "remote_id/channel",
             channel, remote & 0xFF, (remote >> 8) & 0xFF, remote, remote, action, action);
  }
}

// Decode contract (see PROTOCOL.md; proven by walking the known-good frame).
// Input = post-preamble DATA edges only (signed us). half-bit ~640us;
// Manchester decoded_bit = first_half ^ 1; drop 1 leading framing bit; then
// 5 LSB-first bytes [channel][id_lo][id_hi][command][checksum].
bool RaexBlindTransmitComponent::rx_decode_(const std::vector<int32_t> &pulses, uint8_t &channel,
                                            uint16_t &remote, uint8_t &action) {
  static const int HALFBIT_US = 640;
  static const size_t US_MAX = 256;
  std::vector<uint8_t> us;
  us.reserve(US_MAX);
  for (int32_t p : pulses) {
    bool lvl = p > 0;
    uint32_t d = (uint32_t) (p > 0 ? p : -p);
    int n = (int) ((d + HALFBIT_US / 2) / HALFBIT_US);  // round, not truncate
    if (n < 1)
      n = 1;
    if (n > 6)
      n = 6;  // safety clamp
    for (int i = 0; i < n && us.size() < US_MAX; i++)
      us.push_back(lvl ? 1 : 0);
  }

  // Manchester: pair half-bits from index 0 (phase anchored by preamble-skip);
  // bit = first_half ^ 1; a non-transition pair marks end of frame.
  std::vector<uint8_t> bits;
  for (size_t i = 0; i + 1 < us.size(); i += 2) {
    uint8_t a = us[i], b = us[i + 1];
    if (a == b)
      break;
    bits.push_back((uint8_t) (a ^ 1));
  }
  if (bits.size() < 41)
    return false;  // need 1 framing + 40 payload bits

  uint8_t by[5];
  for (int k = 0; k < 5; k++) {
    uint8_t v = 0;
    for (int j = 0; j < 8; j++)
      v |= (uint8_t) (bits[1 + k * 8 + j] << j);  // drop bits[0]; LSB-first
    by[k] = v;
  }
  uint8_t ch = by[0], lo = by[1], hi = by[2], cmd = by[3], ck = by[4];
  if (((ch + lo + hi + cmd + 3) & 0xFF) != ck)
    return false;

  channel = ch;
  remote = (uint16_t) (lo | (hi << 8));
  action = cmd;
  return true;
}

// Interpolated current position for an in-flight move (no side effects).
// Always integrates from start_pos (the best estimate) so partial moves take
// |delta|*travel_time regardless of `known`; the cover is assumed-state and a
// full OPEN/CLOSE self-resyncs any drift at the hard limit.
double RaexBlindTransmitComponent::pos_interp_(const RaexPos &p, uint32_t key, uint32_t now) {
  if (p.op == POS_IDLE)
    return p.pos;
  BlindCfg &c = this->cfg_for_(key);
  uint32_t tt = (p.op == POS_OPENING) ? c.open_ms : c.close_ms;
  if (tt == 0)
    return p.pos;  // defensive: register_blind clamps to >=1, never divide by 0
  double frac = (double) (now - p.move_start_ms) / (double) tt;
  double v = (p.op == POS_OPENING) ? p.start_pos + frac : p.start_pos - frac;
  if (v < 0.0)
    v = 0.0;
  if (v > 1.0)
    v = 1.0;
  return v;
}

void RaexBlindTransmitComponent::pos_settle_(RaexPos &p, uint32_t key, uint32_t now) {
  p.pos = this->pos_interp_(p, key, now);
  p.start_pos = p.pos;
  p.move_start_ms = now;
}

void RaexBlindTransmitComponent::apply_state_(uint16_t remote, uint8_t channel, uint8_t action,
                                              const char *src, uint32_t ts) {
  const char *name = "UNKNOWN";
  ESP_LOGI(TAG, "%s frame: channel=%u id=%u.%u remoteID=%u (0x%04X) action=%u (0x%02X) cksum=OK",
           src, channel, remote & 0xFF, (remote >> 8) & 0xFF, remote, remote, action, action);

  uint32_t key = this->canon_key_(remote, channel);  // fold onto the blind's primary
  RaexPos &p = this->pos_[key];
  double old = p.pos;

  // Freeze the in-flight position at the moment this command takes effect, then
  // anchor the next move there. move_start_ms unconditionally reset to `ts`.
  this->pos_settle_(p, key, ts);

  switch (action) {
    case TX_RAEX_ACTION_UP:
      name = "UP";
      p.target = 1.0;
      p.op = (p.pos >= 1.0 && p.known) ? POS_IDLE : POS_OPENING;
      break;
    case TX_RAEX_ACTION_DOWN:
      name = "DOWN";
      p.target = 0.0;
      p.op = (p.pos <= 0.0 && p.known) ? POS_IDLE : POS_CLOSING;
      break;
    case TX_RAEX_ACTION_STOP:
      name = "STOP";
      p.target = p.pos;
      p.op = POS_IDLE;  // estimate; not exact, not a resync
      break;
    case TX_RAEX_ACTION_NUDGE_LEFT:  // 0xDC micro-up (== favorite, not separable)
      name = "MICRO-UP/FAV";
      p.pos += this->cfg_for_(key).micro;
      if (p.pos > 1.0)
        p.pos = 1.0;
      p.target = p.pos;
      p.start_pos = p.pos;
      p.op = POS_IDLE;
      break;
    case TX_RAEX_ACTION_NUDGE_RIGHT:  // 0xDB micro-down
      name = "MICRO-DOWN";
      p.pos -= this->cfg_for_(key).micro;
      if (p.pos < 0.0)
        p.pos = 0.0;
      p.target = p.pos;
      p.start_pos = p.pos;
      p.op = POS_IDLE;
      break;
    case TX_RAEX_ACTION_PAIR:
      name = "PAIR";
      break;  // no position effect
    default:
      if (action >= 0x0C && action <= 0x6E) {
        name = "POSITION";
        p.target = (double) (action - 11) / 100.0;
        if (p.target > p.start_pos)
          p.op = POS_OPENING;
        else if (p.target < p.start_pos)
          p.op = POS_CLOSING;
        else
          p.op = POS_IDLE;
      }
      break;
  }

  this->pos_dirty_ = true;
  ESP_LOGI(TAG, "pos[%u/%u] %d%%->%d%% target=%d%% op=%u known=%d (src=%s %s)",
           (unsigned) (key >> 8), (unsigned) (key & 0xFF), (int) (old * 100.0 + 0.5),
           (int) (p.pos * 100.0 + 0.5), (int) (p.target * 100.0 + 0.5), p.op, p.known, src,
           name);
}

uint32_t RaexBlindTransmitComponent::canon_key_(uint16_t remote, uint8_t channel) {
  uint32_t key = ((uint32_t) remote << 8) | channel;
  auto it = this->alias_map_.find(key);
  return (it != this->alias_map_.end() && !it->second.empty()) ? it->second.front() : key;
}

void RaexBlindTransmitComponent::add_alias(int32_t from_remote, int32_t from_channel,
                                           int32_t to_remote, int32_t to_channel) {
  uint32_t from = (((uint32_t) from_remote) << 8) | (uint32_t) (from_channel & 0xFF);
  uint32_t to = (((uint32_t) to_remote) << 8) | (uint32_t) (to_channel & 0xFF);
  auto &targets = this->alias_map_[from];
  for (uint32_t t : targets)
    if (t == to)
      return;  // already mapped (config listed the same alias twice)
  targets.push_back(to);
  ESP_LOGI(TAG, "alias: %d/%d -> primary %d/%d (%u target%s)", from_remote, from_channel,
           to_remote, to_channel, (unsigned) targets.size(), targets.size() == 1 ? "" : "s");
}

RaexBlindTransmitComponent::BlindCfg &RaexBlindTransmitComponent::cfg_for_(uint32_t canon) {
  static BlindCfg fallback;  // all-default; only ever serves keys not in blinds_
  auto it = this->blinds_.find(canon);
  return it == this->blinds_.end() ? fallback : it->second;
}

void RaexBlindTransmitComponent::register_blind(int32_t remote_id, int32_t channel,
                                                uint32_t open_ms, uint32_t close_ms,
                                                float micro) {
  uint32_t key = this->canon_key_((uint16_t) remote_id, (uint8_t) channel);
  if (this->blinds_.count(key)) {
    ESP_LOGW(TAG, "register_blind: %d/%d already registered (key=%u) -- ignoring dup",
             remote_id, channel, key);
    return;
  }
  BlindCfg c;
  c.open_ms = open_ms ? open_ms : 1;     // never divide by zero in pos_interp_
  c.close_ms = close_ms ? close_ms : 1;
  c.micro = (double) micro;
  // Stable per-blind NVS id from the canonical identity: deterministic across
  // reboot/reflash, order-independent, namespaced (0x5242 = "RB").
  c.pref_id = 0x52420000u ^ (key * 2654435761u);
  c.pref = global_preferences->make_preference<float>(c.pref_id);
  this->blinds_[key] = c;
  ESP_LOGI(TAG, "register_blind %d/%d key=%u open=%ums close=%ums micro=%.3f pref=0x%08X",
           remote_id, channel, key, c.open_ms, c.close_ms, c.micro, c.pref_id);
}

optional<float> RaexBlindTransmitComponent::cover_position(int32_t remote_id, int32_t channel_id) {
  uint32_t key = this->canon_key_((uint16_t) remote_id, (uint8_t) channel_id);
  auto it = this->pos_.find(key);
  if (it == this->pos_.end())
    return optional<float>{};  // never commanded -> HA shows unknown
  // Live value, computed on read so it's fresh regardless of loop ordering.
  double v = this->pos_interp_(it->second, key, millis());
  if (v < 0.0)
    v = 0.0;
  if (v > 1.0)
    v = 1.0;
  return optional<float>{(float) v};
}

uint8_t RaexBlindTransmitComponent::cover_operation(int32_t remote_id, int32_t channel_id) {
  uint32_t key = this->canon_key_((uint16_t) remote_id, (uint8_t) channel_id);
  auto it = this->pos_.find(key);
  return it == this->pos_.end() ? POS_IDLE : it->second.op;
}

// Per-loop integrator + self-resync + throttled persistence.
void RaexBlindTransmitComponent::pos_loop_() {
  uint32_t now = millis();
  for (auto &kv : this->pos_) {
    RaexPos &p = kv.second;
    if (p.op == POS_IDLE)
      continue;
    bool endpoint = (p.target >= POS_ENDPOINT_HI || p.target <= POS_ENDPOINT_LO);
    // Always interpolate from start_pos over |delta|*travel_time. The tracked
    // position is the best estimate even when !known, so a full OPEN/CLOSE from
    // 80% takes 0.2*travel, not the whole travel. Reaching an endpoint still
    // sets known=true, so the hard limits keep self-resyncing any drift.
    double v = this->pos_interp_(p, kv.first, now);
    p.pos = v;
    bool reached = (p.op == POS_OPENING) ? (v >= p.target - 1e-6) : (v <= p.target + 1e-6);
    if (reached) {
      p.pos = p.target;
      if (endpoint)
        p.known = true;  // reached a hard limit -> position is now exact
      p.op = POS_IDLE;
      this->pos_dirty_ = true;
    }
  }

  if (!this->pos_dirty_)
    return;
  // Persist every registered blind that has engine state. Immediate when all
  // are settled (stop/idle), else throttled to >=30s while any animates
  // (flash-wear safe; ESPHome batches the actual NVS write).
  bool any_moving = false;
  for (auto &bkv : this->blinds_) {
    auto pit = this->pos_.find(bkv.first);
    if (pit != this->pos_.end() && pit->second.op != POS_IDLE)
      any_moving = true;
  }
  if (!any_moving || (now - this->pos_save_ms_) >= POS_SAVE_THROTTLE_MS) {
    for (auto &bkv : this->blinds_) {
      auto pit = this->pos_.find(bkv.first);
      if (pit == this->pos_.end())
        continue;
      float v = (float) pit->second.pos;
      bkv.second.pref.save(&v);
    }
    this->pos_save_ms_ = now;
    this->pos_dirty_ = false;
  }
}

void RaexBlindTransmitComponent::pos_restore_() {
  // blinds_ is fully populated by register_blind() (cover cg.add setters run at
  // boot before this component's setup()); load each blind's last position.
  for (auto &bkv : this->blinds_) {
    uint32_t key = bkv.first;
    float v = 0.0f;
    if (bkv.second.pref.load(&v) && v >= 0.0f && v <= 1.0f) {
      RaexPos &p = this->pos_[key];
      p.pos = v;
      p.start_pos = v;
      p.target = v;
      p.op = POS_IDLE;
      p.known = false;  // always boot uncertain; first full travel re-syncs exactly
      ESP_LOGI(TAG, "restored pos[%u/%u] = %d%% (uncertain until next full travel)",
               (unsigned) (key >> 8), (unsigned) (key & 0xFF),
               (int) (v * 100.0f + 0.5f));
    }
  }
}

void RaexBlindTransmitComponent::manchesterWriteBit(uint16_t clockWidth, bool bit) {
  tx_write(bit ? false : true);  // LOW : HIGH
  delayMicroseconds(clockWidth);
  tx_write(bit ? true : false);  // HIGH : LOW
  delayMicroseconds(clockWidth);
}

void RaexBlindTransmitComponent::manchesterWriteByteBigEndian(uint16_t clockWidth, uint8_t byte) {
  for (size_t i = 0; i < 8; i++) {
    bool bit = (bool) (byte & (1 << i));
    manchesterWriteBit(clockWidth, bit);
  }
}

void RaexBlindTransmitComponent::txPrepare(int numRounds, uint16_t clockWidth) {
  for (int i = 0; i < numRounds; i++) {
    tx_write(true);   // HIGH
    delayMicroseconds(clockWidth);
    tx_write(false);  // LOW
    delayMicroseconds(clockWidth);
  }
  tx_write(true);      // HIGH
  delayMicroseconds(clockWidth);
}

void RaexBlindTransmitComponent::txRaexWriteHeader(uint16_t clockWidth) {
  for (size_t i = 0; i < 20; i++) {
    tx_write(false);  // LOW
    delayMicroseconds(clockWidth * 2);
    tx_write(true);   // HIGH
    delayMicroseconds(clockWidth * 2);
  }

  tx_write(false);    // LOW
  delayMicroseconds(clockWidth * 2);

  tx_write(true);     // HIGH
  delayMicroseconds(clockWidth * 2 * 4);
  tx_write(false);    // LOW
  delayMicroseconds(clockWidth * 2 * 4);
}

void RaexBlindTransmitComponent::rxRaexWriteData(uint16_t remote, uint8_t channel, uint8_t action, int checksum, uint16_t clockWidth) {
  manchesterWriteBit(clockWidth * 2, 0);
  manchesterWriteByteBigEndian(clockWidth * 2, channel);
  manchesterWriteByteBigEndian(clockWidth * 2, remote & 0xFF);
  manchesterWriteByteBigEndian(clockWidth * 2, remote >> 8);
  manchesterWriteByteBigEndian(clockWidth * 2, action);
  manchesterWriteByteBigEndian(clockWidth * 2, checksum);
  manchesterWriteBit(clockWidth * 2, 0);
  manchesterWriteBit(clockWidth * 2, 0);
}

uint8_t RaexBlindTransmitComponent::txRaexCalculateChecksum(uint16_t remote, uint8_t channel, uint8_t action) {
  return channel + (remote & 0xFF) + (remote >> 8) + (action & 0xFF) + 3;
}

void RaexBlindTransmitComponent::txRaexSend(uint16_t remote, uint8_t channel, uint8_t action, uint16_t clockWidth) {
  uint8_t checksum = txRaexCalculateChecksum(remote, channel, action);

  for (int i = 0; i < 4; i++) {
    txRaexWriteHeader(clockWidth);
    rxRaexWriteData(remote, channel, action, checksum, clockWidth);
  }
}

}  // namespace raex_blind
}  // namespace esphome