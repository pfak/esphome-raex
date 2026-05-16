#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/gpio.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/optional.h"
#include "esphome/core/preferences.h"
#include <map>
#include <vector>

namespace esphome {
namespace raex_blind {

// One short burst per command: TRANSMIT_RETRIES sends spaced
// TRANSMIT_RETRY_DELAY_MS apart, then the message is dropped (no timed
// re-sends). Each send itself emits ~4 on-air frames (a remote press is ~4).
#define TRANSMIT_RETRIES 2
#define TRANSMIT_RETRY_DELAY_MS 50
#define CLOCK_WIDTH 330
// Post-TX cooldown before the next queued send: radio TX->RX settle + pacing.
// Heuristic; floor is the CC1101 mode-switch (~ms) and the ~25ms protocol gap.
#define LOCKOUT_DELAY_MS 50
#define RX_DEDUP_WINDOW_MS 300        // collapse the ~75ms 3x on-air burst to one event
#define POS_SAVE_THROTTLE_MS 30000    // min ms between position flash writes while moving
// Endpoint thresholds, shared by the engine and the cover so they can't desync.
static const double POS_ENDPOINT_HI = 0.999;  // target >= -> full-open endpoint
static const double POS_ENDPOINT_LO = 0.001;  // target <= -> full-close endpoint

// The "action" slot is a general command/value byte (see PROTOCOL.md): travel
// codes, micro/fav codes, pair, and arbitrary go-to-position values all share it.
enum raex_action : uint8_t {
  TX_RAEX_ACTION_UP = 254,
  TX_RAEX_ACTION_DOWN = 252,
  TX_RAEX_ACTION_STOP = 253,
  TX_RAEX_ACTION_REV_DIR = 238,
  TX_RAEX_ACTION_NUDGE_LEFT = 220,
  TX_RAEX_ACTION_NUDGE_RIGHT = 219,
  TX_RAEX_ACTION_PAIR = 127,
};
typedef enum raex_action raex_action_t;

class RaexMessage {
  public:
    RaexMessage(
      uint32_t execute_time,
      uint8_t retries_remain,
      uint16_t remote_id,
      uint8_t channel_id,
      uint8_t action_id
    );

    uint32_t execute_time;
    uint8_t retries_remain;
    uint16_t remote_id;
    uint8_t channel_id;
    uint8_t action_id;
};

// ISR-context preamble-skip state machine. The ~320us AGC preamble is ignored
// (never buffered); only the ~41-bit data frame after it is stored.
// buffer entries are signed durations: +us = HIGH, -us = LOW.
static const uint32_t RX_BUF_SIZE = 128;  // post-preamble data only (~82 edges)

struct RaexRxStore {
  enum : uint8_t { RX_IDLE = 0, RX_DATA = 2 };
  ISRInternalGPIOPin pin;
  volatile int32_t buffer[RX_BUF_SIZE];
  volatile uint32_t write{0};
  volatile uint32_t read{0};
  volatile uint32_t prev_us{0};
  volatile bool prev_level{false};
  volatile bool tx_active{false};      // gate ISR off during our own TX
  volatile uint8_t state{RX_IDLE};
  volatile uint16_t data_count{0};
  volatile bool frame_ready{false};
  volatile uint32_t frame_ms{0};
  static void gpio_intr(RaexRxStore *s);
};

// Time-integrated position state per canonical (remote<<8|channel). op values
// match esphome::cover::CoverOperation (IDLE=0, OPENING=1, CLOSING=2) so the
// cover platform can map them directly.
enum raex_pos_op : uint8_t { POS_IDLE = 0, POS_OPENING = 1, POS_CLOSING = 2 };

struct RaexPos {
  double pos{0.0};        // 0.0 closed .. 1.0 open (ESPHome cover-native)
  double start_pos{0.0};  // pos at move_start_ms
  double target{0.0};
  uint32_t move_start_ms{0};
  uint8_t op{POS_IDLE};
  bool known{false};      // true after a move reaches a hard limit; cleared on boot
};

class RaexBlindTransmitComponent
    : public Component,
      public api::CustomAPIDevice,
      public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                            spi::CLOCK_POLARITY_LOW,    // CPOL=0
                            spi::CLOCK_PHASE_LEADING,   // CPHA=0  => SPI mode 0
                            spi::DATA_RATE_4MHZ> {
  private:
    std::map<uint32_t, RaexMessage*> pending_messages;
    uint32_t lockout_until = 0;

  protected:
    InternalGPIOPin *gdo0_{nullptr};
    InternalGPIOPin *rx_pin_{nullptr};
    float freq_hz_{433920000.0f};

    RaexRxStore rx_store_;
    // Time-integration position engine (single source of truth) keyed by
    // canonical (remote<<8|channel).
    std::map<uint32_t, RaexPos> pos_;
    // Per-blind config + persistence, keyed by the blind's canonical
    // (remote<<8|channel). Each cover entry registers one via register_blind().
    static constexpr uint32_t DEFAULT_OPEN_MS = 17000;
    static constexpr uint32_t DEFAULT_CLOSE_MS = 17000;
    static constexpr double DEFAULT_MICRO = 0.005;
    struct BlindCfg {
      uint32_t open_ms{DEFAULT_OPEN_MS};   // full closed -> open travel time
      uint32_t close_ms{DEFAULT_CLOSE_MS}; // full open -> closed travel time
      double micro{DEFAULT_MICRO};         // micro-up/down nudge
      ESPPreferenceObject pref;            // this blind's own persisted position
      uint32_t pref_id{0};
    };
    std::map<uint32_t, BlindCfg> blinds_;
    // blinds_[canon] or a static all-default fallback (defensive: a key not in
    // blinds_ never reaches the engine -- discovery path only).
    BlindCfg &cfg_for_(uint32_t canon);
    uint32_t pos_save_ms_{0};          // throttle flash writes
    bool pos_dirty_{false};
    // Identity key -> one OR MANY primary blind keys. One target = a normal
    // alias; several = a group (a remote's group/ALL button listed as an alias
    // under multiple covers). RX fans the command out to every registered target.
    std::map<uint32_t, std::vector<uint32_t>> alias_map_;
    // Resolve an identity to its primary (folds aliased remotes onto one blind).
    uint32_t canon_key_(uint16_t remote, uint8_t channel);
    uint32_t rx_dedup_key_{0};
    uint8_t rx_dedup_action_{0};
    uint32_t rx_dedup_ms_{0};
    // discovery (unregistered remote): log on first decode, 10s re-throttle
    uint32_t disc_key_{0};
    uint8_t disc_action_{0};
    uint32_t disc_logged_ms_{0};
    uint32_t rx_dump_ms_{0};

    void tx_write(bool level);
    void cc1101_reset();
    void cc1101_write_reg(uint8_t addr, uint8_t val);
    uint8_t cc1101_strobe(uint8_t cmd);
    uint8_t cc1101_read_status(uint8_t addr);  // status regs 0x30-0x3D, burst-bit access
    void cc1101_write_config();
    void cc1101_enter_tx();  // restore TX regs + STX + poll MARCSTATE==TX
    void cc1101_enter_rx();  // apply RX overrides + SRX + poll MARCSTATE==RX
    void cc1101_idle();      // SIDLE

    void rx_process_();      // loop(): gap-detect, drain buffer, decode
    bool rx_decode_(const std::vector<int32_t> &pulses, uint8_t &channel, uint16_t &remote,
                    uint8_t &action);
    // Update the position engine from a command/observed frame. `ts` = the
    // motion-start timestamp (TX: enqueue millis(); RX: ISR frame_ms).
    void apply_state_(uint16_t remote, uint8_t channel, uint8_t action, const char *src, uint32_t ts);
    double pos_interp_(const RaexPos &p, uint32_t key, uint32_t now);  // interpolated current pos
    void pos_settle_(RaexPos &p, uint32_t key, uint32_t now);          // freeze pos at interpolated
    void pos_loop_();                                    // integrator + persistence
    void pos_restore_();                                 // load from flash in setup()

  public:
    void set_gdo0_pin(InternalGPIOPin *pin) { this->gdo0_ = pin; }
    void set_rx_pin(InternalGPIOPin *pin) { this->rx_pin_ = pin; }
    void set_frequency(float hz) { this->freq_hz_ = hz; }
    // Each cover entry registers its blind: primary identity (the RX trust
    // gate), per-direction travel times, micro-step, and its own flash
    // persistence slot. Aliased remotes canon_key_ onto this primary.
    void register_blind(int32_t remote_id, int32_t channel, uint32_t open_ms,
                        uint32_t close_ms, float micro);
    // Map an extra remote (from_*) onto a blind's primary identity (to_*).
    void add_alias(int32_t from_remote, int32_t from_channel, int32_t to_remote, int32_t to_channel);
    // Live position (0..1) / operation for the cover platform. Empty optional
    // until the blind has been commanded at least once.
    optional<float> cover_position(int32_t remote_id, int32_t channel_id);
    uint8_t cover_operation(int32_t remote_id, int32_t channel_id);  // raex_pos_op

    void setup() override;
    void loop() override;

    void transmit(int32_t remote_id, int32_t channel_id, std::string action);
    void transmit_custom(int32_t remote_id, int32_t channel_id, std::string action, int32_t retries);
    // Go-to-position: position is open-% (0 = closed, 100 = open). Per PROTOCOL.md
    // 0% -> DOWN, 100% -> UP, 1..99% -> action byte = position + 11.
    void set_position(int32_t remote_id, int32_t channel_id, int32_t position);

    void manchesterWriteBit(uint16_t clockWidth, bool bit);
    void manchesterWriteByteBigEndian(uint16_t clockWidth, uint8_t byte);
    void txPrepare(int numRounds, uint16_t clockWidth);
    void txRaexWriteHeader(uint16_t clockWidth);
    void rxRaexWriteData(uint16_t remote, uint8_t channel, uint8_t action, int checksum, uint16_t clockWidth);
    uint8_t txRaexCalculateChecksum(uint16_t remote, uint8_t channel, uint8_t action);
    void txRaexSend(uint16_t remote, uint8_t channel, uint8_t action, uint16_t clockWidth);

  private:
    // Shared queue insert: applies the "only UP/DOWN get multi-block eventual
    // consistency" policy, then creates/reuses the per-remote+channel slot.
    void enqueue_(int32_t remote_id, int32_t channel_id, uint8_t action_byte, int32_t retries);
};

}  // namespace raex_blind
}  // namespace esphome