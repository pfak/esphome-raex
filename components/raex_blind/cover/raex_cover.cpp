#include "raex_cover.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace raex_blind {

static const char *const TAG = "raex_cover";

void RaexCover::control(const cover::CoverCall &call) {
  if (this->parent_ == nullptr)
    return;
  if (call.get_stop()) {
    this->parent_->transmit(this->remote_id_, this->channel_, "STOP");
    return;
  }
  auto pos = call.get_position();
  if (pos.has_value()) {
    float t = *pos;
    // Endpoints use the dedicated travel commands (run to the hard limit and
    // self-resync); interior targets use the positional command.
    if (t >= POS_ENDPOINT_HI)
      this->parent_->transmit(this->remote_id_, this->channel_, "OPEN");
    else if (t <= POS_ENDPOINT_LO)
      this->parent_->transmit(this->remote_id_, this->channel_, "CLOSE");
    else
      this->parent_->set_position(this->remote_id_, this->channel_,
                                  (int) (t * 100.0f + 0.5f));
  }
}

cover::CoverTraits RaexCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  traits.set_is_assumed_state(true);  // one-way RF, position is time-estimated
  return traits;
}

void RaexCover::loop() {
  if (this->parent_ == nullptr)
    return;

  uint8_t op = this->parent_->cover_operation(this->remote_id_, this->channel_);
  cover::CoverOperation cop = op == POS_OPENING   ? cover::COVER_OPERATION_OPENING
                              : op == POS_CLOSING ? cover::COVER_OPERATION_CLOSING
                                                  : cover::COVER_OPERATION_IDLE;
  auto p = this->parent_->cover_position(this->remote_id_, this->channel_);

  // Don't animate the % while moving: publish the operation (opening/closing)
  // immediately, but only publish the position when it has settled (back to
  // IDLE) or changed instantaneously while IDLE (e.g. a micro-step). The engine
  // still tracks % continuously internally for STOP/chaining/resync.
  bool op_changed = cop != this->current_operation;
  bool idle_pos_changed = cop == cover::COVER_OPERATION_IDLE && p.has_value() &&
                          std::fabs(*p - this->position) > 0.0005f;
  if (op_changed || idle_pos_changed) {
    if (cop == cover::COVER_OPERATION_IDLE && p.has_value())
      this->position = *p;  // settled final value
    this->current_operation = cop;
    this->publish_state(false);  // engine owns persistence; don't double-store
  }
}

}  // namespace raex_blind
}  // namespace esphome
