#pragma once

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/raex_blind/raex_blind.h"

namespace esphome {
namespace raex_blind {

// Thin HA cover entity over the time-integration engine in
// RaexBlindTransmitComponent. control() issues RF via the parent (which feeds
// the engine through apply_state_); loop() publishes the engine's live
// position/operation so physical-remote-driven moves surface in HA too.
class RaexCover : public cover::Cover, public Component {
 public:
  void set_parent(RaexBlindTransmitComponent *p) { this->parent_ = p; }
  void set_remote_id(int32_t r) { this->remote_id_ = r; }
  void set_channel(int32_t c) { this->channel_ = c; }

  void loop() override;
  void control(const cover::CoverCall &call) override;
  cover::CoverTraits get_traits() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  RaexBlindTransmitComponent *parent_{nullptr};
  int32_t remote_id_{0};
  int32_t channel_{0};
};

}  // namespace raex_blind
}  // namespace esphome
