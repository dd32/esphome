#pragma once

#include "esphome/core/defines.h"

#include "canopen.h"
#include "types.h"

namespace esphome::canopen {

// Base of components that drive another node (device profiles): the hub's events for that node, SDO / NMT with its
// id. Monitored with register_node(heartbeat_timeout=...).
class CANopenNode {
 public:
  CANopenNode(CANopenComponent *parent, uint8_t node_id);
  // the hub callbacks capture this: no copies or moves
  CANopenNode(const CANopenNode &) = delete;
  CANopenNode &operator=(const CANopenNode &) = delete;

  CANopenComponent *get_canopen() const { return this->canopen_; }
  uint8_t get_node_id() const { return this->node_id_; }

  // from the node's heartbeats; false / UNKNOWN if it isn't monitored
  bool is_online() const { return this->canopen_->is_node_online(this->node_id_); }
  NMTState get_nmt_state() const { return this->canopen_->get_node_state(this->node_id_); }
  bool is_operational() const { return this->is_online() && this->get_nmt_state() == NMTState::NMT_STATE_OPERATIONAL; }

  // the default SDO context is the CANopenNode subobject: use from_context<T>(), not static_cast
  template<typename T> static T *from_context(void *context) {
    return static_cast<T *>(static_cast<CANopenNode *>(context));
  }

#ifdef USE_CANOPEN_SDO_CLIENT
  // SDO calls for this node; a nullptr context means this
  template<typename T> bool read_sdo(uint16_t index, uint8_t sub, SDOCallback callback, void *context = nullptr) {
    return this->canopen_->read_sdo<T>(this->node_id_, index, sub, callback, this->context_(context));
  }
  template<typename T>
  bool write_sdo(uint16_t index, uint8_t sub, T value, SDOCallback callback = nullptr, void *context = nullptr) {
    return this->canopen_->write_sdo<T>(this->node_id_, index, sub, value, callback, this->context_(context));
  }
  // data must stay valid until the callback
  bool read_sdo_data(uint16_t index, uint8_t sub, uint8_t *data, size_t capacity, SDOCallback callback,
                     void *context = nullptr) {
    return this->canopen_->read_sdo_data(this->node_id_, index, sub, data, capacity, callback, this->context_(context));
  }
  // data must stay valid until the callback
  bool write_sdo_data(uint16_t index, uint8_t sub, const uint8_t *data, size_t len, SDOCallback callback = nullptr,
                      void *context = nullptr) {
    return this->canopen_->write_sdo_data(this->node_id_, index, sub, data, len, callback, this->context_(context));
  }
  // drops context's queued requests and aborts its transfer, without callbacks
  void cancel_sdo(void *context = nullptr) { this->canopen_->cancel_sdo(this->context_(context)); }
#endif
#ifdef USE_CANOPEN_NMT_MASTER
  bool send_nmt(NMTCommand command) { return this->canopen_->send_nmt(this->node_id_, command); }
#endif

 protected:
  // not deleted through a CANopenNode pointer
  ~CANopenNode() = default;

  // on_node_nmt_changed() also follows online and offline: act on online / reset
  virtual void on_node_online() {}
  virtual void on_node_offline() {}
  virtual void on_node_nmt_changed(NMTState state) {}
  // boot-up: the node lost its runtime configuration
  virtual void on_node_reset() {}
  virtual void on_node_emcy(const Emergency &emergency) {}

  void *context_(void *context) { return context != nullptr ? context : this; }

  CANopenComponent *canopen_;
  uint8_t node_id_;
};

}  // namespace esphome::canopen
