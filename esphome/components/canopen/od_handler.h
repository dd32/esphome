#pragma once

// OD handlers: OdObject subclasses get every access (OdStream) to their entries.
#include <cstddef>
#include <cstdint>

#include "esphome/core/defines.h"  // before CANopenNode: clang-tidy gets the CO_CONFIG_* values from it

#define OD_DEFINITION
// Use the library search path: quoted includes match our canopen.h on Windows.
#include <CANopen.h>

namespace esphome::canopen {

// CANopenNode's ODR_t values; errors become SDO abort codes
enum class OdResult : int8_t {
  OD_RESULT_PARTIAL = ODR_PARTIAL,  // more read / write calls follow
  OD_RESULT_OK = ODR_OK,
  OD_RESULT_OUT_OF_MEMORY = ODR_OUT_OF_MEM,
  OD_RESULT_UNSUPPORTED_ACCESS = ODR_UNSUPP_ACCESS,
  OD_RESULT_WRITE_ONLY = ODR_WRITEONLY,
  OD_RESULT_READ_ONLY = ODR_READONLY,
  OD_RESULT_NOT_EXIST = ODR_IDX_NOT_EXIST,
  OD_RESULT_DEVICE_INCOMPATIBLE = ODR_DEV_INCOMPAT,
  OD_RESULT_HARDWARE = ODR_HW,
  OD_RESULT_TYPE_MISMATCH = ODR_TYPE_MISMATCH,
  OD_RESULT_DATA_LONG = ODR_DATA_LONG,
  OD_RESULT_DATA_SHORT = ODR_DATA_SHORT,
  OD_RESULT_SUB_NOT_EXIST = ODR_SUB_NOT_EXIST,
  OD_RESULT_INVALID_VALUE = ODR_INVALID_VALUE,
  OD_RESULT_VALUE_HIGH = ODR_VALUE_HIGH,
  OD_RESULT_VALUE_LOW = ODR_VALUE_LOW,
  OD_RESULT_GENERAL = ODR_GENERAL,
  OD_RESULT_DATA_TRANSFER = ODR_DATA_TRANSF,
  OD_RESULT_DEVICE_STATE = ODR_DATA_DEV_STATE,
  OD_RESULT_NO_DATA = ODR_NO_DATA,
};

// one access to an OD sub-entry, chunked for large data (wraps OD_stream_t)
class OdStream {
 public:
  explicit OdStream(OD_stream_t *stream) : stream_(stream) {}

  uint16_t index() const { return this->stream_->index; }
  uint8_t sub() const { return this->stream_->subIndex; }
  // bytes already transferred; a handler returning PARTIAL advances it
  size_t offset() const { return this->stream_->dataOffset; }
  void set_offset(size_t offset) { this->stream_->dataOffset = offset; }
  // the entry's length. A domain download has 0 until the last chunk, then the total; a read handler sets it at
  // offset 0 to indicate the upload size.
  size_t size() const { return this->stream_->dataLength; }
  void set_size(size_t size) { this->stream_->dataLength = size; }
  // the entry's storage, may be nullptr
  void *data() const { return this->stream_->dataOrig; }

  // the default access to the entry's storage
  OdResult read_original(void *buf, size_t count, size_t *done) {
    OD_size_t n = 0;
    auto ret = static_cast<OdResult>(OD_readOriginal(this->stream_, buf, count, &n));
    *done = n;
    return ret;
  }
  OdResult write_original(const void *buf, size_t count, size_t *done) {
    OD_size_t n = 0;
    auto ret = static_cast<OdResult>(OD_writeOriginal(this->stream_, buf, count, &n));
    *done = n;
    return ret;
  }

 protected:
  OD_stream_t *stream_;
};

// Base of OD handlers (set_object()): subclasses hide od_read() / od_write(), called without virtual dispatch, for SDO
// transfers and write_od() / read_od(), after the attribute check.
struct OdObject {
  OdResult od_read(OdStream stream, void *buf, size_t count, size_t *done) {
    return stream.read_original(buf, count, done);
  }
  OdResult od_write(OdStream stream, const void *buf, size_t count, size_t *done) {
    return stream.write_original(buf, count, done);
  }
};

}  // namespace esphome::canopen
