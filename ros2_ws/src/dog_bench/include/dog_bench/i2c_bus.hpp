// I2C register access for the bench tools: the real Linux i2c-dev bus (one
// ioctl(I2C_RDWR) per transaction, the slave address inside every message) and
// an in-memory fake used by the gtests.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace dog_bench
{

/// One I2C master on one /dev node. Every transaction carries the slave
/// address; failures return false and never throw.
class I2cBus
{
public:
  virtual ~I2cBus() = default;

  /// Write the register pointer, then read 2 big-endian bytes.
  [[nodiscard]] virtual bool read16(int addr, uint8_t reg, uint16_t & out) = 0;
  /// Write one 16-bit register.
  [[nodiscard]] virtual bool write16(int addr, uint8_t reg, uint16_t value) = 0;
  /// Read 2 bytes without writing the register pointer first.
  [[nodiscard]] virtual bool readBare16(int addr, uint16_t & out) = 0;
  /// Write the register pointer, then read 1 byte.
  [[nodiscard]] virtual bool read8(int addr, uint8_t reg, uint8_t & out) = 0;
  /// One transaction: data[0] is the first register, the rest are written to
  /// consecutive registers. len 0 or above 32 is rejected.
  [[nodiscard]] virtual bool writeBytes(int addr, const uint8_t * data, std::size_t len) = 0;
};

/// Linux i2c-dev bus. Opening failures leave ok() false with the reason in
/// error(); copying is forbidden.
class LinuxI2cBus : public I2cBus
{
public:
  explicit LinuxI2cBus(const std::string & device);
  ~LinuxI2cBus() override;
  LinuxI2cBus(const LinuxI2cBus &) = delete;
  LinuxI2cBus & operator=(const LinuxI2cBus &) = delete;

  [[nodiscard]] bool read16(int addr, uint8_t reg, uint16_t & out) override;
  [[nodiscard]] bool write16(int addr, uint8_t reg, uint16_t value) override;
  [[nodiscard]] bool readBare16(int addr, uint16_t & out) override;
  [[nodiscard]] bool read8(int addr, uint8_t reg, uint8_t & out) override;
  [[nodiscard]] bool writeBytes(int addr, const uint8_t * data, std::size_t len) override;

  bool ok() const {return fd_ >= 0;}
  const std::string & error() const {return error_;}
  int lastErrno() const {return last_errno_;}

private:
  bool failed(int err);

  int fd_{-1};
  std::string error_;
  int last_errno_{0};
};

/// In-memory bus for tests: per-address register maps, a register pointer per
/// device and deterministic failure injection.
class FakeI2cBus : public I2cBus
{
public:
  void addDevice(int addr);
  void setReg16(int addr, uint8_t reg, uint16_t value);
  /// Unknown registers read as 0.
  uint16_t reg16(int addr, uint8_t reg) const;
  void setReg8(int addr, uint8_t reg, uint8_t value);
  uint8_t reg8(int addr, uint8_t reg) const;
  /// The write still reports success but the register keeps its value.
  void dropWrites(int addr, bool drop);
  /// Add a failing range: `count` consecutive transactions from `first`
  /// (numbered from 0, counted since construction) fail. Ranges accumulate.
  void failRange(int first, int count);
  /// Called at the start of every transaction (tests move a fake clock here).
  void setTransactionHook(std::function<void()> hook);

  int transactions() const {return transactions_;}
  int transactions(int addr) const;
  /// Attempted write16 and writeBytes calls.
  int writes() const {return writes_;}
  /// read16 (pointer + read) calls.
  int pointerReads() const {return pointer_reads_;}
  /// readBare16 calls.
  int bareReads() const {return bare_reads_;}

  [[nodiscard]] bool read16(int addr, uint8_t reg, uint16_t & out) override;
  [[nodiscard]] bool write16(int addr, uint8_t reg, uint16_t value) override;
  [[nodiscard]] bool readBare16(int addr, uint16_t & out) override;
  [[nodiscard]] bool read8(int addr, uint8_t reg, uint8_t & out) override;
  [[nodiscard]] bool writeBytes(int addr, const uint8_t * data, std::size_t len) override;

private:
  struct Device
  {
    std::map<uint8_t, uint16_t> regs;
    uint8_t pointer{0x00};
    bool drop_writes{false};
  };

  Device * deviceFor(int addr);
  const Device * deviceFor(int addr) const;
  bool fails(int number) const;
  int beginTransaction(int addr);

  std::map<int, Device> devices_;
  std::map<int, int> counts_;
  std::function<void()> hook_;
  int transactions_{0};
  int writes_{0};
  int pointer_reads_{0};
  int bare_reads_{0};
  std::vector<std::pair<int, int>> fail_ranges_;
};

}  // namespace dog_bench
