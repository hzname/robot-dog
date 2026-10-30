// dog_diagnostics/reporter.hpp - every node reports itself in the same shape.
//
// The reason string is the same one the node writes to its log: the page shows
// exactly what the operator can grep for (DIA-01, DIA-02). Levels follow
// diagnostic_msgs: OK while everything is in order, WARN when a threshold is near,
// ERROR when the robot cannot work (latched fault, fallen, sensor gone).

#ifndef DOG_DIAGNOSTICS__REPORTER_HPP_
#define DOG_DIAGNOSTICS__REPORTER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dog_diagnostics
{

enum class Level : uint8_t {OK = 0, WARN = 1, ERROR = 2};

inline uint8_t toStatusLevel(Level level)
{
  switch (level) {
    case Level::ERROR: return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    case Level::WARN: return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    default: return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
}

/// Rolls one node's status into a `diagnostics` message. `hardware_id` names the
/// board, `source` the node: the page groups by source and shows the worst level.
class Reporter
{
public:
  Reporter(rclcpp::Node * node, std::string source, std::string hardware_id = "robot")
  : source_(std::move(source)), hardware_id_(std::move(hardware_id)), node_(node)
  {
    pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  }

  /// Builds the status. `values` are extra numbers/flags for the log reader.
  void publish(Level level, const std::string & reason,
    std::vector<std::pair<std::string, std::string>> values = {})
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = toStatusLevel(level);
    status.name = source_;
    status.hardware_id = hardware_id_;
    status.message = reason;
    for (auto & kv : values) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = kv.first;
      item.value = kv.second;
      status.values.push_back(item);
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = node_->now();
    array.status.push_back(status);
    pub_->publish(array);
  }

private:
  std::string source_;
  std::string hardware_id_;
  rclcpp::Node * node_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
};

}  // namespace dog_diagnostics

#endif  // DOG_DIAGNOSTICS__REPORTER_HPP_
