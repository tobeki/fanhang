#pragma once

#include <mutex>

namespace manual_return_planner {

// The two command sources that may feed the cascadePID controller.
//
// NORMAL         : the ordinary diff_planner -> traj_server command stream.
// MANUAL_RETURN  : the Manual Return Executor command stream.
//
// The gate is a pure policy: exactly one of these sources may be forwarded to
// the controller output topic at any time.  A source that is not currently
// selected is dropped even if messages keep arriving.  This is what guarantees
// a stale diff_planner/traj_server stream can never fight the return flight.
enum class CommandSource {
  NORMAL = 0,
  MANUAL_RETURN = 1
};

// Pure, thread-safe, header-only policy used by CommandGateNode and by the
// unit tests.  It intentionally does not touch ROS so the switch semantics can
// be verified without a running graph.
class CommandGateCore {
 public:
  CommandGateCore() = default;

  // Select which source is allowed through.  Switching is a deliberate action;
  // arriving commands never cause an implicit switch.
  void setSource(CommandSource source) {
    std::lock_guard<std::mutex> lock(mutex_);
    source_ = source;
  }

  CommandSource source() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return source_;
  }

  // A one-way latch used by the trigger flow: once armed for manual return the
  // gate stays there until an explicit reset to NORMAL.
  void switchToManualReturn() {
    std::lock_guard<std::mutex> lock(mutex_);
    source_ = CommandSource::MANUAL_RETURN;
  }

  void switchToNormal() {
    std::lock_guard<std::mutex> lock(mutex_);
    source_ = CommandSource::NORMAL;
  }

  // True iff a command arriving from `input` should be forwarded to the
  // controller.  In NORMAL mode a return command is dropped; in MANUAL_RETURN
  // mode a normal command is dropped.
  bool shouldForward(CommandSource input) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return input == source_;
  }

 private:
  mutable std::mutex mutex_;
  CommandSource source_ = CommandSource::NORMAL;
};

}  // namespace manual_return_planner
