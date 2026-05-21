#include "link_watchdog.h"

LinkWatchdog::LinkWatchdog(uint32_t timeoutMs)
    : timeoutMs_(timeoutMs), lastCommandMs_(0), linkOk_(false) {}

LinkWatchdog::Change LinkWatchdog::onCommand(bool restoresLink, uint32_t nowMs) {
  lastCommandMs_ = nowMs;
  if (!linkOk_ && restoresLink) {
    linkOk_ = true;
    return Change::Restored;
  }
  return Change::None;
}

LinkWatchdog::Change LinkWatchdog::update(uint32_t nowMs) {
  if (linkOk_ && nowMs - lastCommandMs_ >= timeoutMs_) {
    linkOk_ = false;
    return Change::Lost;
  }
  return Change::None;
}
