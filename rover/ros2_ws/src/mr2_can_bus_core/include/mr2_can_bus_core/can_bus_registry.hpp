// =============================================================
// can_bus_registry.hpp  —  per‑interface shared instance cache
// =============================================================
#pragma once

#include "can_bus_manager.hpp"
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

class CanBusRegistry {
public:
  /** Get or create manager for given interface name. */
  static std::shared_ptr<CanBusManager> get(const std::string &iface,
                                            int bitrate = 1'000'000) {
    std::lock_guard<std::mutex> lk(map_mtx_);
    auto &weak = map_[iface];
    auto sp = weak.lock();
    if (!sp) {
      sp = std::make_shared<CanBusManager>();
      if (!sp->start(iface, bitrate)) {
        map_.erase(iface);
        return {};
      }
      weak = sp;
    }
    return sp;
  }

private:
  static inline std::unordered_map<std::string, std::weak_ptr<CanBusManager>>
      map_{};
  static inline std::mutex map_mtx_;
};
