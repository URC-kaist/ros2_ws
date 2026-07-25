// =============================================================
// can_bus_registry.hpp  —  per‑interface shared instance cache
// =============================================================
#pragma once

#include "can_bus_manager.hpp"
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class CanBusRegistry {
public:
  /** Get or create manager for given interface name. */
  static std::shared_ptr<CanBusManager> get(const std::string &iface) {
    std::lock_guard<std::mutex> lk(map_mtx());
    auto &weak = map()[iface];
    auto sp = weak.lock();
    if (!sp) {
      sp = std::make_shared<CanBusManager>();
      if (!sp->start(iface)) {
        map().erase(iface);
        return {};
      }
      weak = sp;
    }
    return sp;
  }

  /** Stop every live manager before owners of registered callbacks vanish. */
  static void stop_all() {
    std::vector<std::shared_ptr<CanBusManager>> managers;
    {
      std::lock_guard<std::mutex> lk(map_mtx());
      managers.reserve(map().size());
      for (auto &[iface, weak] : map()) {
        (void)iface;
        if (auto manager = weak.lock()) {
          managers.push_back(std::move(manager));
        }
      }
    }
    for (auto &manager : managers) {
      manager->stop();
    }
  }

private:
  // Intentionally heap-allocate process-lifetime statics to avoid deinit-order
  // crashes when plugin/shared-library teardown happens at process exit.
  static std::unordered_map<std::string, std::weak_ptr<CanBusManager>> &map() {
    static auto *instance =
        new std::unordered_map<std::string, std::weak_ptr<CanBusManager>>();
    return *instance;
  }

  static std::mutex &map_mtx() {
    static auto *instance = new std::mutex();
    return *instance;
  }
};
