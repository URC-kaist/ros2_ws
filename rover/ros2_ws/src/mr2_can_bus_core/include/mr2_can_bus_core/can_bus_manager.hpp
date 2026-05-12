// =============================================================
// can_bus_manager.hpp  —  SocketCAN poll‑loop manager
// =============================================================
#pragma once

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

/**
 * CanBusManager — owns a single CAN interface (e.g. "can0").
 *  * non‑copyable, created via std::make_shared
 *  * start() opens SocketCAN RAW, spawns poll() I/O thread
 *  * register_listener() adds (id,mask) filter + callback
 *  * enqueue_tx()   queues frame and wakes thread via pipe
 */
class CanBusManager : public std::enable_shared_from_this<CanBusManager> {
public:
  using RxCallback = std::function<void(const struct can_frame &)>;
  using FdRxCallback = std::function<void(const struct canfd_frame &)>;

  CanBusManager() = default;
  ~CanBusManager() { stop(); }

  CanBusManager(const CanBusManager &) = delete;
  CanBusManager &operator=(const CanBusManager &) = delete;

  bool start(const std::string &iface);
  void stop();

  // device‑side API
  void register_listener(uint32_t id, uint32_t mask, RxCallback cb);
  void register_fd_listener(uint32_t id, uint32_t mask, FdRxCallback cb);
  void enqueue_tx(const struct can_frame &fr);
  void enqueue_tx(const struct canfd_frame &fr);
  bool transmit_last(const struct can_frame &fr);

  uint64_t dropped_tx() const noexcept { return dropped_tx_; }

private:
  struct Listener {
    uint32_t id, mask;
    RxCallback cb;
  };
  struct FdListener {
    uint32_t id, mask;
    FdRxCallback cb;
  };
  struct TxFrame {
    struct canfd_frame frame {};
    size_t mtu {CANFD_MTU};
  };

  // worker helpers
  void io_loop_();
  void flush_tx_();
  void rx_once_();

  // members
  int fd_{-1};
  int wake_pipe_[2]{-1, -1};
  std::thread io_th_;
  std::atomic_bool running_{false};

  std::string iface_{};

  std::vector<Listener> listeners_;
  std::vector<FdListener> fd_listeners_;
  std::mutex lst_mtx_;

  std::queue<TxFrame> tx_q_;
  std::mutex tx_mtx_;
  std::mutex write_mtx_;
  std::atomic_bool final_tx_requested_{false};
  std::atomic_uint64_t dropped_tx_{0};
};

// -------------- Inline implementation ----------------
inline bool CanBusManager::start(const std::string &iface) {
  if (running_)
    return true;
  iface_ = iface;

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    perror("socket");
    return false;
  }

  const int enable_can_fd = 1;
  if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_can_fd,
                 sizeof(enable_can_fd)) < 0 &&
      errno != ENOPROTOOPT) {
    perror("CAN_RAW_FD_FRAMES");
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  struct sockaddr_can addr {};
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    perror("SIOCGIFINDEX");
    goto err;
  }
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("bind");
    goto err;
  }

  if (pipe2(wake_pipe_, O_NONBLOCK) < 0) {
    perror("pipe2");
    goto err;
  }

  running_ = true;
  io_th_ = std::thread(&CanBusManager::io_loop_, this);
  return true;
err:
  if (fd_ >= 0)
    ::close(fd_);
  fd_ = -1;
  return false;
}

inline void CanBusManager::stop() {
  running_ = false;
  if (io_th_.joinable())
    io_th_.join();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  if (wake_pipe_[0] >= 0) {
    ::close(wake_pipe_[0]);
    ::close(wake_pipe_[1]);
    wake_pipe_[0] = wake_pipe_[1] = -1;
  }
}

inline void CanBusManager::register_listener(uint32_t id, uint32_t mask,
                                             RxCallback cb) {
  std::lock_guard<std::mutex> lk(lst_mtx_);
  listeners_.push_back({id, mask, std::move(cb)});
}

inline void CanBusManager::register_fd_listener(uint32_t id, uint32_t mask,
                                                FdRxCallback cb) {
  std::lock_guard<std::mutex> lk(lst_mtx_);
  fd_listeners_.push_back({id, mask, std::move(cb)});
}

inline void CanBusManager::enqueue_tx(const struct can_frame &fr) {
  TxFrame tx {};
  tx.frame.can_id = fr.can_id;
  tx.frame.len = fr.can_dlc;
  std::memcpy(tx.frame.data, fr.data, CAN_MAX_DLEN);
  tx.mtu = CAN_MTU;
  {
    std::lock_guard<std::mutex> lk(tx_mtx_);
    tx_q_.push(tx);
  }
  char one = 1;
  write(wake_pipe_[1], &one, 1);
}

inline void CanBusManager::enqueue_tx(const struct canfd_frame &fr) {
  TxFrame tx {};
  tx.frame = fr;
  tx.mtu = CANFD_MTU;
  {
    std::lock_guard<std::mutex> lk(tx_mtx_);
    tx_q_.push(tx);
  }
  char one = 1;
  write(wake_pipe_[1], &one, 1);
}

inline bool CanBusManager::transmit_last(const struct can_frame &fr) {
  final_tx_requested_ = true;

  TxFrame tx {};
  tx.frame.can_id = fr.can_id;
  tx.frame.len = fr.can_dlc;
  std::memcpy(tx.frame.data, fr.data, CAN_MAX_DLEN);
  tx.mtu = CAN_MTU;

  {
    std::lock_guard<std::mutex> lk(tx_mtx_);
    std::queue<TxFrame> empty;
    tx_q_.swap(empty);
  }

  std::lock_guard<std::mutex> lk(write_mtx_);
  return ::write(fd_, &tx.frame, tx.mtu) == static_cast<ssize_t>(tx.mtu);
}

inline void CanBusManager::io_loop_() {
  struct pollfd pfds[2]{{fd_, POLLIN, 0}, {wake_pipe_[0], POLLIN, 0}};
  while (running_) {
    flush_tx_();
    int n = poll(pfds, 2, 10);
    if (n < 0) {
      if (errno != EINTR) {
        // Back off on driver/socket faults to avoid spinning an entire core.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      continue;
    }
    if (n == 0)
      continue;

    bool handled = false;
    if (pfds[0].revents & POLLIN) {
      rx_once_();
      handled = true;
    }
    if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
      handled = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (pfds[1].revents & POLLIN) {
      char buf[16];
      while (read(wake_pipe_[0], buf, sizeof(buf)) > 0) {
      }
      handled = true;
    }
    if (pfds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) {
      handled = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!handled) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

inline void CanBusManager::flush_tx_() {
  TxFrame tx {};
  for (;;) {
    {
      std::lock_guard<std::mutex> lk(tx_mtx_);
      if (tx_q_.empty())
        break;
      tx = tx_q_.front();
      tx_q_.pop();
    }
    std::lock_guard<std::mutex> write_lk(write_mtx_);
    if (final_tx_requested_) {
      continue;
    }
    if (::write(fd_, &tx.frame, tx.mtu) != static_cast<ssize_t>(tx.mtu))
      dropped_tx_++;
  }
}

inline void CanBusManager::rx_once_() {
  struct canfd_frame fd_fr {};
  const auto nbytes = ::read(fd_, &fd_fr, CANFD_MTU);
  if (nbytes != CAN_MTU && nbytes != CANFD_MTU)
    return;

  std::lock_guard<std::mutex> lk(lst_mtx_);
  if (nbytes == CAN_MTU) {
    struct can_frame fr {};
    fr.can_id = fd_fr.can_id;
    fr.can_dlc = fd_fr.len;
    std::memcpy(fr.data, fd_fr.data, CAN_MAX_DLEN);
    for (const auto &l : listeners_)
      if ((fr.can_id & l.mask) == l.id)
        l.cb(fr);
  }

  for (const auto &l : fd_listeners_)
    if ((fd_fr.can_id & l.mask) == l.id)
      l.cb(fd_fr);
}
