// =============================================================
// can_bus_manager.hpp  —  SocketCAN poll‑loop manager
// =============================================================
#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
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
class CanBusManager : public std::enable_shared_from_this<CanBusManager>
{
public:
  using RxCallback = std::function<void(const struct can_frame&)>;

  CanBusManager()  = default;
  ~CanBusManager() { stop(); }

  CanBusManager(const CanBusManager&)            = delete;
  CanBusManager& operator=(const CanBusManager&) = delete;

  bool start(const std::string& iface, int bitrate);
  void stop();

  // device‑side API
  void register_listener(uint32_t id, uint32_t mask, RxCallback cb);
  void enqueue_tx(const struct can_frame& fr);

  uint64_t dropped_tx() const noexcept { return dropped_tx_; }

private:
  struct Listener { uint32_t id, mask; RxCallback cb; };

  // worker helpers
  void io_loop_();
  void flush_tx_();
  void rx_once_();

  // members
  int  fd_{-1};
  int  wake_pipe_[2]{-1, -1};
  std::thread      io_th_;
  std::atomic_bool running_{false};

  std::string iface_{}; int bitrate_{0};

  std::vector<Listener> listeners_;
  std::mutex            lst_mtx_;

  std::queue<struct can_frame> tx_q_;
  std::mutex                   tx_mtx_;
  std::atomic_uint64_t         dropped_tx_{0};
};

// -------------- Inline implementation ----------------
inline bool CanBusManager::start(const std::string& iface, int bitrate)
{
  if (running_) return true;
  iface_ = iface; bitrate_ = bitrate;

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) { perror("socket"); return false; }

  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

  struct ifreq ifr{}; std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ-1);
  struct sockaddr_can addr{};
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); goto err; }
  addr.can_family=AF_CAN; addr.can_ifindex=ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) { perror("bind"); goto err; }

  if (pipe2(wake_pipe_, O_NONBLOCK) < 0) { perror("pipe2"); goto err; }

  running_ = true;
  io_th_   = std::thread(&CanBusManager::io_loop_, this);
  return true;
err:
  if (fd_>=0) ::close(fd_);
  fd_=-1; return false;
}

inline void CanBusManager::stop()
{
  running_ = false;
  if (io_th_.joinable()) io_th_.join();
  if (fd_>=0) { ::close(fd_); fd_=-1; }
  if (wake_pipe_[0]>=0) { ::close(wake_pipe_[0]); ::close(wake_pipe_[1]); wake_pipe_[0]=wake_pipe_[1]=-1; }
}

inline void CanBusManager::register_listener(uint32_t id, uint32_t mask, RxCallback cb)
{ std::lock_guard<std::mutex> lk(lst_mtx_); listeners_.push_back({id,mask,std::move(cb)}); }

inline void CanBusManager::enqueue_tx(const struct can_frame& fr)
{
  { std::lock_guard<std::mutex> lk(tx_mtx_); tx_q_.push(fr); }
  char one=1; write(wake_pipe_[1], &one, 1);
}

inline void CanBusManager::io_loop_()
{
  struct pollfd pfds[2]{{fd_,POLLIN,0},{wake_pipe_[0],POLLIN,0}};
  while(running_){
    flush_tx_();
    int n=poll(pfds,2,10);
    if(n<=0) continue;
    if(pfds[0].revents&POLLIN) rx_once_();
    if(pfds[1].revents&POLLIN){ char buf[16]; read(wake_pipe_[0],buf,sizeof(buf)); }
  }
}

inline void CanBusManager::flush_tx_()
{
  struct can_frame fr{};
  for(;;){
    { std::lock_guard<std::mutex> lk(tx_mtx_); if(tx_q_.empty()) break; fr=tx_q_.front(); tx_q_.pop(); }
    if(::write(fd_,&fr,CAN_MTU)!=CAN_MTU) dropped_tx_++; }
}

inline void CanBusManager::rx_once_()
{
  struct can_frame fr{}; if(::read(fd_,&fr,CAN_MTU)!=CAN_MTU) return;
  std::lock_guard<std::mutex> lk(lst_mtx_);
  for(const auto &l: listeners_) if((fr.can_id&l.mask)==l.id) l.cb(fr);
}

