#pragma once
#include <webots/Lidar.hpp>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <time.h>

struct KissPoint { float x, y, z, time; };

class KissSlamClient {
 public:
  explicit KissSlamClient(std::string path) : path_(std::move(path)) {}
  ~KissSlamClient() { closeSocket(); }
  bool connect() {
    if (fd_ >= 0) return true;
    const double now = monotonicSeconds();
    if (now - lastAttempt_ < 1.0) return false;
    lastAttempt_ = now;
    int fd = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (fd < 0) return false;
    sockaddr_un addr{}; addr.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(addr.sun_path)) { ::close(fd); return false; }
    std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path)-1);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(fd); return false;
    }
    fd_ = fd; return true;
  }
  bool connected() const { return fd_ >= 0; }
  unsigned closureCount() const { return closures_; }
  void sendScan(const std::vector<KissPoint>& points, double simTime) {
    if (points.size() < 30) return;
    if (!connect()) return;
    constexpr uint32_t magic = 0x4B495353;
    #pragma pack(push, 1)
    struct Header { uint32_t magic, version, count; double sim_time; };
    #pragma pack(pop)
    Header h{magic,1,static_cast<uint32_t>(points.size()),simTime};
    std::vector<unsigned char> packet(sizeof(h) + points.size()*sizeof(KissPoint));
    std::memcpy(packet.data(), &h, sizeof(h));
    std::memcpy(packet.data()+sizeof(h), points.data(), points.size()*sizeof(KissPoint));
    const ssize_t sent = ::send(fd_, packet.data(), packet.size(), MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(packet.size())) closeSocket();
  }
  void pollPose() {
    if (fd_ < 0) return;
    struct Reply { uint32_t magic, closures; double pose[16]; } r{};
    const ssize_t n = ::recv(fd_, &r, sizeof(r), MSG_DONTWAIT);
    if (n == 0) { closeSocket(); return; }
    if (n == static_cast<ssize_t>(sizeof(r)) && r.magic == 0x4B535250) {
      closures_ = r.closures;
      std::memcpy(pose_, r.pose, sizeof(pose_));
      havePose_ = true;
    } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) closeSocket();
  }
  bool havePose() const { return havePose_; }
  const double* pose() const { return pose_; }
 private:
  static double monotonicSeconds() {
    timespec t{}; clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<double>(t.tv_sec) + 1e-9*t.tv_nsec;
  }
  void closeSocket() { if (fd_ >= 0) ::close(fd_); fd_=-1; }
  std::string path_; int fd_=-1; double lastAttempt_=-1e9;
  unsigned closures_=0; bool havePose_=false; double pose_[16]{};
};
