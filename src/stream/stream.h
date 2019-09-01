#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/select.h>
#include <string>

class Stream {
  public:
    virtual ~Stream() {}

    enum class Status {
      DISCONNECT = 0,
      CONNECT,
      ERROR,
      STATUS_CNT,
    };
    enum class RW {
      RW_READ = 0,
      RW_WRITE,
      RW_CNT,
    };
    enum class SocketType {
      SOCKET_TCP = 0,
      SOCKET_UDP,
      SOCKET_CNT,
    };
    const char *SocketTypeStr[static_cast<int>(SocketType::SOCKET_CNT)] = {"SOCKET_TCP", "SOCKET_UDP"};

    Status status_ = Status::DISCONNECT;

    static Stream *Serial(const std::string device_name, uint32_t baud_rate);
    static Stream *Socket(const std::string address, uint16_t port, SocketType socket_type);

    virtual bool Connect() = 0;
    virtual bool Disconnect() = 0;
    virtual size_t Read(uint8_t *buffer, size_t length, size_t buffer_size) = 0;
    virtual size_t Write(const uint8_t *buffer, size_t length) = 0;
    inline size_t ReadLine(uint8_t *buffer, size_t buffer_size) {
      uint8_t temp = 0; size_t recv_cnt = 0;
      while(temp != '\n') {
        if (1!=Read(&temp, 1, 1)) {break;}
        buffer[recv_cnt++] = temp;
      }
      return recv_cnt;}
    inline size_t Write(const std::string &buffer) {
      return Write(reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());}

    bool WaitForReadWrite(int fd, uint32_t timeout_us, RW type) {
      int ret = 0;
      timespec timeout;
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(fd, &fds);
      timeout.tv_sec = timeout_us / 1000000;
      timeout.tv_nsec = (timeout_us % 1000000) * 1000;
      if (type == RW::RW_READ) {ret = pselect(fd+1, &fds, NULL, NULL, &timeout, NULL);}
      else if (type == RW::RW_WRITE) {ret = pselect(fd+1, NULL, &fds, NULL, &timeout, NULL);}
      else {return false;}
      if (ret<=0) {return false;}               // error or timeout
      if (!FD_ISSET(fd, &fds)) {return false;}  // double check
      return true;
    }
  protected:
    Stream() {}
};
