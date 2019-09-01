#include "stream.h"

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include <iostream>

class StreamSocket: public Stream {
  public:
    StreamSocket(std::string ip, uint16_t port, Stream::SocketType socket_type);
    ~StreamSocket();

    virtual bool Connect();
    virtual bool Disconnect();
    virtual size_t Read(uint8_t *buffer, size_t length, size_t buffer_size);
    virtual size_t Write(const uint8_t *buffer, size_t length);

  private:
    void OpenSocket();
    void CloseSocket();

    std::string peer_ip_;
    uint16_t peer_port_;
    Stream::SocketType socket_type_;
    int sockfd_;
};

StreamSocket::StreamSocket(std::string ip, uint16_t port, Stream::SocketType socket_type) {
  sockfd_ = -1;
  peer_ip_ = ip;
  peer_port_ = port;
  socket_type_ = socket_type;
  if (!strlen(peer_ip_.c_str()) || !peer_port_ || \
    socket_type_ < Stream::SocketType::SOCKET_TCP ||
    socket_type_ >= Stream::SocketType::SOCKET_CNT) {
    status_ = Stream::Status::ERROR;
  }
}

StreamSocket::~StreamSocket() {
  CloseSocket();
}

void StreamSocket::OpenSocket() {
  int fd = -1;
  if (socket_type_ == Stream::SocketType::SOCKET_TCP) {
    fd = socket(AF_INET, SOCK_STREAM, 0);
  } else if (socket_type_ == Stream::SocketType::SOCKET_UDP) {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
  }
  if (fd<0) {
    std::cout << "creat socket failed, errno: " << strerror(errno) << std::endl;
    return;
  }

  sockfd_ = fd;
  std::cout << "socket created! fd = " << sockfd_ << std::endl;
}

void StreamSocket::CloseSocket() {
  if (sockfd_ > 0) {
    std::cout << "close socket. fd=" << sockfd_ << std::endl;
    close(sockfd_);
    sockfd_ = -1;
    status_ = Stream::Status::DISCONNECT;
  }
}

bool StreamSocket::Connect() {
  if (sockfd_ <= 0) {
    OpenSocket();
    if (sockfd_ <= 0) {
      return false;
    }
  }

  if (status_ == Stream::Status::CONNECT) {
    return true;
  }

  fd_set fds;
  timeval time_out = {10, 0};
  int ret = 0;
  sockaddr_in peer_addr;
  bzero(&peer_addr, sizeof(peer_addr));
  peer_addr.sin_family = AF_INET;
  peer_addr.sin_addr.s_addr = inet_addr(peer_ip_.c_str());
  peer_addr.sin_port = htons(peer_port_);

  // set as block mode
  int flags = fcntl(sockfd_, F_GETFL, 0);
  if (flags == -1) {
    close(sockfd_);
    sockfd_ = -1;
    std::cout << "setsockopt F_GETFL failed, errno: " << \
      strerror(errno) << std::endl;
    return false;
  }
  if (fcntl(sockfd_, F_SETFL, flags & ~O_NONBLOCK) == -1) {
    close(sockfd_);
    std::cout << "fcntl set block failed, errno: " << strerror(errno) << std::endl;
    return false;
  }

  // disable Nagle of TCP
  int enable = 1;
  if (setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, 
    reinterpret_cast<void*>(&enable), sizeof(enable)) < 0) {
    std::cout << "setsockopt TCP_NODELAY failed, errno: " << \
      strerror(errno) << std::endl;
    status_ = Stream::Status::ERROR;
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  // set SO_LINGER
  linger lin;
  uint32_t l=sizeof(lin);
  lin.l_onoff=1;
  lin.l_linger=1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_LINGER, reinterpret_cast<void*>(&lin), l)<0) {
    std::cout << "setsockopt SO_LINGER failed, errno: " << \
      strerror(errno) << std::endl;
    status_ = Stream::Status::ERROR;
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  // connect until succeed or handless error
  while ((ret = connect(sockfd_, reinterpret_cast<sockaddr*>(&peer_addr), sizeof(peer_addr)))<0) {
    std::cout << "connect failed, error: " << strerror(errno) << std::endl;
    sleep(1);
  }

  if (socket_type_ == Stream::SocketType::SOCKET_TCP) {
    std::cout << Stream::SocketTypeStr[static_cast<int>(socket_type_)] << \
      "TCP: connect success. " << peer_ip_ << ":" << peer_port_ << std::endl;
  } else if (socket_type_ == Stream::SocketType::SOCKET_UDP) {
    std::cout << Stream::SocketTypeStr[static_cast<int>(socket_type_)] << \
      "UDP: Server IP&Port has been bind!" << std::endl;
  }
  status_ = Stream::Status::CONNECT;
  return true;
}

bool StreamSocket::Disconnect() {
  if (sockfd_<=0) {
    std::cout << Stream::SocketTypeStr[static_cast<int>(socket_type_)] << " Disconnect failed! " << \
      peer_ip_ << ":" << peer_port_ << " not open!" << std::endl;
    return false;
  }

  CloseSocket();
  std::cout << Stream::SocketTypeStr[static_cast<int>(socket_type_)] << " Disconnect succeed! " << \
    peer_ip_ << ":" << peer_port_ << std::endl;
  return true;
}

size_t StreamSocket::Read(uint8_t *buffer, size_t length, size_t buffer_size) {
  if (length > buffer_size) {
    std::cout << "Socket Read buffer is not enough!" << std::endl;
    return 0;
  }
  if (status_ != Stream::Status::CONNECT) {
    Connect();
    if (status_ != Stream::Status::CONNECT) {
      return 0;
    }
  }
  size_t read_total = 0;
  while(length>0) {
    int read_bytes = read(sockfd_, buffer, length);
    // handle error
    if (read_bytes < 0) {
      std::cout << "Socket read failed, error: " << strerror(errno) << std::endl;
      switch(errno) {
        case EINTR:
        case EAGAIN:
          read_bytes = 0;
          break;
        default:
          std::cout << "Socket read failed!" << std::endl;
          status_ = Stream::Status::ERROR;
          Disconnect();
          return read_total;
      }
    // no data current
    } else if (!read_bytes) {
      if (!WaitForReadWrite(sockfd_, 10000, Stream::RW::RW_READ)) {
        break;
      }
      continue;
    }
    read_total += read_bytes;
    buffer += read_bytes;
    length -= read_bytes;
  }
  return read_total;
}

size_t StreamSocket::Write(const uint8_t *buffer, size_t length) {
  if (status_ != Stream::Status::CONNECT) {
    Connect();
    if (status_ != Stream::Status::CONNECT) {
      return 0;
    }
  }
  size_t write_total = 0;
  while (length > 0) {
    ssize_t write_bytes = write(sockfd_, buffer, length);
    if (write_bytes < 0) {
      std::cout << "Socket write failed, error: " << strerror(errno) << std::endl;
      switch (errno) {
        case EINTR:
        case EAGAIN:
          write_bytes = 0;
          break;
        default:
          status_ = Stream::Status::ERROR;
          Disconnect();
          return write_total;
      }
    } else if (!write_bytes) {
      if (!WaitForReadWrite(sockfd_, 10000, Stream::RW::RW_WRITE)) {
        break;
      }
      continue;
    }
    write_total += write_bytes;
    length -= write_bytes;
    buffer += write_bytes;
  }

  return write_total;
}

Stream* Stream::Socket(const std::string address, uint16_t port, \
    Stream::SocketType socket_type) {
  return new StreamSocket(address, port, socket_type);
}
