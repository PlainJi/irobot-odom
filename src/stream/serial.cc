#include "stream.h"

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <linux/serial.h>

#include <iostream>

class StreamSerial : public Stream {
  public:
    StreamSerial(std::string device_name, speed_t baud_rate);
    ~StreamSerial();

    virtual bool Connect();
    virtual bool Disconnect();
    virtual size_t Read(uint8_t *buffer, size_t length, size_t buffer_size);
    virtual size_t Write(const uint8_t *buffer, size_t length);

  private:
    void OpenSerial();
    bool ConfigSerial();
    void CloseSerial();

    std::string device_name_;
    speed_t baud_rate_;
    char parity_;           // 'N' 'n' 'O' 'o' 'E' 'e'
    uint32_t byte_size_;
    uint32_t stop_bits_;
    int fd_;
};

speed_t get_baudrate(uint32_t baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B0;  // hang up
  }
}

StreamSerial::StreamSerial(std::string device_name, speed_t baud_rate):
    device_name_(device_name),
    baud_rate_(baud_rate),
    parity_('N'),
    byte_size_(8),
    stop_bits_(1),
    fd_(-1) {
  if (device_name_.empty()) {
    status_ = Stream::Status::ERROR;
  }
}

StreamSerial::~StreamSerial() {
  CloseSerial();
}

void StreamSerial::OpenSerial() {
  int fd = -1;
  //if ((fd = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
  if ((fd = open(device_name_.c_str(), O_RDWR | O_NOCTTY)) < 0) {
    if (errno == EINTR) {
      // Recurse because it is a recoverable error.
      return OpenSerial();
    } else {
      std::cout << "Open device " << device_name_ << " failed, error: " \
        << strerror(errno) << std::endl;
        return;
    }
  }
  fd_ = fd;

  // ttyUSB0/1/2 of novatel connect via micro-usb needn't config serial 
  // while usb2serial cable need it
  if (baud_rate_) {
    if (ConfigSerial()<0) {
      close(fd_);
      fd_ = -1;
      return;
    }
  }

  std::cout << "serial opened! fd = " << fd_ << std::endl;
}

bool StreamSerial::ConfigSerial() {
  struct termios newtio,oldtio;

  if( tcgetattr(fd_, &oldtio)  !=  0) {
    std::cout << "tcgetattr failed! error: " << strerror(errno) << std::endl;
    return -1;
  }
  bzero( &newtio, sizeof( newtio ) );
  newtio.c_cflag  |=  CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  cfsetispeed(&newtio, get_baudrate(baud_rate_));
  cfsetospeed(&newtio, get_baudrate(baud_rate_));

  switch( byte_size_ ) {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }

  switch( parity_ ) {
    case 'o':
    case 'O':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'e':
    case 'E':
      newtio.c_iflag |= (INPCK | ISTRIP);
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      break;
    case 'n':
    case 'N':
      newtio.c_cflag &= ~PARENB;
      break;
    default:
      break;
  }

  if( 1 == stop_bits_ )
    newtio.c_cflag &=  ~CSTOPB;
  else if ( 2 == stop_bits_ )
    newtio.c_cflag |=  CSTOPB;

  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN] = 1;

  tcflush(fd_, TCIFLUSH);
  if((tcsetattr(fd_, TCSANOW, &newtio))!=0) {
    std::cout << "tcsetattr failed! error: " << strerror(errno) << std::endl;
    return -2;
  }   

  std::cout << "Set COM: " << "speed=" << baud_rate_ << " parity=" << parity_ << \
    " byte_size=" << byte_size_ << " stopbits=" << stop_bits_ << std::endl;
}

void StreamSerial::CloseSerial() {
  if (fd_>0) {
    close(fd_);
    fd_ = -1;
    status_ = Stream::Status::DISCONNECT;
  }
}

bool StreamSerial::Connect() {
  if (fd_<=0) {
    OpenSerial();
    if (fd_<=0) {
      status_ = Stream::Status::ERROR;
      std::cout << "Connect " << device_name_ << " failed!" << std::endl;
      // sleep 1 sec, prevent reconnect too fast.
      sleep(1);
      return false;
    }
  }

  status_ == Stream::Status::CONNECT;
  std::cout << "Connect " << device_name_ << " succeed!" << std::endl;
  return true;
}

bool StreamSerial::Disconnect() {
  if (fd_<=0) {
    std::cout << "Disconnect " << device_name_ << " failed!" << std::endl;
    return false;
  }

  CloseSerial();
  std::cout << "Disconnect " << device_name_ << " succeed!" << std::endl;
  return true;
}

size_t StreamSerial::Write(const uint8_t *buffer, size_t length) {
  if(fd_<=0) {
    if (!Connect()) {
      return 0;
    }
  }

  size_t send_total = 0;

  while(length>0) {
    size_t send_bytes = write(fd_, buffer, length);
    if (send_bytes < 0) {
      std::cout << "serial write failed, error: " << strerror(errno) << std::endl;
      switch (errno) {
        // usually occurs in O_NONBLOCK mode, indicating that no data currently
        case EAGAIN:
        case EINVAL:
          send_bytes = 0;
          break;
        // I/O error
        case EBADF:
        case EIO:
          Disconnect();
          if (Connect()) {
            std::cout << "Reconnect " << device_name_ << "succeed!" << std::endl;
            send_bytes = 0;
            break;    // has been recovered
          }
        // Reconnect failed and other error
        default:
          std::cout << "Serial write failed!" << std::endl;
          status_ = Stream::Status::ERROR;
          return send_total;
      }
    } else if (!send_bytes) {
      if (!WaitForReadWrite(fd_, 10000, Stream::RW::RW_WRITE)) {
        break;
      }
      continue;
    }
    send_total += send_bytes;
    length -= send_bytes;
    buffer += send_bytes;
  }
  return send_total;
}

// user should 
size_t StreamSerial::Read(uint8_t *buffer, size_t length, size_t buffer_size) {
  if (length > buffer_size) {
    std::cout << "Serial Read buffer is not enough!" << std::endl;
    return 0;
  }

  if(fd_<=0) {
    if (!Connect()) {
      return 0;
    }
  }

  size_t read_total = 0;

  while (length>0) {
    size_t read_bytes = read(fd_, buffer, length);
    if (read_bytes < 0) {
      std::cout << "serial write failed, error: " << strerror(errno) << std::endl;
      switch (errno) {
        // usually occurs in O_NONBLOCK mode, indicating that no data currently
        case EAGAIN:
        case EINVAL:
          read_bytes = 0;
          break;
        // I/O error
        case EBADF:
        case EIO:
          Disconnect();
          if (Connect()) {
            std::cout << "Reconnect " << device_name_ << "succeed!" << std::endl;
            read_bytes = 0;
            break;    // has been recovered
          }
        // Reconnect failed and other error
        default:
          std::cout << "Serial read failed!" << std::endl;
          status_ = Stream::Status::ERROR;
          return read_total;
      }
    } else if (!read_bytes) {
      if (!WaitForReadWrite(fd_, 10000, Stream::RW::RW_READ)) {
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

Stream* Stream::Serial(const std::string device_name, uint32_t baud_rate) {
  return new StreamSerial(device_name, static_cast<speed_t>(baud_rate));
}
