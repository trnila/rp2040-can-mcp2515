#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <list>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

std::ostream &operator<<(std::ostream &out, const struct can_frame &frame) {
  out << "Frame can_id=0x" << std::hex << frame.can_id
      << " can_dlc=" << std::dec << (size_t)frame.can_dlc
      << " data=" << std::hex << std::setfill('0') << std::setw(2);

  for (size_t i = 0; i < frame.can_dlc; i++) {
    out << (int)frame.data[i] << " ";
  }
  return out << std::dec;
}

int can_open(const char *name) {
  int s;
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("socket");
    exit(1);
  }

  struct ifreq ifr;
  strcpy(ifr.ifr_name, name);
  ioctl(s, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr = {0};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    exit(1);
  }

  // set buffer smaller than hardware one, otherwise we get ENOBUFS and POLLOUT
  // will not work
  int sndbuf = 8;
  if (setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) < 0) {
    perror("setsockopt");
  }

  return s;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " can_tx_iface can_rx_iface\n";
    exit(1);
  }

  struct pollfd fds[2] = {0};
  fds[0].fd = can_open(argv[1]);
  fds[0].events = POLLOUT;
  fds[1].fd = can_open(argv[2]);
  fds[1].events = POLLIN;

  std::list<struct can_frame> frames;

  struct stats {
    uint64_t tx;
    uint64_t rx;
  };

  stats stats_prev{};
  stats stats_cur{};
  std::chrono::steady_clock::time_point last_stats =
      std::chrono::steady_clock::now();

  for (;;) {
    if (poll(fds, sizeof(fds) / sizeof(*fds), -1) < 0) {
      perror("poll");
      exit(1);
    }

    if (fds[0].revents & POLLOUT) {
      struct can_frame frame = {0};
      frame.can_id = rand() % 0x20000000;
      // set extended format if ID > 11 bits or with 50% probability
      if (frame.can_id >= 0x800 || rand() % 2 == 1) {
        frame.can_id |= 1U << 31;
      }
      frame.can_dlc = rand() % 9;
      for (size_t i = 0; i < frame.can_dlc; i++) {
        frame.data[i] = rand() % 256;
      }

      if (write(fds[0].fd, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write");
        exit(1);
      }

      frames.push_back(frame);
      stats_cur.tx++;
    }

    if (fds[1].revents & POLLIN) {
      struct can_frame rd = {0};
      if (read(fds[1].fd, &rd, sizeof(rd)) != sizeof(rd)) {
        perror("read");
        exit(1);
      }
      stats_cur.rx++;
      auto found =
          std::find_if(frames.begin(), frames.end(), [&rd](const can_frame &f) {
            return f.can_id == rd.can_id && f.can_dlc == rd.can_dlc &&
                   memcmp(f.data, rd.data, rd.can_dlc) == 0;
          });

      if (found == frames.end()) {
        std::cout << "Frame not found: " << rd << '\n';
        exit(1);
      } else {
        frames.erase(found);
      }
    }

    auto now = std::chrono::steady_clock::now();
    double diff =
        std::chrono::duration_cast<std::chrono::seconds>(now - last_stats)
            .count();
    if (diff >= 1) {
      std::cout << "tx=" << (stats_cur.tx - stats_prev.tx) / diff << "/s "
                << "rx=" << (stats_cur.rx - stats_prev.rx) / diff << "/s "
                << "buffer=" << frames.size() << '\n';

      stats_prev = stats_cur;
      last_stats = now;
    }
  }
}
