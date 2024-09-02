#include <arpa/inet.h>
#include <asm-generic/socket.h>
#include <bits/types/struct_timeval.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#include <time.h>
#include <unistd.h>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>

#include "ring_buffer.h"
#include "uart_parser.h"

#define SERVER_PORT 8888
#define SERVER_IP "192.168.0.6"
#define BUFLEN 64

const char *interface = "enp3s0f3u2";

int main(int argc, char *argv[]) {
  RingBuffer_t RingBuffer = {0};
  uint8_t ReceivedLine = 0;
  uint8_t ReceivedData[64] = {0};

  struct sockaddr_in sa_int, sa_ext;

  uint8_t serial_recv_buf;

  struct eth_data send_data[64] = {0};
  struct eth_data recv_data[64] = {0};

  uint8_t parse_eth = 0;

  socklen_t peer_addr_len = sizeof(sa_ext);

  srand(time(NULL)); // Fill seed with current time

  /* Create UDP socket */
  int sock_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (sock_fd == -1) {
    printf("Socket creation failed");
    exit(1);
  }

  if (setsockopt(sock_fd, SOL_SOCKET, SO_BINDTODEVICE, interface,
                 strlen(interface)) < 0) {
    printf("Failed to set interface bind\n");
    exit(1);
  }

  /* Set the sockaddr_in sa_ext structure */
  memset((char *)&sa_ext, 0, sizeof(sa_ext));
  sa_ext.sin_family = AF_INET;
  sa_ext.sin_port = htons(SERVER_PORT);

  if (inet_aton(SERVER_IP, &sa_ext.sin_addr) == 0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }

  /* Open ST-Link Uart device */
  int serial_port = open("/dev/ttyACM0", O_RDWR);

  if (serial_port < 0) {
    printf("Failed to open /dev/ttyACM0 device: %s\n", strerror(errno));
  }

  struct termios tty;

  if (tcgetattr(serial_port, &tty) != 0) {
    printf("Failed to get ttyACM0 attributes: %s\n", strerror(errno));
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_lflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 1;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Failed to save ttyACM0 settings: %s\n", strerror(errno));
    exit(1);
  }

  tcflush(serial_port, TCIOFLUSH);
  uint8_t j = 0;
  uint8_t tmp = 0;

  while (1) {
    read(serial_port, &serial_recv_buf, 1);

    RB_Write(&RingBuffer, serial_recv_buf);

    if (serial_recv_buf == ENDLINE) {
      ReceivedLine++;
    }

    // printf("%c\n", serial_recv_buf);

    if (ReceivedLine > 0) {
      parse_line(&RingBuffer, ReceivedData);

      printf("ReceivedData: %s\n", ReceivedData);
      if (parse_eth == 1) {
        parse_eth_data(ReceivedData, &recv_data[j]);
        j++;
        if (j >= 64) {
          parse_eth = 0;
        }
      }
      ReceivedLine--;
    }

    if (strcmp((char *)ReceivedData, "Starting") == 0) {
      for (uint8_t i = 0; i < 64; i++) {
        send_data[i].counter = i;
        send_data[i].random_data = rand();
        if (sendto(sock_fd, &send_data[i], sizeof(send_data), 0,
                   (struct sockaddr *)&sa_ext, peer_addr_len) == -1) {
          printf("sendto failed\n");
        }
        usleep(15000);
      }
      memset(&ReceivedData, 0, sizeof(ReceivedData));
    } else if (strcmp((char *)ReceivedData, "Retrieve") == 0) {
      parse_eth = 1;
      memset(&ReceivedData, 0, sizeof(ReceivedData));
    } else if (strcmp((char *)ReceivedData, "Compare") == 0) {
      parse_eth = 0;
      for (uint8_t i = 0; i < 64; i++) {
        if ((send_data[i].counter == recv_data[i].counter) &&
            (send_data[i].random_data == recv_data[i].random_data)) {
          tmp++;
        }
      }
      if (tmp == 64) {
        unsigned char msg[] = {'M', 'A', 'T', 'C', 'H', '\n'};
        write(serial_port, msg, sizeof(msg));
      } else {
        unsigned char msg[] = {'M', 'I', 'S', 'M', 'A', 'T', 'C', 'H', '\n'};
        write(serial_port, msg, sizeof(msg));
      }
      memset(&ReceivedData, 0, sizeof(ReceivedData));
    }
  }
  close(sock_fd);
  return EXIT_SUCCESS;
}
