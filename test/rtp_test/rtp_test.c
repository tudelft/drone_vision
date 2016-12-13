#include <unistd.h> // usleep
#include <stdio.h> // printf

#include "streaming/rtp.h"
#include "streaming/udp_socket.h"


int main(int argc,char ** argv)
{
  printf("Starting video test program!\n");

  // Network Transmit
  struct UdpSocket video_sock;

  // Open udp socket
  udp_socket_create(&video_sock, "127.0.0.1", 5000, -1, 0);

  long cnt = 0;
  while (1) {

    // Aquire image
    printf("Aquiring image %ld...\n",cnt);
    cnt++;

    rtp_frame_test(&video_sock);
    unsigned char buff[128];
    int len = 0;
    len = udp_socket_recv_dontwait(&video_sock,buff,128);
    if (len > 0)
    	printf("read %d\n",len);
    usleep(20000);
  }

  return 0;
}
