#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

//#include "v4l/v4l2.h"
#include "encoding/jpeg.h"
#include "streaming/rtp.h"
#include "streaming/udp_socket.h"

#include "image.h"

#include "settings.h"

#ifndef DOWNSIZE_FACTOR
#define DOWNSIZE_FACTOR   8
#endif

#define RTP 1

int main(int argc,char ** argv)
{
  printf("Starting video test program!\n");

  /* Video Input
  struct vid_struct vid;
  vid.device = (char*) VIDEO_DEVICE;
  vid.w=VIDEO_DEVICE_WIDTH;
  vid.h=VIDEO_DEVICE_HEIGHT;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    return 0;
  }
  */

  // Video Grabbing
  struct image_t img_new; // = video_create_image(&vid);

  // Video Resizing
  struct image_t small;
  image_create(&small,32,32,IMAGE_YUV422);

  // Video Compression
  struct image_t jpegbuf;
  image_create(&jpegbuf,32,32,IMAGE_JPEG);

  // Network Transmit
  struct UdpSocket sock;
  #define FMS_UNICAST 0
  #define FMS_BROADCAST 1
  int ret = udp_socket_create(&sock, RTP_TARGET_IP, 5000, -1, FMS_UNICAST);

  while (1) {

    uint8_t with_header = 0;

    // Aquire image
    printf("Aquiring an image ...\n");
    //video_grab_image(&vid, img_new);


    // Resize: device by 4
    image_yuv422_downsample(&img_new, &small, DOWNSIZE_FACTOR);

    // JPEG encode the image:
    uint32_t quality_factor = 6; // quality factor from 1 (high quality) to 8 (low quality)
    jpeg_encode_image (&small, &jpegbuf, quality_factor, with_header);

    printf("Sending an image ...%u\n",jpegbuf.buf_size);
#if RTP == 1
//#pragma message "sending over RTP"
    rtp_frame_send(&sock, &jpegbuf,0, 30, with_header, 2500);
#else
//#pragma message "sending over UDP+SVS Header"
    size = create_svs_jpeg_header(jpegbuf,size,small.w);
    udp_write(sock,jpegbuf,size);
#endif
  }

  //video_close(&vid);

  return 0;
}
