#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#include "v4l/v4l2.h"
#include "encoding/jpeg.h"
#include "encoding/rtp.h"
#include "udp_socket.h"

#include "lib/vision/image.h"

#include "settings.h"

#include "video_thread.h"

#ifndef DOWNSIZE_FACTOR
#define DOWNSIZE_FACTOR   8
#endif

#define RTP 1

int main(int argc,char ** argv)
{
  printf("Starting video test program!\n");

  // Video Input
  struct vid_struct vid;
  vid.device = (char*) VIDEO_DEVICE;
  vid.w=VIDEO_DEVICE_WIDTH;
  vid.h=VIDEO_DEVICE_HEIGHT;
  vid.n_buffers = 4;
  if (video_init(&vid)<0) {
    printf("Error initialising video\n");
    return 0;
  }

  // Video Grabbing
  struct img_struct* img_new = video_create_image(&vid);

  // Video Resizing
  struct img_struct small;
  small.w = vid.w / DOWNSIZE_FACTOR;
  small.h = vid.h / DOWNSIZE_FACTOR;
  small.buf = (uint8_t*)malloc(small.w*small.h*2);

  // Video Compression
  uint8_t* jpegbuf = (uint8_t*)malloc(vid.h*vid.w*2);


  // Network Transmit
  struct UdpSocket* sock;
  //#define FMS_UNICAST 0
  //#define FMS_BROADCAST 1
  sock = udp_socket(RTP_TARGET_IP, 5000, 5001, FMS_BROADCAST);

  while (1) {

    uint8_t with_header = 0;

    // Aquire image
    printf("Aquiring an image ...\n");
    video_grab_image(&vid, img_new);


    // Resize: device by 4
    resize_uyuv(img_new, &small, DOWNSIZE_FACTOR);

    // JPEG encode the image:
    uint32_t quality_factor = 6; // quality factor from 1 (high quality) to 8 (low quality)
    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
    uint8_t* end = encode_image (small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, with_header);
    uint32_t size = end-(jpegbuf);

    printf("Sending an image ...%u\n",size);
#if RTP == 1
#pragma message "sending over RTP"
    send_rtp_frame(sock, jpegbuf,size, small.w, small.h,0, 30, with_header, 2500);
#else
#pragma message "sending over UDP+SVS Header"
    size = create_svs_jpeg_header(jpegbuf,size,small.w);
    udp_write(sock,jpegbuf,size);
#endif
  }

  video_close(&vid);

  return 0;
}
