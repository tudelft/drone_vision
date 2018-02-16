/*
 * filter->c
 *
 *  Created on: 24 Oct 2017
 *      Author: Kirk Scheper
 */

#include "filter.h"
#include <stdlib.h>
#include "math.h"

/**
 * Filter colors in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image
 * @param[in] *filter The filter paramerters to apply
 * @return The amount of filtered pixels
 */
uint16_t image_yuv422_colorfilt(struct image_t *input, struct image_t *output, struct image_filter_t* filter)
{
  uint16_t cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;

  // Go through all the pixels
  for (uint16_t y = 0; y < input->h; y++) {
    for (uint16_t x = 0; x < input->w; x += 2) {
      // Check if the color is inside the specified values
      if ( (source[0] >= filter->u_min)
        && (source[0] <= filter->u_max)
        && (source[2] >= filter->v_min)
        && (source[2] <= filter->v_max)
      ) {
        // UYVY
        if (source[1] >= filter->y_min && source[1] <= filter->y_max){
          dest[0] = source[0];  // U
        } else {
          dest[0] = 127;        // U
        }
        if (source[3] >= filter->y_min && source[3] <= filter->y_max){
          dest[2] = source[2];  // V
        } else {
          dest[2] = 127;        // V
        }
      } else {
        // UYVY
        dest[0] = 127;        // U
        dest[2] = 127;        // V
      }

      dest[1] = source[1];  // Y1
      dest[3] = source[3];  // Y2

      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
  return cnt;
}

/**
 * Calculate the  gradients using the following matrix:
 * dx = [0 0 0; -1 0 1; 0 0 0] and dy = [0 -1 0; 0 0 0; 0 1 0]
 * @param[in] *input Input grayscale image
 * @param[out] *dx Output gradient in the X direction
 * @param[out] *dy Output gradient in the Y direction
 */
void image_gradients(struct image_t *input, struct image_t *dx, struct image_t *dy)
{
  if (dx->buf_size < input->buf_size || dy->buf_size < input->buf_size){
    return;
  }

  // Fetch the buffers in the correct format
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint8_t *dx_buf = (int8_t *)dx->buf;
  uint8_t *dy_buf = (int8_t *)dy->buf;

  uint32_t idx;
  uint32_t size = input->w * input->h;

  // Go through all pixels except the borders
  // split computation of x and y to two loops to optimize run time performance
  for (idx = 1; idx < size - 1; idx++) {
    dx_buf[idx] = (int16_t)input_buf[idx + 1] - (int16_t)input_buf[idx - 1];
  }

  // overwrite incorrect pixels
  for (idx = dx->w-1; idx < size; idx+=dx->w) {
    dx_buf[idx] = 0;
    dx_buf[idx + 1] = 0;
  }

  for (idx = dy->w; idx < size - input->w; idx++) {
    dy_buf[idx] = (int16_t)input_buf[idx + input->w] - (int16_t)input_buf[idx - input->w];
  }
}

/* Integer implementation of square root using Netwon's method
 * Can only compute squares of numbers below 65536, ie result is always uint8_t
 */
uint8_t sqrti(int32_t num)
{
#ifdef LINUX
  uint32_t root = (uint32_t)sqrtf(float(num));
#else

  static const uint8_t max_iter = 100;
  int32_t root = num/2, prev_root = root;

  if(num <= 0){
    return 0;
  } else if (num >= 65025){ // 255 * 255 = 65025
    return 255;
  } else if (num == 1){
    return 1;
  } else {
    for(uint16_t i = 0; i < max_iter; i++){
      root = root - (root*root - num)/(root*2);
      if (root == prev_root){
        break;
      } else {
        prev_root = root;
      }
    }
  }

  // round result to nearest
  if (10*(root*root - num)/(root*2)>5){
    root -= 1;
  }
#endif
  return (uint8_t)root;
}

/**
 * Calculate the  gradients using the following matrix:
 * d = |[0 -1 0; -1 0 1; 0 1 0] * IMG|
 * @param[in] *input Input grayscale image
 * @param[out] *d Output mean gradient
 */
void image_2d_gradients(struct image_t *input, struct image_t *d)
{
  if (d->buf_size < input->buf_size){
    return;
  }

  // Fetch the buffers in the correct format
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint8_t *d_buf = (uint8_t *)d->buf;

  uint32_t idx, idx1;
  uint32_t size = input->w * input->h;
  int32_t temp1, temp2;

  // Go through all pixels except the borders
  for (idx = d->w + 1; idx < size - d->w - 1; idx++) {
    temp1 = (int32_t)input_buf[idx + 1] - (int32_t)input_buf[idx - 1];
    temp2 = (int32_t)input_buf[idx + input->w] - (int32_t)input_buf[idx - input->w];
    d_buf[idx] = sqrti(temp1*temp1 + temp2*temp2);
  }

  // set x gradient for first and last row
  for (idx = 1, idx1 = size - d->w + 1; idx1 < size - 1; idx++, idx1++) {
    d_buf[idx] = (uint8_t)abs((int16_t)input_buf[idx + 1] - (int16_t)input_buf[idx - 1]);
    d_buf[idx1] = (uint8_t)abs((int16_t)input_buf[idx1 + 1] - (int16_t)input_buf[idx1 - 1]);
  }

  // set y gradient for first and last col
  for (idx = d->w, idx1 = 2*d->w-1; idx1 < size - input->w; idx+=input->w, idx1+=input->w) {
    d_buf[idx] = (uint8_t)abs((int16_t)input_buf[idx + input->w] - (int16_t)input_buf[idx - input->w]);
    d_buf[idx1] = (uint8_t)abs((int16_t)input_buf[idx1 + input->w] - (int16_t)input_buf[idx1 - input->w]);
  }
}

/**
 * Calculate the  gradients using the following matrix:
 * dx = [-1 0 1; -2 0 2; -1 0 1] * IMG
 * dy = [-1 -2 -1; 0 0 0; 1 2 1] * IMG
 * d = sqrt(dx*dx + dy*dy)
 * @param[in] *input Input grayscale image
 * @param[out] *d Output mean gradient
 */
void image_2d_sobel(struct image_t *input, struct image_t *d)
{
  if (d->buf_size < input->buf_size){
    return;
  }

  // Fetch the buffers in the correct format
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint8_t *d_buf = (uint8_t *)d->buf;

  uint32_t idx, idx1;
  uint32_t size = input->w * input->h;
  int32_t temp1, temp2;

  // Go through all pixels except the borders
  for (idx = d->w + 1; idx < size - d->w - 1; idx++) {
    temp1 = 2*((int32_t)input_buf[idx + 1] - (int32_t)input_buf[idx - 1])
         + (int32_t)input_buf[idx + 1 - input->w] - (int32_t)input_buf[idx - 1 - input->w]
         + (int32_t)input_buf[idx + 1 + input->w] - (int32_t)input_buf[idx - 1 + input->w];
    temp2 = 2*((int32_t)input_buf[idx + input->w] - (int32_t)input_buf[idx - input->w])
        + (int32_t)input_buf[idx - 1 + input->w] - (int32_t)input_buf[idx - 1 - input->w]
        + (int32_t)input_buf[idx + 1 + input->w] - (int32_t)input_buf[idx + 1 - input->w];
    d_buf[idx] = sqrti(temp1*temp1 + temp2*temp2);
  }

  // set x gradient for first and last row
  for (idx = 1, idx1 = size - d->w + 1; idx1 < size - 1; idx++, idx1++) {
    d_buf[idx] = (uint8_t)abs((int16_t)input_buf[idx + 1] - (int16_t)input_buf[idx - 1]);
    d_buf[idx1] = (uint8_t)abs((int16_t)input_buf[idx1 + 1] - (int16_t)input_buf[idx1 - 1]);
  }

  // set y gradient for first and last col
  for (idx = d->w, idx1 = 2*d->w-1; idx1 < size - input->w; idx+=input->w, idx1+=input->w) {
    d_buf[idx] = (uint8_t)abs((int16_t)input_buf[idx + input->w] - (int16_t)input_buf[idx - input->w]);
    d_buf[idx1] = (uint8_t)abs((int16_t)input_buf[idx1 + input->w] - (int16_t)input_buf[idx1 - input->w]);
  }
}

