/*
 * filter.h
 *
 *  Created on: 24 Oct 2017
 *      Author: kirk
 */

#ifndef CV_FILTERS_FILTER_H_
#define CV_FILTERS_FILTER_H_

#include "image.h"

/* YUV Color Filter Parameters */
struct image_filter_t {
  uint8_t y_min;            ///< YUV color filter
  uint8_t y_max;
  uint8_t u_min;
  uint8_t u_max;
  uint8_t v_min;
  uint8_t v_max;
};

extern uint16_t image_yuv422_colorfilt(struct image_t *input, struct image_t *output, struct image_filter_t filter);
extern void image_gradients(struct image_t *input, struct image_t *dx, struct image_t *dy);
extern uint8_t sqrti(int32_t num);
extern void image_2d_gradients(struct image_t *input, struct image_t *d);
extern void image_2d_sobel(struct image_t *input, struct image_t *d);

#endif /* CV_FILTERS_FILTER_H_ */
