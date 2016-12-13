#ifndef SKYSEGMENTATION
#define SKYSEGMENTATION

extern unsigned int imgWidth, imgHeight;


/***
 *    \brief:   Sky Segmentation: find ground/sky pixels
 *              no_yco: No y-coordinate in decission tree
 *
 *              frame_buf = input image -> output of the classification
 *              frame_buf2 = output of the certainty of the classification
 *              adjust_factor = from 0 to 10, 3 = no adjust (adjust: find more or less ground)
 */

extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor);

/***
 *    \brief:   Sky Segmentation: find ground/sky pixels
 *              no_yco: No y-coordinate in decission tree
 *
 *              frame_buf = input image -> output of the classification
 *              frame_buf2 = output of the certainty of the classification
 *              adjust_factor = from 0 to 10, 3 = no adjust (adjust: find more or less ground)
 *              n_bins = number of bins -> obstacle_bins and uncertainty_bins are pointers to arrays
 *              pitch and roll are used to overlay the horizon line
 */

void get_obstacle_bins_above_horizon(unsigned char *frame_buf, unsigned char *frame_buf2, char adjust_factor, unsigned int n_bins, unsigned int* obstacle_bins, unsigned int* uncertainty_bins, int pitch, int roll);

#endif /* SKYSEGMENTATION */
