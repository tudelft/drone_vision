#include "skysegmentation.h"
#include <stdlib.h>     /* abs */
#include <stdio.h> /* printf */
#include "trig.h"

/******************Defines********************/
#define uint8_t unsigned char
#define N_BINS 10
#define image_index(xx, yy)  ((yy * imgWidth + xx) * 2) & 0xFFFFFFFC  // always a multiple of 4

/******************Private global variables********************/
const int MAX_ROLL_ANGLE = 60;
const int MAX_PITCH_ANGLE = 40;
const int MAX_I2C_BYTE = 254;

/******************Private prototypes********************/
// Feature extraction:
extern void getGradientPixel(unsigned char *frame_buf, int x, int y, int* dx, int* dy);
extern int getGradient(unsigned char *frame_buf, int x, int y);
extern void getGradientImage(unsigned char *frame_buf, unsigned char *frame_buf2, unsigned char *frame_buf3);
extern unsigned int getMaximumY(unsigned char *frame_buf);
extern unsigned int getMinimumY(unsigned char *frame_buf);
extern int getHarrisPixel(unsigned char *frame_buf, int x, int y);
extern int getNoblePixel(unsigned char *frame_buf, int x, int y);
extern int getPatchTexture(unsigned char *frame_buf, int x, int y, int patch_size);
extern int getPatchMean(unsigned char *frame_buf, int x, int y, int patch_size);
extern int get_FD_YCV(unsigned char *frame_buf, int x, int y);
extern int get_FD_CV(unsigned char *frame_buf, int x, int y);

extern void segmentSkyUncertainty2(unsigned char *frame_buf, unsigned char *frame_buf2);
extern void segment_no_yco(unsigned char *frame_buf, unsigned char *frame_buf2);


void getObstacles(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL);
void getUncertainty(unsigned int* uncertainty, unsigned int n_bins, unsigned char *frame_buf);
void getObstacles2Way(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL, int pitch_pixels, int roll_angle);
void horizonToLineParameters(int pitch_pixel, int roll_angle, int* a, int* b);

void drawLine(unsigned char *frame_buf, int a, int b, int resolution);

static inline uint8_t scale_to_range(int x, int min, int max, int range);
static inline int pitch_angle_to_pitch_pixel(int pitch);





/**************************Code********************/

// *****************
// INLINE FUNCTIONS:
// *****************

static inline void groundPixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0x00;
  frame_buf[ip+1] = 0x00;
  frame_buf[ip+2] = 0x00;
  frame_buf[ip+3] = 0x00;
}

static inline void redPixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0x00;
  frame_buf[ip+1] = 0x00;
  frame_buf[ip+2] = 0xff;
  frame_buf[ip+3] = 0x00;
}

static inline void blackDot(unsigned char *frame_buf, int x, int y)
{
  unsigned int ip;
  int xx, yy;
  for(xx = x-1; xx <= x+1; xx++)
  {
    for(yy = y-1; yy <= y+1; yy++)
    {
      if(xx >= 0 && xx < (int)imgWidth && yy >= 0 && yy < (int)imgHeight)
      {
        ip = image_index(xx,yy);
        frame_buf[ip] = 0x7f;
        frame_buf[ip+1] = 0x00;
        frame_buf[ip+2] = 0x7f;
        frame_buf[ip+3] = 0x00;
      }
    }
  }


}

static inline void setUncertainty(unsigned char *frame_buf, unsigned int ip, unsigned int uncertainty)
{
  // if(uncertainty > 255) uncertainty = 255;
  frame_buf[ip] = 127;
  frame_buf[ip+1] = uncertainty;
  frame_buf[ip+2] = 127;
  frame_buf[ip+3] = uncertainty;
}

static inline int isGroundPixel(unsigned char *frame_buf, unsigned int ip)
{
  if(frame_buf[ip] == 0x00 && frame_buf[ip+1] == 0x00 && frame_buf[ip+2] == 0x00 && 	frame_buf[ip+3] == 0x00) return 1;
  else return 0;
}

static inline void linePixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0;
  frame_buf[ip+1] = 255;
  frame_buf[ip+2] = 255;
  frame_buf[ip+3] = 255;
}


// ******************
// FEATURE EXTRACTION
// ******************

extern int getPatchTexture(unsigned char *frame_buf, int x, int y, int patch_size)
{
  unsigned int value, ix;
  int half_patch_size, dx, dy, center_pixel, texture, indx, indy;
  half_patch_size = patch_size / 2;
  texture = 0;
  // correct coordinates of center pixel if necessary:
  x = (x < half_patch_size) ? half_patch_size : x;
  x = (x >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : x;
  y = (y < half_patch_size) ? half_patch_size : y;
  y = (y >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : y;

  ix = image_index(x,y);
  center_pixel = (int)((((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1);
  for(dx = -half_patch_size; dx <= half_patch_size; dx++)
  {
    for(dy = -half_patch_size; dy <= half_patch_size; dy++)
    {
      if(!(dx == 0 && dy == 0))
      {
        indx = x + dx;
        indy = y + dy;
        ix = image_index(indx, indy);
        value = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
        texture += abs((int) value - (int) center_pixel);
      }
    }
  }
  texture /= (patch_size * patch_size - 1);
  return texture;
}

extern int get_FD_YCV(unsigned char *frame_buf, int x, int y)
{
  unsigned int Y, Cb, Cr, ix;
  int FD_YCV;

  ix = image_index(x, y);
  Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
  Cb = (unsigned int)frame_buf[ix];
  Cr = (unsigned int)frame_buf[ix+2];

  // In the blackfin, values range from [0, 255] while the formula is based on
  // values in the range [0,1]. Therefore we divide by 255 after the color channels.
  // This leaves the factor 100 with which all coefficients were multiplied.
  FD_YCV = (860 * (int)Y - 501 * (int) Cr + 2550 * (int) Cb) / 255 - 1545;
  return FD_YCV;
}

extern int get_FD_CV(unsigned char *frame_buf, int x, int y)
{
  unsigned int Cb, Cr, ix;
  int FD_CV;

  ix = image_index(x, y);
  Cb = (unsigned int)frame_buf[ix];
  Cr = (unsigned int)frame_buf[ix+2];

  // In the blackfin, values range from [0, 255] while the formula is based on
  // values in the range [0,1]. Therefore we divide by 255 after the channels.
  // This leaves the factor 100 with which all coefficients were multiplied.
  FD_CV = (1975 * (int) Cb - 446 * (int) Cr) / 255 - 818;
  return FD_CV;
}



extern int getPatchMean(unsigned char *frame_buf, int x, int y, int patch_size)
{
  unsigned int value, ix;
  int half_patch_size, dx, dy, mean, indx, indy;
  half_patch_size = patch_size / 2;
  mean = 0;
  // correct coordinates of center pixel if necessary:
  x = (x < half_patch_size) ? half_patch_size : x;
  x = (x >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : x;
  y = (y < half_patch_size) ? half_patch_size : y;
  y = (y >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : y;

  for(dx = -half_patch_size; dx <= half_patch_size; dx++)
  {
    for(dy = -half_patch_size; dy <= half_patch_size; dy++)
    {
      indx = x + dx;
      indy = y + dy;
      ix = image_index(indx, indy);
      value = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
      mean += (int) value;
    }
  }
  mean /= (patch_size * patch_size);
  return mean;
}

extern int getHarrisPixel(unsigned char *frame_buf, int x, int y)
{
  int dx, dy, dx2, dxy, dy2, Harris, min_x, min_y, xx, yy, it;
  int smooth[9];
  smooth[0] = 1;
  smooth[0] = 2;
  smooth[0] = 1;
  smooth[0] = 2;
  smooth[0] = 4;
  smooth[0] = 2;
  smooth[0] = 1;
  smooth[0] = 2;
  smooth[0] = 1;// in MATLAB-language: [1,2,1;2,4,2;1,2,1]

  int smooth_factor = 1400; // this factor was chosen to keep Harris within bounds

  // determine the 3 x 3 patch around x,y:
  if(x <= 0) min_x = 0;
  else if(x >= (int)imgWidth - 1) min_x = (int)imgWidth - 2;
  else min_x = x - 1;
  if(y <= 0) min_y = 0;
  else if(y >= (int)imgHeight - 1) min_y = (int)imgHeight - 2;
  else min_y = y - 1;

  // use the patch to determine dx2, dxy, and dy2
  it = 0;
  dx2 = 0;
  dxy = 0;
  dy2 = 0;
  for(yy = min_y; yy < min_y + 3; yy++)
  {
    for(xx = min_x; xx < min_x + 3; xx++)
    {
      getGradientPixel(frame_buf, xx, yy, &dx, &dy);
      // approximation of smoothed second order derivatives:
      dx2 += smooth[it] * dx * dx;
      dxy += smooth[it] * dx * dy;
      dy2 += smooth[it] * dy * dy;
      // update iterator of the smoothing filter:
      it++;
    }
  }

  // correcting the approximations to keep Harris numbers within bounds:
  dx2 /= smooth_factor;
  dxy /= smooth_factor;
  dy2 /= smooth_factor;

  // Harris = (dx2 * dy2 - dxy*dxy) - k * (dx2 + dy2) * (dx2 + dy2);
  // where k = 0.04 in floating point
  Harris = (dx2 * dy2 - dxy*dxy) - ((dx2 + dy2) * (dx2 + dy2)) / 25;
  return Harris;
}

extern int getNoblePixel(unsigned char *frame_buf, int x, int y)
{
  int dx, dy, dx2, dxy, dy2, Noble, min_x, min_y, xx, yy, it;
  int smooth[9];
  smooth[0] = 1;
  smooth[0] = 2;
  smooth[0] = 1;
  smooth[0] = 2;
  smooth[0] = 4;
  smooth[0] = 2;
  smooth[0] = 1;
  smooth[0] = 2;
  smooth[0] = 1;// in MATLAB-language: [1,2,1;2,4,2;1,2,1]
  int smooth_factor = 1400; // this factor was chosen to keep Harris within bounds

  // determine the 3 x 3 patch around x,y:
  if(x <= 0) min_x = 0;
  else if(x >= (int)imgWidth - 1) min_x = (int)imgWidth - 2;
  else min_x = x - 1;
  if(y <= 0) min_y = 0;
  else if(y >= (int)imgHeight - 1) min_y = (int)imgHeight - 2;
  else min_y = y - 1;

  // use the patch to determine dx2, dxy, and dy2
  it = 0;
  dx2 = 0;
  dxy = 0;
  dy2 = 0;
  for(yy = min_y; yy < min_y + 3; yy++)
  {
    for(xx = min_x; xx < min_x + 3; xx++)
    {
      getGradientPixel(frame_buf, xx, yy, &dx, &dy);
      // approximation of smoothed second order derivatives:
      dx2 += smooth[it] * dx * dx;
      dxy += smooth[it] * dx * dy;
      dy2 += smooth[it] * dy * dy;
      // update iterator of the smoothing filter:
      it++;
    }
  }

  // correcting the approximations to keep Harris numbers within bounds:
  dx2 /= smooth_factor;
  dxy /= smooth_factor;
  dy2 /= smooth_factor;

  // Noble = (dx2*dy2 - dxy*dxy) / (dx2 + dy2 + epsilon), where epsilon is a small number
  // Noble values are quite small, so we might have to multiply with a fixed number somewhere below
  if(dx2 + dy2 > 0)
  {
    Noble = (dx2 * dy2 - dxy * dxy) / (dx2 + dy2);
  }
  else
  {
    Noble = dx2 * dy2 - dxy * dxy;
    if(Noble > 0)
    {
      if(Noble <= 65) Noble *= 1000;
      else Noble = 65335;
    }
    else
    {
      Noble = 0;
    }
  }

  return Noble;
}




// This function gives the maximum of a subsampled version of the image
unsigned int getMaximumY(unsigned char *frame_buf)
{
  unsigned int ix, y, max_y;
  unsigned int color_channels = 4;
  unsigned int step = 5 * color_channels;
  max_y = 0;
  for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
  {
    // we can speed things up by just looking at the first channel:
    y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    if(y > max_y) max_y = y;
  }
  return max_y;
}

extern unsigned int getMinimumY(unsigned char *frame_buf)
{
  unsigned int ix, y, min_y;
  unsigned int color_channels = 4;
  unsigned int step = 5 * color_channels;
  min_y = 255;
  for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
  {
    // we can speed things up by just looking at the first channel:
    y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    if(y < min_y) min_y = y;
  }
  return min_y;

}


// For computational efficiency, we currently do not use any mask for determining the gradient.
extern void getGradientImage(unsigned char *frame_buf, unsigned char *frame_buf2, unsigned char *frame_buf3)
{
  unsigned int x,y,ix;
  int dx,dy;
  for(x = 0; x < imgWidth; x++)
  {
    for(y = 0; y < imgHeight; y++)
    {
      ix = image_index(x,y);
      getGradientPixel(frame_buf, x, y, &dx, &dy);

      // gradient has to be stored in unsigned format. We prefer to cut the values off at -127 / +127 than to reduce the resolution
      dx = dx + 127;
      dx = (dx < 0) ? 0 : dx;
      dx = (dx > 255) ? 255 : dx;
      dy = dy + 127;
      dy = (dy < 0) ? 0 : dy;
      dy = (dy > 255) ? 255 : dy;
      frame_buf2[ix+1] = (unsigned int) dx;
      frame_buf2[ix+3] = (unsigned int) dx;
      frame_buf3[ix+1] = (unsigned int) dy;
      frame_buf3[ix+3] = (unsigned int) dy;
    }
  }
}

extern void getGradientPixel(unsigned char *frame_buf, int x, int y, int* dx, int* dy)
{
  unsigned int ix, Y1, Y2;
  unsigned int xx, yy;
  // currently we use [0 0 0; -1 0 1; 0 0 0] as mask for dx
  if(x >= 0 && x < (int)imgWidth && y >= 0 && y < (int)imgHeight)
  {
    if(x > 0)
    {
      xx = x - 1;
      yy = y;
      ix = image_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = 0; yy = y;
      ix = image_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    if(x < (int)imgWidth - 1)
    {
      xx = x+1; yy = y;
      ix = image_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = imgWidth - 1; yy = y;
      ix = image_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    (*dx) = ((int)Y2) - ((int)Y1);

    if(y > 0)
    {
      xx = x; yy = y - 1;
      ix = image_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = x; yy = 0;
      ix = image_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    if(y < (int)imgHeight - 1)
    {
      xx = x; yy = y + 1;
      ix = image_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = x; yy = imgHeight-1;
      ix = image_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    (*dy) = ((int)Y2) - ((int)Y1);
  }
}

extern int getGradient(unsigned char *frame_buf, int x, int y)
{
  int dx, dy, gra;

  if(x >= 0 && x < (int)imgWidth && y >= 0 && y < (int)imgHeight)
  {
    getGradientPixel(frame_buf, x, y, &dx, &dy);
    dx = abs(dx);
    dy = abs(dy);
    gra = dx + dy;
    return gra;
  }
  else
  {
    // coordinate not within image
    return 0;
  }


}



void segmentSkyUncertainty2(unsigned char *frame_buf, unsigned char *frame_buf2)
{
  // use a pre-defined tree to segment the image:
  // the second buffer (image) stores the uncertainties between 0 and 100.
  int x, y, threshold, value;
  unsigned int ix, Y, U, V, maxY;

  // the maximal illuminance is used in almost all sub-branches, so it is better to calculate it immediately once:
  maxY = getMaximumY(frame_buf);

  for(x = 0; x < (int)imgWidth; x++)
  {
    for(y = 0; y < (int)imgHeight; y++) // we could divide imgHeight by 2 to speed things up
    {
      ix = image_index(x,y);

      threshold = (imgHeight * 41) / 100;
      if(y <= threshold) // high in the image
      {
        value = getGradient(frame_buf, x, y);
        if(value <= 4) // little gradient
        {
          U = (unsigned int)frame_buf[ix];

          if(U <= 137)
          {
            Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            if(Y <= (maxY * 30) / 100) // not very bright: was 59, now 30
            {
              V = (unsigned int)frame_buf[ix + 2];

              if(V <= 143) // check this one: are the colors right?
              {
                threshold = (imgHeight * 29) / 100;
                if(y <= threshold)
                {
                  if(value <= 3) // was 1
                  {
                    // sky
                    // 70.0%
                    setUncertainty(frame_buf2, ix, 30);
                  }
                  else
                  {
                    // ground
                    // 77.6%
                    groundPixel(frame_buf, ix);
                    setUncertainty(frame_buf2, ix, 22);
                  }
                }
                else
                {
                  // ground:
                  // 87.76%
                  groundPixel(frame_buf, ix);
                  setUncertainty(frame_buf2, ix, 12);
                }
              }
              else
              {
                // sky:
                // 76,53%
                setUncertainty(frame_buf2, ix, 23);
              }
            }
            else
            {
              // sky
              // 87.37% certainty
              setUncertainty(frame_buf2, ix, 12);
            }
          }
          else
          {
            // sky
            // 93,45% certainty
            setUncertainty(frame_buf2, ix, 6);
          }
        }
        else
        {
          // more gradient
          Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
          if(Y <= (maxY * 30) / 100) // not very bright: was 59, now 30
          {
            U = (unsigned int)frame_buf[ix];
            if(U <= 141)
            {
              // ground:
              // 92.0%
              groundPixel(frame_buf, ix);
              setUncertainty(frame_buf2, ix, 8);
            }
            else
            {
              if(U <= 152)
              {
                threshold = (imgHeight * 21) / 100;
                if(y <= threshold)
                {
                  // sky:
                  // 67.7%
                  setUncertainty(frame_buf2, ix, 32);
                }
                else
                {
                  // ground:
                  // 80.0%
                  groundPixel(frame_buf, ix);
                  setUncertainty(frame_buf2, ix, 20);
                }
              }
              else
              {
                // sky
                // 74.5%
                setUncertainty(frame_buf2, ix, 25);
              }
            }
          }
          else
          {
            U = (unsigned int)frame_buf[ix];
            if(U <= 135)
            {
              if(value <= 28) // medium gradient:
              {
                if(Y <= (maxY * 75) / 100)
                {
                  // ground:
                  // 71.5%
                  groundPixel(frame_buf, ix);
                  setUncertainty(frame_buf2, ix, 28);
                }
                else
                {
                  //sky:
                  // 74.1% certainty
                  setUncertainty(frame_buf2, ix, 26);
                }
              }
              else
              {
                // high gradient:
                // ground
                // 78.6%
                groundPixel(frame_buf, ix);
                setUncertainty(frame_buf2, ix, 21);
              }
            }
            else
            {
              // sky
              // 79.5%
              setUncertainty(frame_buf2, ix, 20);
            }
          }

        }
      }
      else
      {
        threshold = (imgHeight * 57) / 100;
        if(y <= threshold)
        {
          Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
          if(Y <= (maxY * 58) / 100)
          {
            // ground
            // 94.5%
            groundPixel(frame_buf, ix);
            setUncertainty(frame_buf2, ix, 5);
          }
          else
          {
            value = getGradient(frame_buf, x, y);
            if(value <= 16)
            {
              V = (unsigned int)frame_buf[ix + 2];
              if(V <= 118)
              {
                // sky
                // 85.1%
                setUncertainty(frame_buf2, ix, 15);
              }
              else
              {
                threshold = (imgHeight * 47) / 100;
                if(y <= threshold)
                {
                  // sky:
                  // 70.3%
                  setUncertainty(frame_buf2, ix, 29);
                }
                else
                {
                  // ground:
                  // 71.9%
                  groundPixel(frame_buf, ix);
                  setUncertainty(frame_buf2, ix, 28);
                }
              }
            }
            else
            {
              // ground
              // 84.2%
              groundPixel(frame_buf, ix);
              setUncertainty(frame_buf2, ix, 16);
            }
          }
        }
        else
        {
          // ground: 98,74%
          groundPixel(frame_buf, ix);
          setUncertainty(frame_buf2, ix, 1);
        }
      }
    }
  }
}

extern void segment_no_yco(unsigned char *frame_buf, unsigned char *frame_buf2)
{
  // use a pre-defined tree to segment the image:
  // the second buffer (image) stores the uncertainties between 0 and 100.
  int x, y, maxY, patch_size, patch_texture;
  int FD_YCV, FD_CV;
  unsigned int ix, Y, Cr;

  // global variables, so that they are calculated / initialized only once:
  maxY = getMaximumY(frame_buf);
  patch_size = 10;

  for(x = 0; x < (int)imgWidth; x++)
  {
    for(y = 0; y < (int)imgHeight; y++)
    {
      ix =image_index(x,y);

      FD_YCV = get_FD_YCV(frame_buf, x, y);
      if(FD_YCV <= 10)
      {
        Cr = (unsigned int)frame_buf[ix+2];
        if(Cr <= 153)
        {
          FD_YCV = get_FD_YCV(frame_buf, x, y);
          if(FD_YCV <= -125)
          {
            // ground: 98%
            setUncertainty(frame_buf2, ix, 2);
            groundPixel(frame_buf, ix);
          }
          else
          {
            patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
            if(patch_texture <= 4)
            {
              Cr = (unsigned int)frame_buf[ix+2];
              if(Cr <= 129)
              {
                // sky: 74%
                setUncertainty(frame_buf2, ix, 26);
              }
              else
              {
                // ground: 75%
                setUncertainty(frame_buf2, ix, 25);
                groundPixel(frame_buf, ix);
              }
            }
            else
            {
              // ground: 90%
              setUncertainty(frame_buf2, ix, 10);
              groundPixel(frame_buf, ix);
            }
          }
        }
        else
        {
          patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
          if(patch_texture <= 7)
          {
            // sky: 69%
            setUncertainty(frame_buf2, ix, 31);
          }
          else
          {
            // ground: 88%
            setUncertainty(frame_buf2, ix, 12);
            groundPixel(frame_buf, ix);
          }
        }
      }
      else
      {
        patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
        if(patch_texture <= 7)
        {
          FD_CV = get_FD_CV(frame_buf, x, y);
          if(FD_CV <= -83)
          {
            // ground: 67%
            setUncertainty(frame_buf2, ix, 33);
            groundPixel(frame_buf, ix);
          }
          else
          {
            // sky: 90%
            setUncertainty(frame_buf2, ix, 10);
          }
        }
        else
        {
          FD_YCV = get_FD_YCV(frame_buf, x, y);
          if(FD_YCV <= 164)
          {
            patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
            if(patch_texture <= 13)
            {
              FD_CV = get_FD_CV(frame_buf, x, y);
              if(FD_CV <= -52)
              {
                // ground: 79%
                setUncertainty(frame_buf2, ix, 21);
                groundPixel(frame_buf, ix);
              }
              else
              {
                Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
                if(Y <= (unsigned int)(maxY * 60) / 100)
                {
                  // ground: 76%
                  setUncertainty(frame_buf2, ix, 24);
                  groundPixel(frame_buf, ix);
                }
                else
                {
                  // sky: 71%
                  setUncertainty(frame_buf2, ix, 29);
                }
              }
            }
            else
            {
              // ground: 84%
              setUncertainty(frame_buf2, ix, 16);
              groundPixel(frame_buf, ix);
            }
          }
          else
          {
            // sky: 76%
            setUncertainty(frame_buf2, ix, 24);
          }
        }
      }
    }
  }
}

extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor)
{
  // use a pre-defined tree to segment the image:
  // the second buffer (image) stores the uncertainties between 0 and 100.
  int x, y, maxY, adjust_rel_Y, patch_size, patch_texture, adjust_patch_texture, adjust_Cr, FD_YCV, adjust_FD_YCV, FD_CV, adjust_FD_CV;
  unsigned int ix, Y, Cr;

  // global variables, so that they are calculated / initialized only once:
  maxY = getMaximumY(frame_buf);
  patch_size = 10;

  // variables for adjusting thresholds:
  adjust_Cr = adjust_factor * 3;
  adjust_rel_Y = adjust_factor * -5;
  adjust_patch_texture = adjust_factor * 2;
  adjust_FD_YCV = adjust_factor * -48;
  adjust_FD_CV = adjust_factor * -48;


  for(x = 0; x < (int)imgWidth; x++)
  {
    for(y = 0; y < (int)imgHeight; y++)
    {
      ix = image_index(x,y);

      FD_YCV = get_FD_YCV(frame_buf, x, y);
      if(FD_YCV <= 58 + adjust_FD_YCV)
      {
        Cr = (unsigned int)frame_buf[ix+2];
        if(Cr <= 150 + (unsigned int)adjust_Cr)
        {
          FD_YCV = get_FD_YCV(frame_buf, x, y);
          if(FD_YCV <= -77 + adjust_FD_YCV)
          {
            // ground: 98%
            setUncertainty(frame_buf2, ix, 2);
            groundPixel(frame_buf, ix);
          }
          else
          {
            patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
            if(patch_texture <= 2 + adjust_patch_texture)
            {
              Cr = (unsigned int)frame_buf[ix+2];
              if(Cr <= 126 + (unsigned int)adjust_Cr)
              {
                // sky: 74%
                setUncertainty(frame_buf2, ix, 26);
              }
              else
              {
                // ground: 75%
                setUncertainty(frame_buf2, ix, 25);
                groundPixel(frame_buf, ix);
              }
            }
            else
            {
              // ground: 90%
              setUncertainty(frame_buf2, ix, 10);
              groundPixel(frame_buf, ix);
            }
          }
        }
        else
        {
          patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
          if(patch_texture <= 4 + adjust_patch_texture)
          {
            // sky: 69%
            setUncertainty(frame_buf2, ix, 31);
          }
          else
          {
            // ground: 88%
            setUncertainty(frame_buf2, ix, 12);
            groundPixel(frame_buf, ix);
          }
        }
      }
      else
      {
        patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
        if(patch_texture <= 5 + adjust_patch_texture)
        {
          FD_CV = get_FD_CV(frame_buf, x, y);
          if(FD_CV <= -51 + adjust_FD_CV)
          {
            // ground: 67%
            setUncertainty(frame_buf2, ix, 33);
            groundPixel(frame_buf, ix);
          }
          else
          {
            // sky: 90%
            setUncertainty(frame_buf2, ix, 10);
          }
        }
        else
        {
          FD_YCV = get_FD_YCV(frame_buf, x, y);
          if(FD_YCV <= 212 + adjust_FD_YCV)
          {
            patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
            if(patch_texture <= 11 + adjust_patch_texture)
            {
              FD_CV = get_FD_CV(frame_buf, x, y);
              if(FD_CV <= -19 + adjust_FD_CV)
              {
                // ground: 79%
                setUncertainty(frame_buf2, ix, 21);
                groundPixel(frame_buf, ix);
              }
              else
              {
                Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
                if(Y <= (unsigned int)(maxY * 66) / 100 + adjust_rel_Y)
                {
                  // ground: 76%
                  setUncertainty(frame_buf2, ix, 24);
                  groundPixel(frame_buf, ix);
                }
                else
                {
                  // sky: 71%
                  setUncertainty(frame_buf2, ix, 29);
                }
              }
            }
            else
            {
              // ground: 84%
              setUncertainty(frame_buf2, ix, 16);
              groundPixel(frame_buf, ix);
            }
          }
          else
          {
            // sky: 76%
            setUncertainty(frame_buf2, ix, 24);
          }
        }
      }
    }
  }
}

void getObstacles(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL)
{
  unsigned int x,y,ix,GRND, bin_size, bin, HALF_HEIGHT, bin_surface;
  // reset obstacles values
  for(bin = 0; bin < n_bins; bin++)
  {
    obstacles[bin] = 0;
  }
  bin_size = imgWidth / n_bins;
  HALF_HEIGHT = imgHeight / 2;
  bin_surface = bin_size * HALF_HEIGHT;
  GRND = 0x00;
  // frame_buf contains the segmented image. All non-zero elements are sky.
  // This function assumes no pitch and roll.
  for(x = 0; x < imgWidth; x++)
  {
    for(y = 0; y < HALF_HEIGHT; y++)
    {
      ix = image_index(x,y);
      if(frame_buf[ix] == GRND) // Of course, the original image could also use black pixels - of which probably few in the sky
      {
        bin = x / bin_size;
        if(bin >= n_bins)
        {
          bin = n_bins-1;
        }
        obstacles[bin]++;
      }
    }
  }
  // obstacles[bin] should have a maximum corresponding to MAX_SIGNAL
  (*max_bin) = 0;
  (*obstacle_total) = 0;

  for(bin = 0; bin < n_bins; bin++)
  {
    obstacles[bin] *= MAX_SIGNAL;
    obstacles[bin] /= bin_surface;
    if(obstacles[bin] > (*max_bin))
    {
      (*max_bin) = obstacles[bin];
    }
    (*obstacle_total) += obstacles[bin];
  }
  (*obstacle_total) /= n_bins;
}

void horizonToLineParameters(int pitch_pixel, int roll_angle, int* a, int* b)
{
  (*b) = 1000 * (pitch_pixel + imgHeight / 2);
  (*a) = -tan_zelf(roll_angle);
}

void getObstacles2Way(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL, int pitch_pixels, int roll_angle)
{
  // procedure:
  // 1) determine the horizon line on the basis of pitch and roll
  // 2) determine the central point projected on the horizon line
  // 3) determine the step_x in order to retain the same bin_size along the horizon line
  // 4) run over the image from left to right in lines parallel to the horizon line
  unsigned int ix,bin;
  int a, b, halfWidth, halfHeight;
  int i, x, y;
  int x1, y1, x2, y2, RESOLUTION;
  int a2, b2, x12, y12, bin_size, step_x;
  int xx, yy, x_start;
  int bin_surface;
  halfWidth = imgWidth / 2;
  halfHeight = imgHeight / 2;
  bin_size = imgWidth / n_bins;

  // initialize bins:
  for(bin = 0; bin < n_bins; bin++)
  {
    obstacles[bin] = 0;
  }

  // 1) determine the horizon line on the basis of pitch and roll
  RESOLUTION = 1000;
  horizonToLineParameters(pitch_pixels, roll_angle, &a, &b);
  //printf("Roll: %d, Slope: %d\n",roll_angle,a);

  // 2) determine the central point projected on the horizon line:
  x1 = -halfWidth;
  y1 = (b + a * x1) / RESOLUTION;
  x2 = halfWidth;
  y2 = (b + a * x2) / RESOLUTION;


  if(a == 0) a = 1;
  a2 = (RESOLUTION / a) * RESOLUTION; // slope orthogonal to a
  b2 = halfHeight * RESOLUTION; // goes through the center
  x12 = (100*(b2 - b)) / (a + a2); // RESOLUTION factor disappears
  x12 /= 100;
  y12 = (a * x12 + b) / RESOLUTION;

  // only further process the image if the horizon line is entirely visible
  if(y1 >= 0 && y1 < (int)imgHeight && y2 >= 0 && y2 < (int)imgHeight)
  {
    // 3) determine the step_x in order to retain the same bin_size along the horizon line
    step_x = (int)isqrt(
        (unsigned int) (bin_size * bin_size) / (((a*a) / (RESOLUTION*RESOLUTION)) + 1)
    );

    // 4) run over the image from left to right in lines parallel to the horizon line
    x_start = x12 - (n_bins / 2) * step_x;

    for(i = 0; i > -halfHeight; i--)
    {
      for(x = x_start; x < halfWidth; x++)
      {
        // determine the appropriate bin:
        bin = (x - x_start) / step_x;
        if(bin >= n_bins) bin = n_bins - 1;
        y = (a * x + b) / RESOLUTION + i;
        // transform to image coordinates:
        xx = x + halfWidth;
        yy = y;

        if(xx >= 0 && xx < (int)imgWidth && yy >= 0 && yy < (int)imgHeight)
        {
          ix = image_index(xx,yy);
          if(isGroundPixel(frame_buf, ix))
          {
            // make pixel red
            redPixel(frame_buf, ix);
            // add pixel to obstacle bin:
            obstacles[bin]++;
          }
        }
      }
    }

    // get the variables of interest and transform them to output form
    bin_surface = bin_size * halfHeight;
    // obstacles[bin] should have a maximum corresponding to MAX_SIGNAL
    (*max_bin) = 0;
    (*obstacle_total) = 0;

    for(bin = 0; bin < n_bins; bin++)
    {
      obstacles[bin] *= MAX_SIGNAL;
      obstacles[bin] /= bin_surface;
      if(obstacles[bin] > (*max_bin))
      {
        (*max_bin) = obstacles[bin];
      }
      (*obstacle_total) += obstacles[bin];
    }
    (*obstacle_total) /= n_bins;

  }
  else
  {
    (*max_bin) = 0;
    (*obstacle_total) = 0;
  }
  // 1000 is the resolution of the tan-function
  drawLine((unsigned char *)frame_buf, a, y1*RESOLUTION, RESOLUTION);
  blackDot((unsigned char *)frame_buf, x12+halfWidth, y12);
}

void drawLine(unsigned char *frame_buf, int a, int b, int resolution)
{
  int x, y, b_res;
  unsigned int ix;

  if(resolution == 0) resolution = 1;
  b_res = b / resolution;

  for(x = 0; x < (int)imgWidth; x++)
  {
    y = (a * x) / resolution + b_res;
    if(y >= 0 && y < (int)imgHeight)
    {
      ix = image_index(x,y);
      linePixel(frame_buf, ix);
    }
  }
}


void getUncertainty(unsigned int* uncertainty, unsigned int n_bins, unsigned char *frame_buf)
{
  unsigned int x,y, ix, bin_size, bin, HALF_HEIGHT, uncertainty_line, last_bin;
  // reset uncertainty values
  for(bin = 0; bin < n_bins; bin++)
  {
    uncertainty[bin] = 0;
  }
  bin_size = imgWidth / n_bins;
  HALF_HEIGHT = imgHeight / 2;
  // frame_buf contains the uncertainties in the range [0,50] with 50 maximally uncertain.
  // (if higher, the other class should be chosen).
  // This function assumes no pitch and roll.
  last_bin = 0;

  for(x = 0; x < imgWidth; x++)
  {
    bin = x / bin_size;
    if(bin >= n_bins) bin = n_bins - 1;
    if(bin > last_bin)
    {
      // the final value in uncertainty[bin] is an average per pixel in the bin
      uncertainty[bin-1] /= bin_size;
      last_bin = bin;
    }
    // sum the uncertainty value over one vertical line:
    uncertainty_line = 0;
    for(y = 0; y < HALF_HEIGHT; y++)
    {
      ix = image_index(x,y);
      uncertainty_line += frame_buf[ix+1]; // Y-channel contains the uncertainty
    }
    uncertainty[bin] += uncertainty_line / HALF_HEIGHT; // per vertical line, the average uncertainty value per pixel is added to uncertainty[bin]
  }
  // the final value in uncertainty[bin] is an average per pixel in the bin
  uncertainty[n_bins-1] /= bin_size;
}

static inline int pitch_angle_to_pitch_pixel(int pitch)
{
  int pitch_pixel = scale_to_range(pitch, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE, imgHeight);
  pitch_pixel -= imgHeight / 2;
  return pitch_pixel;
}
static inline uint8_t scale_to_range(int x, int min, int max, int range)
{
  if (x < min)
    x = min;
  else if (x > max)
    x = max;

  x -= min;
  x *= range;
  x /= (max - min);
  return (uint8_t) x;
}

void get_obstacle_bins_above_horizon(unsigned char *frame_buf, unsigned char *frame_buf2, char adjust_factor, unsigned int n_bins, unsigned int* obstacle_bins, unsigned int* uncertainty_bins, int pitch, int roll)
{
  int MAX_BIN_VALUE = MAX_I2C_BYTE;
  unsigned int max_bin, bin_total;

  if(adjust_factor < 3)
  {
    if(adjust_factor == 0) adjust_factor = -10;
    if(adjust_factor == 1) adjust_factor = -5;
    if(adjust_factor == 2) adjust_factor = -2;
  }
  else if(adjust_factor == 3)
  {
    adjust_factor = 0;
  }
  else
  {
    if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
    if(adjust_factor == 7) adjust_factor = 5;
    if(adjust_factor == 8) adjust_factor = 8;
    if(adjust_factor == 9) adjust_factor = 11;
    if(adjust_factor == 10) adjust_factor = 15;
  }


  // Segment Pixels into ground and sky
  segment_no_yco_AdjustTree(frame_buf, frame_buf2, adjust_factor);

  // Make obstacle bins
  getObstacles2Way(obstacle_bins, n_bins, (unsigned char *)frame_buf, &max_bin, &bin_total, MAX_BIN_VALUE, pitch_angle_to_pitch_pixel(pitch), roll);

  // Get uncertainty per obstacle
  getUncertainty(uncertainty_bins, n_bins, (unsigned char *)frame_buf2);

}

