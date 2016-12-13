#include <stdlib.h>
#include "optic_flow_gdc.h"

#define int_index(x,y) (y * IMG_WIDTH + x)
#define uint_index(xx, yy) (((yy * IMG_WIDTH + xx) * 2) & 0xFFFFFFFC)
#define NO_MEMORY -1
#define OK 0
#define N_VISUAL_INPUTS 51
#define N_ACTIONS 3

unsigned int IMG_WIDTH, IMG_HEIGHT;
int weights[153] = {-78, -46, 18, 59, 0, 100, 0, 0, 100, -29, -45, 0, 15, -30, 59, -100, -99, -100, -47, 0, -100, -100, 2, -78, 0, 10, -68, 53, 0, 0, -61, -28, 51, 0, -86, -73, 10, -65, -100, 98, -19, 63, -100, -42, -83, 21, 0, 3, 7, 0, -100, 24, -100, -99, -40, -100, 91, 0, 0, 54, 0, -90, -22, 13, 6, 31, 0, 100, -58, -31, 100, 5, 21, -100, 37, -100, 57, 100, -96, -3, -74, -3, -64, -68, 6, -100, -71, -81, 100, 13, 100, 0, -100, -57, 77, -100, -61, -100, 0, 37, -100, -100, -100, 10, -36, -100, 62, 8, 0, 21, 2, -61, -5, 32, -64, 15, -100, -90, -74, -18, -22, -28, 42, -92, 0, 3, -3, -13, 100, -5, 88, 0, 7, -100, 90, 73, -53, 100, 0, 2, 0, -95, -60, -62, 0, -6, 82, 0, -79, -69, 73, -38, 100};

void getVisualInputs(unsigned char *frame_buf, int x, int y, int* visual_inputs, unsigned int half_patch);
void applyNeuralNetwork(int* visual_inputs, int* actions, int RESOLUTION);


static inline void bluePixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0xff;
  frame_buf[ip+1] = 0xff;
  frame_buf[ip+2] = 0x00;
  frame_buf[ip+3] = 0xff;
}

static inline void redPixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0x00;
  frame_buf[ip+1] = 0xff;
  frame_buf[ip+2] = 0xff;
  frame_buf[ip+3] = 0xff;
}

static inline void greenPixel(unsigned char *frame_buf, unsigned int ip)
{
  frame_buf[ip] = 0x00;
  frame_buf[ip+1] = 0xff;
  frame_buf[ip+2] = 0x00;
  frame_buf[ip+3] = 0xff;
}

void multiplyImages(int* ImA, int* ImB, int* ImC, int width, int height)
{
  int x,y;
  unsigned int ix;

  // printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  for(x = 0; x < width; x++)
  {
    for(y = 0; y < height; y++)
    {
      ix = (y * width + x);
      ImC[ix] = ImA[ix] * ImB[ix];
      // If we want to keep the values in [0, 255]:
      // ImC[ix] /= 255;
    }
  }
}

void getImageDifference(int* ImA, int* ImB, int* ImC, int width, int height)
{
  int x,y;
  unsigned int ix;

  // printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  for(x = 0; x < width; x++)
  {
    for(y = 0; y < height; y++)
    {
      ix = (y * width + x);
      ImC[ix] = ImA[ix] - ImB[ix];
    }
  }

}

void getGradientPixelWH(unsigned char *frame_buf, int x, int y, int* dx, int* dy)
{
  unsigned int ix, Y1, Y2;
  unsigned int xx, yy;
  // currently we use [0 0 0; -1 0 1; 0 0 0] as mask for dx
  if(x >= 0 && x < (int)IMG_WIDTH && y >= 0 && y < (int)IMG_HEIGHT)
  {
    if(x > 0)
    {
      xx = x - 1;
      yy = y;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = 0; yy = y;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    if(x < (int)IMG_WIDTH - 1)
    {
      xx = x+1; yy = y;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = IMG_WIDTH - 1; yy = y;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    (*dx) = ((int)Y2) - ((int)Y1);
    /*
		if((*dx) > 510 || (*dx) < -510)
		{
			printf("\n\r*** dx = %d, Y1 = %d, Y2 = %d ***\n\r", (*dx), Y1, Y2);
		}
     */

    if(y > 0)
    {
      xx = x; yy = y - 1;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = x; yy = 0;
      ix = uint_index(xx,yy);
      Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    if(y < (int)IMG_HEIGHT - 1)
    {
      xx = x; yy = y + 1;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }
    else
    {
      xx = x; yy = IMG_HEIGHT-1;
      ix = uint_index(xx,yy);
      Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
    }

    (*dy) = ((int)Y2) - ((int)Y1);
    /*
		if((*dx) > 510 || (*dx) < -510)
		{
			printf("\n\r*** dy = %d, Y1 = %d, Y2 = %d ***\n\r", (*dy), Y1, Y2);
		}
     */
  }
}


void getSimpleGradient(unsigned char* frame_buf, int* DX, int* DY)
{
  unsigned int x,y,ix;
  int dx,dy;
  for(x = 0; x < IMG_WIDTH; x++)
  {
    for(y = 0; y < IMG_HEIGHT; y++)
    {
      // get dx, dy for the current position:
      getGradientPixelWH(frame_buf, x, y, &dx, &dy);
      // put it in the matrix:
      ix = int_index(x,y);
      DX[ix] = dx;
      DY[ix] = dy;

    }
  }
}

void excludeArea(unsigned int* Mask, int x, int y, int suppression_distance_squared)
{
  // suppression_distance_squared:
  // in a future implementation we can make a circular mask used to exclude pixels,
  // for now we just implement a square with sides equal to this value
  int xx, yy;
  unsigned int ix;
  //printf("suppression distance = %d\n\r", suppression_distance_squared);
  for(xx = x-suppression_distance_squared; xx<= x+suppression_distance_squared; xx++)
  {
    for(yy = y -suppression_distance_squared; yy < y+suppression_distance_squared; yy++)
    {
      if(yy >= 0 && yy < (int)IMG_HEIGHT && xx >= 0 && xx < (int)IMG_WIDTH)
      {
        ix = int_index(xx,yy);
        Mask[ix] = 1;
      }
    }
  }
}


int findLocalMaxima(int* Harris, int max_val, int MAX_POINTS, int* p_x, int* p_y, int suppression_distance_squared, int* n_found_points)
{
  //printf("W = %d, H = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

  unsigned int* Mask; // Mask contains pixels that are excluded
  int x, y, local_max, xx, yy;
  unsigned int ix, iy;
  Mask = (unsigned int *) malloc(IMG_WIDTH * IMG_HEIGHT * sizeof(unsigned int));
  if(Mask == 0) return NO_MEMORY;
  (*n_found_points) = 0;
  // initialize with zeros (none excluded yet)
  for(x = 0; x < (int)IMG_WIDTH; x++)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      Mask[ix] = 0;
    }
  }

  //printf("MAX_POINTS = %d\n\r", MAX_POINTS);

  // Find local maxima, forget about the border pixels:
  for(x = 1; x < (int)IMG_WIDTH-1; x++)
  {
    for(y = 1; y < (int)IMG_HEIGHT-1; y++)
    {
      // stop if the maximum number of points has been reached:
      if((*n_found_points) == MAX_POINTS) break;

      ix = int_index(x,y);
      // only evaluate a pixel if it is not too close to another corner:
      if(Mask[ix] == 0)
      {
        // is the pixel higher than its direct neighbors?
        local_max = 1;
        for(xx = x - 1; xx <= x + 1; xx++)
        {
          for(yy = y - 1; yy <= y + 1; yy++)
          {
            iy = int_index(xx,yy);
            if(iy != ix)
            {
              if(Harris[iy] >= Harris[ix])
              {
                local_max = 0;
                break;
              }
            }
          }
          if(!local_max) break;
        }

        if(local_max)
        {
          // printf("maximum at %d,%d\n\r", x, y);
          // store the point:
          p_x[(*n_found_points)] = x;
          p_y[(*n_found_points)] = y;
          (*n_found_points)++;

          // clear the area around the point:
          excludeArea(Mask, x, y, suppression_distance_squared);
          //mask_val = Mask[ix];
          //ix = int_index(x,y+3);
          //printf("Mask x,y = %d, x,y+3 = %d", mask_val, Mask[ix]);
        }
      }
    }
    // stop if the maximum number of points has been reached:
    if((*n_found_points) == MAX_POINTS) break;
  }
  // free Mask:
  free((char*)Mask);
  return OK;
}

// This function gives the maximum of a subsampled version of the image
int getMaximum(int * Im)
{
  int x, y, val, max_val;
  unsigned int ix;
  int step = 1;
  max_val = Im[0];
  for(x = 0; x < (int)IMG_WIDTH; x += step)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y += step)
    {
      ix = int_index(x,y);
      val = Im[ix];
      if(val > max_val) max_val = val;
    }
  }
  return max_val;
}

// This function gives the maximum of a subsampled version of the image
int getMinimum(int * Im)
{
  int x, y, val, min_val;
  unsigned int ix;
  int step = 1;
  min_val = Im[0];
  for(x = 0; x < (int)IMG_WIDTH; x += step)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y += step)
    {
      ix = int_index(x,y);
      val = Im[ix];
      if(val < min_val) min_val = val;
    }
  }
  return min_val;
}


void getHarris(int* DXX, int* DXY, int* DYY, int* Harris)
{
  // Harris = (dx2 * dy2 - dxy*dxy) - ((dx2 + dy2) * (dx2 + dy2)) / 25;
  int x, y, sumDXXDYY;
  unsigned int ix;
  int reciprocal_K = 25;
  //int printed = 0;
  //int PRECISION = 1;
  for(x = 0; x < (int)IMG_WIDTH; x++)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      sumDXXDYY = DXX[ix] + DYY[ix];
      if(sumDXXDYY > 255) sumDXXDYY = 255;
      Harris[ix] = (DXX[ix] * DYY[ix] - DXY[ix] * DXY[ix]) - (sumDXXDYY * sumDXXDYY) / reciprocal_K;

      /*
			if(printed < 10 && (Harris[ix] > 65025 || Harris[ix] < -65025))
			{
				printf("dxxdyy = %d, dxydxy = %d, (dxx+dyy) = %d, Harris / k = %d, Harris = %d\n\r", DXX[ix] * DYY[ix], DXY[ix] * DXY[ix], sumDXXDYY, (DXX[ix] * DYY[ix] - DXY[ix] * DXY[ix]) - (sumDXXDYY * sumDXXDYY), (DXX[ix] * DYY[ix] - DXY[ix] * DXY[ix]) - (sumDXXDYY * sumDXXDYY) / reciprocal_K);
				printed++;
			}
       */

      //Harris[ix] = (DXX[ix] * DYY[ix] / PRECISION - DXY[ix] * DXY[ix] / PRECISION) - ((sumDXXDYY / PRECISION) * (sumDXXDYY / PRECISION)) / reciprocal_K;
    }
  }
}

void smoothGaussian(int* Src, int* Dst)
{
  int x, y, min_x, min_y, xx, yy, it;
  unsigned int ix, ixx;
  int smooth[9]; // = {1,2,1,2,4,2,1,2,1};
  int smooth_factor_1 = 14; // retain energy
  int smooth_factor_2 = 255; // for Harris to stay within
  //int printed = 0;

  // set the smoothing filter:
  smooth[0] = 1;
  smooth[1] = 2;
  smooth[2] = 1;
  smooth[3] = 2;
  smooth[4] = 4;
  smooth[5] = 2;
  smooth[6] = 1;
  smooth[7] = 2;
  smooth[8] = 1; // in MATLAB-language: [1,2,1;2,4,2;1,2,1]

  // is the following necessary, or is it already filled with zeros?
  // fill the borders with zeros:
  for(x = 0; x < (int)IMG_WIDTH; x += (int)IMG_WIDTH-1)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      Dst[ix] = 0;
    }
  }
  for(y = 0; y < (int)IMG_HEIGHT; y += (int)IMG_HEIGHT-1)
  {
    for(x = 0; x < (int)IMG_WIDTH; x++)
    {
      ix = int_index(x,y);
      Dst[ix] = 0;
    }
  }

  for(x = 1; x < (int)IMG_WIDTH-1; x++)
  {
    for(y = 1; y < (int)IMG_HEIGHT-1; y++)
    {
      min_x = x - 1;
      min_y = y - 1;
      ix = int_index(x,y);
      Dst[ix] = 0;
      // use the patch to determine dx2, dxy, and dy2
      it = 0;
      for(yy = min_y; yy < min_y + 3; yy++)
      {
        for(xx = min_x; xx < min_x + 3; xx++)
        {
          ixx = int_index(xx,yy);

          Dst[ix] += smooth[it] * (Src[ixx] / smooth_factor_1);

          /*if(printed < 9 && x > 30 && y > 30)
					{
						printf("SG: it = %d, s = %d, src = %d, src / sf = %d, filtered = %d, Dst = %d\n\r", it, smooth[it], Src[ixx], Src[ixx] / smooth_factor_1, smooth[it] * (Src[ixx] / smooth_factor_1), Dst[ix]);
						printed += 1;
					}*/

          // update iterator of the smoothing filter:
          it++;
        }
      }

      Dst[ix] /= smooth_factor_2;
    }
  }
}

void thresholdImage(int* Harris, int max_val, int max_factor)
{
  // only retain values that are larger than max_val / max_factor:
  // are there images for which Harris' maximal value is negative?
  int x, y, threshold, n_remaining;
  unsigned int ix;
  threshold = max_val / max_factor;
  n_remaining = 0;
  for(x = 0; x < (int)IMG_WIDTH; x++)
  {
    for(y = 0; y < (int)IMG_HEIGHT; y++)
    {
      ix = int_index(x,y);
      if(Harris[ix] < threshold) Harris[ix] = 0;
      else n_remaining++;
    }
  }
  //printf("Remaining points = %d\n\r", n_remaining);
}

int findCorners(unsigned char *frame_buf, int MAX_POINTS, int *x, int *y, int suppression_distance_squared, int* n_found_points, int mark_points, int imW, int imH)
{
  // Algorithmic steps:
  // (1) get the dx, dy gradients in the image
  // (2) determine dxx, dxy, dyy
  // (3) smooth dxx, dxy, dyy
  // (4) determine the Harris values
  // (5) find maximal values (with suppression of points close by)
  // (6) mark the points green in the image

  int max_val, error, p, i, j; // min_val,
  unsigned int ix;
  unsigned int im_size;
  int* DX; int* DY; int* DXX; int* DXY; int* DYY;
  int* SDXX; int* SDXY; int* SDYY; int* Harris;
  // for debugging:
  //int n_rows, n_cols, x_center, y_center;

  IMG_WIDTH = imW;
  IMG_HEIGHT = imH;
  im_size = IMG_WIDTH * IMG_HEIGHT;
  // for debugging:
  //n_rows = 20; n_cols = 20;
  //x_center = IMG_WIDTH / 2;
  //y_center = IMG_HEIGHT / 2;

  // (1) get the dx, dy gradients in the image
  DX = (int *) malloc(im_size * sizeof(int));
  DY = (int *) malloc(im_size * sizeof(int));
  if(DX == 0 || DY == 0) return NO_MEMORY;
  getSimpleGradient(frame_buf, DX, DY);

  // DEBUGGING:
  //i_center = int_index(x_center, y_center);
  /*printf("DX = ");
	printIntMatrixPart(DX, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("DY = ");
	printIntMatrixPart(DY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  //max_val = getMaximum(DX);
  //min_val = getMinimum(DX);
  //printf("DX[center] = %d, max = %d, min = %d\n\r", DX[i_center], max_val, min_val);

  // (2) determine dxx, dxy, dyy

  // form DXX and DXY
  DXX = (int *) malloc(im_size * sizeof(int));
  DXY = (int *) malloc(im_size * sizeof(int));
  if(DXX == 0 || DXY == 0) return NO_MEMORY;
  multiplyImages(DX, DX, DXX, IMG_WIDTH, IMG_HEIGHT);
  multiplyImages(DX, DY, DXY, IMG_WIDTH, IMG_HEIGHT);
  // free DX
  free((char*) DX);

  // DEBUGGING:
  /*	printf("DXY = ");
	printIntMatrixPart(DXY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("DXX = ");
	printIntMatrixPart(DXX, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  //max_val = getMaximum(DXX);
  //min_val = getMinimum(DXX);
  //printf("DXX[center] = %d, max = %d, min = %d\n\r", DXX[i_center], max_val, min_val);

  // form DYY
  DYY = (int *) malloc(im_size * sizeof(int));
  if(DYY == 0) return NO_MEMORY;
  multiplyImages(DY, DY, DYY, IMG_WIDTH, IMG_HEIGHT);
  // free DY
  free((char*) DY);

  // (3) smooth dxx, dxy, dyy
  // unfortunately, this smoothing is quite necessary:

  SDXX = (int *) malloc(im_size * sizeof(int));
  if(SDXX == 0) return NO_MEMORY;
  smoothGaussian(DXX, SDXX);

  // free DXX
  free((char*) DXX);

  // DEBUGGING:
  /*	printf("DYY = ");
	printIntMatrixPart(DYY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("SDXX = ");
	printIntMatrixPart(SDXX, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  //max_val = getMaximum(SDXX);
  //min_val = getMinimum(SDXX);
  //printf("SDXX[center] = %d, max = %d, min = %d\n\r", SDXX[i_center], max_val, min_val);

  SDXY = (int *) malloc(im_size * sizeof(int));
  if(SDXY == 0) return NO_MEMORY;
  smoothGaussian(DXY, SDXY);


  // free DXY
  free((char*) DXY);

  SDYY = (int *) malloc(im_size * sizeof(int));
  smoothGaussian(DYY, SDYY);
  /*
	printf("SDYY = ");
	printIntMatrixPart(SDYY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
	printf("SDXY = ");
	printIntMatrixPart(SDXY, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
   */
  if(SDXX == 0 || SDXY == 0 || SDYY == 0) return NO_MEMORY;
  // free DYY
  free((char*) DYY);

  // (4) determine the Harris values
  Harris = (int *) malloc(im_size * sizeof(int));
  if(Harris == 0) return NO_MEMORY;
  getHarris(SDXX, SDXY, SDYY, Harris);
  //printf("Harris1 = ");
  //printIntMatrixPart(Harris, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);

  // free SDXX, SDYY, SDXY:
  free((char*)SDXX); free((char*)SDYY); free((char*)SDXY);

  // DEBUGGING:
  //max_val = getMaximum(Harris);
  //min_val = getMinimum(Harris);
  //printf("Harris[center] = %d, max = %d, min = %d\n\r", Harris[i_center], max_val, min_val);

  // (5) find maximal values (with suppression of points close by)
  // (a) find the maximum
  max_val = getMaximum(Harris);
  // (b) threshold the image on the basis of the found (approximative) maximum:
  thresholdImage(Harris, max_val, 5);
  //printf("Harris2 = ");
  //printIntMatrixPart(Harris, IMG_WIDTH, IMG_HEIGHT, n_cols, n_rows, x_center, y_center);
  // (c) find local maxima
  error = findLocalMaxima(Harris, max_val, MAX_POINTS, x, y, suppression_distance_squared, n_found_points);
  if(error == NO_MEMORY) return NO_MEMORY;
  // free Harris:
  free((char*) Harris);

  // DEBUGGING:
  // printf("Points found: %d\n\r", (*n_found_points));

  // (6) mark the points green in the image
  if(mark_points > 0)
  {
    // printf("IMG_WIDTH = %d, IMG_HEIGHT = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

    for(p = 0; p < (*n_found_points); p++)
    {
      if(x[p] >= 1 && y[p] >= 1 && x[p] < (int)IMG_WIDTH - 1 && y[p] < (int)IMG_HEIGHT - 1)
      {
        // printf("(x,y) = (%d,%d)\n\r", x[p],y[p]);
        for(i = -1; i <= 1; i++)
        {
          for(j = -1; j <= 1; j++)
          {
            // printf("(x+i,y+j) = (%d,%d)\n\r", x[p]+i,y[p]+j);
            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
            // printf("ix = %d, ixx = %d\n\r", ix, (((y[p]+j) * IMG_WIDTH + (x[p]+i)) * 2) & 0xFFFFFFFC);
            redPixel(frame_buf, ix);
          }
        }
      }
    }
  }
  // routine successful:
  return OK;
}

int findActiveCorners(unsigned char *frame_buf, unsigned int GRID_ROWS, int ONLY_STOPPED, int *x, int *y, int* active, int* n_found_points, int mark_points, int imW, int imH)
{
  // Algorithmic steps:
  // 1) initialize the agents' positions
  // 2) let the agents search in the image
  // 3) select the points to be returned

  unsigned int grid_step_x, grid_step_y, border, n_agents, gr, gc, a;
  unsigned int t, n_time_steps, n_active, finished, active_agents;
  int visual_inputs[N_VISUAL_INPUTS];
  int actions[N_ACTIONS];
  int dx, dy, MAX_JUMP;
  unsigned int RESOLUTION, half_patch;
  int p, i, j;
  unsigned int ix;

  // copy image parameters:
  IMG_WIDTH = imW;
  IMG_HEIGHT = imH;

  // The resolution will allow subpixel positions:
  RESOLUTION = 100;

  // ***********************************
  // 1) initialize the agents' positions
  // ***********************************

  n_agents = GRID_ROWS * GRID_ROWS;
  border = 10;
  grid_step_x = (imW - 2 * border) / (GRID_ROWS-1);
  grid_step_y = (imH - 2 * border) / (GRID_ROWS-1);

  a = 0;
  for(gr = 0; gr < GRID_ROWS; gr++)
  {
    for(gc = 0; gc < GRID_ROWS; gc++)
    {
      x[a] = (border + gr * grid_step_x) * RESOLUTION;
      y[a] = (border + gc * grid_step_y) * RESOLUTION;
      active[a] = 1;
      a++;
      if(a == n_agents) break;
    }

    if(a == n_agents) break;
  }

  // *************************************
  // 2) let the agents search in the image
  // *************************************

  MAX_JUMP = 10;
  half_patch = 2;
  n_time_steps = 20;
  t = 0;
  finished = 0;
  while(finished == 0)
  {
    // number of agents still active:
    n_active = 0;

    // loop over the agents:
    for(a = 0; a < n_agents; a++)
    {
      // only extract inputs and move if still active:
      if(active[a] == 1)
      {
        // printf("Agent %d at (%d,%d)\n\r", a, x[a], y[a]);
        n_active++;

        // sensing, inputs in [0,255]:
        getVisualInputs(frame_buf, x[a] / RESOLUTION, y[a] / RESOLUTION, visual_inputs, half_patch);

        // determining actions, in [-RESOLUTION, RESOLUTION]:
        applyNeuralNetwork(visual_inputs, actions, RESOLUTION);

        // printf("Actions = (%d, %d, %d)\n\r", actions[0], actions[1], actions[2]);

        // possibly stopping:
        if(actions[2] < 0)
        {
          // printf("Agent %d stopped.\n\r", a);
          active[a] = 0;
        }

        // if moving:
        if(active[a] == 1)
        {
          dx = (actions[0] * MAX_JUMP);
          dy = (actions[1] * MAX_JUMP);

          x[a] += dx;
          y[a] += dy;

          // checking the limits, making the agents go round the image when they leave the image:
          if(x[a] / RESOLUTION < half_patch + 1) // the + 1 is necessary for the convolution with image filters
          {
            x[a] = (IMG_WIDTH - half_patch - 2) * RESOLUTION;
          }
          else if(x[a] / RESOLUTION > IMG_WIDTH - half_patch - 2)
          {
            x[a] = (half_patch + 1) * RESOLUTION;
          }
          if(y[a] / RESOLUTION < half_patch + 1) // the + 1 is necessary for the convolution with image filters
          {
            y[a] = (IMG_HEIGHT - half_patch - 2) * RESOLUTION;
          }
          else if(y[a] / RESOLUTION > IMG_HEIGHT - half_patch - 2)
          {
            y[a] = (half_patch + 1) * RESOLUTION;
          }

        }
      }
    }

    // update time and check end conditions:
    // printf("Time = %d / %d\n\r", t, n_time_steps);
    t++;
    if(t == n_time_steps || n_active == 0)
    {
      finished = 1;
    }
  }

  // ***********************************
  // 3) select the points to be returned
  // ***********************************

  if(!ONLY_STOPPED)
  {
    for(a = 0; a < n_agents; a++)
    {
      x[a] = x[a] / RESOLUTION;
      y[a] = y[a] / RESOLUTION;
    }
    (*n_found_points) = n_agents;
  }
  else
  {

    active_agents = 0;

    for(a = 0; a < n_agents; a++)
    {
      if(active[a] == 0)
      {
        x[active_agents] = x[a] / RESOLUTION;
        y[active_agents] = y[a] / RESOLUTION;
        active_agents++;
      }
    }

    (*n_found_points) = active_agents;
  }

  // ********************************
  // (4) mark the points in the image
  // ********************************

  if(mark_points > 0)
  {
    //printf("Mark points\n\r");
    // printf("IMG_WIDTH = %d, IMG_HEIGHT = %d\n\r", IMG_WIDTH, IMG_HEIGHT);

    for(p = 0; p < (*n_found_points); p++)
    {
      if(x[p] >= 1 && y[p] >= 1 && x[p] < (int)IMG_WIDTH - 1 && y[p] < (int)IMG_HEIGHT - 1)
      {
        //printf("(x,y) = (%d,%d)\n\r", x[p],y[p]);
        for(i = -1; i <= 1; i++)
        {
          for(j = -1; j <= 1; j++)
          {
            // printf("(x+i,y+j) = (%d,%d)\n\r", x[p]+i,y[p]+j);
            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
            // printf("ix = %d, ixx = %d\n\r", ix, (((y[p]+j) * IMG_WIDTH + (x[p]+i)) * 2) & 0xFFFFFFFC);
            redPixel(frame_buf, ix);
          }
        }
      }
    }
  }


  return 0;
}

void getVisualInputs(unsigned char *frame_buf, int x, int y, int* visual_inputs, unsigned int half_patch)
{
  int i, dx, dy, half_inputs, xx, yy;
  // will round off correctly (bias is counted as visual input):
  half_inputs = N_VISUAL_INPUTS / 2;
  i = 0;
  for(xx = x - (int)half_patch; xx <= x + (int)half_patch; xx++)
  {
    for(yy = y - (int)half_patch; yy <= y + (int)half_patch; yy++)
    {
      //printf("gp(%d, %d), ", xx, yy);
      getGradientPixelWH(frame_buf, xx, yy, &dx, &dy);
      visual_inputs[i] = dx;
      visual_inputs[half_inputs+i] = dy;
      i++;
    }
  }
  // bias, scaled to the input range:
  visual_inputs[N_VISUAL_INPUTS-1] = 255;

  /*printf("Visual inputs:\n");
	for(i = 0; i < N_VISUAL_INPUTS; i++)
	{
		printf("%d, ", visual_inputs[i]);
	}
	printf("\n\r");*/

  return;
}

void applyNeuralNetwork(int* visual_inputs, int* actions, int RESOLUTION)
{

  int a, i, w, factor;

  //printf("NN\n\r");

  // inputs in [0,255] weights in [-100,100]
  // 51 weights per neuron
  // 25500 * 51 = 1,300,500 (max)
  // float point:
  // inputs in [-1, 1] weights in [-1, 1]
  // 51 weights, 51 (max) (* RESOLUTION = 5100)
  // but the max are very unlikely, so:
  factor = 25500 / RESOLUTION;
  w = 0;
  // Perceptron neural network:
  for(a = 0; a < N_ACTIONS; a++)
  {
    actions[a] = 0;

    for(i = 0; i < N_VISUAL_INPUTS; i++)
    {
      actions[a] += weights[w] * visual_inputs[i];
      w++;
    }

    //printf("actions[%d] after summing: %d\n\r", a, actions[a]);

    // account for changes in value ranges (25500 vs. 1 -> 1*RESOLUTION)
    actions[a] /= factor;

    //printf("actions[%d] after factor: %d\n\r", a, actions[a]);

    // activation function:
    actions[a] /= 2;

    //printf("actions[%d] after /2: %d\n\r", a, actions[a]);

    // saturate at RESOLUTION / -RESOLUTION:
    actions[a] = (actions[a] > RESOLUTION) ? RESOLUTION : actions[a];
    actions[a] = (actions[a] < -RESOLUTION) ? -RESOLUTION : actions[a];

    //printf("actions[%d] after saturation: %d\n\r", a, actions[a]);

  }

  return;
}



void getSubPixel(int* Patch, unsigned char* frame_buf, int center_x, int center_y, int half_window_size, int subpixel_factor)
{
  int x, y, x_0, y_0, x_0_or, y_0_or, i, j, window_size, alpha_x, alpha_y, max_x, max_y;
  //int printed, limit;
  unsigned int ix1, ix2, Y;
  window_size = half_window_size * 2 + 1;
  max_x = (IMG_WIDTH-1)*subpixel_factor;
  max_y = (IMG_HEIGHT-1)*subpixel_factor;
  //printed = 0; limit = 4;

  for(i = 0; i < window_size; i++)
  {
    for(j = 0; j < window_size; j++)
    {
      // index for this position in the patch:
      ix1 = (j * window_size + i);

      // determine subpixel coordinates of the current pixel:
      x = center_x + (i - half_window_size) * subpixel_factor;
      if(x < 0) x = 0;
      if(x > max_x) x = max_x;
      y = center_y + (j - half_window_size) * subpixel_factor;
      if(y < 0) y = 0;
      if(y > max_y) y = max_y;
      // pixel to the top left:
      x_0_or = (x / subpixel_factor);
      x_0 = x_0_or * subpixel_factor;
      y_0_or = (y / subpixel_factor);
      y_0 = y_0_or * subpixel_factor;
      /*if(printed < limit)
			{
				printf("x_0_or = %d, y_0_or = %d;\n\r", x_0_or, y_0_or);
				printf("x_0 = %d, y_0 = %d\n\r");
				printed++;
			}*/


      if(x == x_0 && y == y_0)
      {
        // simply copy the pixel:
        ix2 = uint_index(x_0_or, y_0_or);
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        Patch[ix1] = (int) Y;
      }
      else
      {
        // blending according to how far the subpixel coordinates are from the pixel coordinates
        alpha_x = (x - x_0);
        alpha_y = (y - y_0);

        // the patch pixel is a blend from the four surrounding pixels:
        ix2 = uint_index(x_0_or, y_0_or);
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        Patch[ix1] = (subpixel_factor - alpha_x) * (subpixel_factor - alpha_y) * ((int) Y);

        ix2 = uint_index((x_0_or+1), y_0_or);
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: TR = %d\n\r", Y);
        Patch[ix1] += alpha_x * (subpixel_factor - alpha_y) * ((int) Y);

        ix2 = uint_index(x_0_or, (y_0_or+1));
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: BL = %d\n\r", Y);
        Patch[ix1] += (subpixel_factor - alpha_x) * alpha_y * ((int) Y);

        ix2 = uint_index((x_0_or+1), (y_0_or+1));
        Y = ((unsigned int)frame_buf[ix2+1] + (unsigned int)frame_buf[ix2+3]) >> 1;
        //if(printed < limit) printf("subpixel: BR = %d\n\r", Y);
        Patch[ix1] += alpha_x * alpha_y * ((int) Y);

        // normalize patch value
        Patch[ix1] /= (subpixel_factor * subpixel_factor);

        /*if(printed < limit)
				{

					printf("alpha_x = %d, alpha_y = %d, x_0 = %d, y_0 = %d, x = %d, y = %d, Patch[ix1] = %d\n\r", alpha_x, alpha_y, x_0, y_0, x, y, Patch[ix1]);
					// printed++;
				}
         */

      }
    }
  }

  return;
}

void getGradientPatch(int* Patch, int* DX, int* DY, int half_window_size)
{
  unsigned int ix1, ix2;
  int x, y, padded_patch_size, patch_size, Y1, Y2;
  //	int printed; printed = 0;

  padded_patch_size = 2 * (half_window_size + 1)+ 1;
  patch_size = 2 * half_window_size + 1;
  // currently we use [0 0 0; -1 0 1; 0 0 0] as mask for dx
  for(x = 1; x < padded_patch_size - 1; x++)
  {
    for(y = 1; y < padded_patch_size - 1; y++)
    {
      // index in DX, DY:
      ix2 = (unsigned int) ((y-1) * patch_size + (x-1));

      ix1 = (unsigned int) (y * padded_patch_size + x-1);
      Y1 = Patch[ix1];
      ix1 = (unsigned int) (y * padded_patch_size + x+1);
      Y2 = Patch[ix1];
      DX[ix2] = Y2 - Y1;

      ix1 = (unsigned int) ((y-1) * padded_patch_size + x);
      Y1 = Patch[ix1];
      ix1 = (unsigned int) ((y+1) * padded_patch_size + x);
      Y2 = Patch[ix1];
      DY[ix2] = Y2 - Y1;

      /*if(printed < 1 && DX[ix2] > 0)
			{
				printf("DX = %d, DY = %d\n\r", DX[ix2], DY[ix2]);
				printed++;
			}
			else if(printed == 1 && DX[ix2] < 0)
			{
				printf("DX = %d, DY = %d\n\r", DX[ix2], DY[ix2]);
				printed++;
			}*/


    }
  }

  return;
}

int getSumPatch(int* Patch, int size)
{
  int x, y, sum; // , threshold
  unsigned int ix;

  // in order to keep the sum within range:
  //threshold = 50000; // typical values are far below this threshold

  sum = 0;
  for(x = 0; x < size; x++)
  {
    for(y = 0; y < size; y++)
    {
      ix = (y * size) + x;
      //if(sum < threshold && sum > -threshold)
      //{
      sum += Patch[ix]; // do not check thresholds
      //}
      /*else
			{
				if(sum > threshold)
				{
					sum = threshold;
				}
				else
				{
					sum = -threshold;
				}
			}*/
    }
  }

  return sum;
}

int calculateG(int* G, int* DX, int* DY, int half_window_size)
{
  int patch_size;
  int* DXX; int* DXY; int* DYY;

  patch_size = 2 * half_window_size + 1;

  // allocate memory:
  DXX = (int *) malloc(patch_size * patch_size * sizeof(int));
  DXY = (int *) malloc(patch_size * patch_size * sizeof(int));
  DYY = (int *) malloc(patch_size * patch_size * sizeof(int));

  if(DXX == 0 || DXY == 0 || DYY == 0)
    return NO_MEMORY;

  // then determine the second order gradients
  multiplyImages(DX, DX, DXX, patch_size, patch_size);
  multiplyImages(DX, DY, DXY, patch_size, patch_size);
  multiplyImages(DY, DY, DYY, patch_size, patch_size);

  // calculate G:
  G[0] = getSumPatch(DXX, patch_size);
  G[1] = getSumPatch(DXY, patch_size);
  G[2] = G[1];
  G[3] = getSumPatch(DYY, patch_size);

  // free memory:
  free((char*) DXX); free((char*) DXY); free((char*) DYY);

  // no errors:
  return OK;
}



int calculateError(int* ImC, int width, int height)
{
  int x,y, error;
  unsigned int ix;

  error = 0;

  for(x = 0; x < width; x++)
  {
    for(y = 0; y < height; y++)
    {
      ix = (y * width + x);
      error += ImC[ix]*ImC[ix];
    }
  }

  return error;
}

int opticFlowLK(unsigned char * new_image_buf, unsigned char * old_image_buf, int* p_x, int* p_y, int n_found_points, int imW, int imH, int* new_x, int* new_y, int* status, int half_window_size, int max_iterations)
{
  // A straightforward one-level implementation of Lucas-Kanade.
  // For all points:
  // (1) determine the subpixel neighborhood in the old image
  // (2) get the x- and y- gradients
  // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
  // (4) iterate over taking steps in the image to minimize the error:
  //     [a] get the subpixel neighborhood in the new image
  //     [b] determine the image difference between the two neighborhoods
  //     [c] calculate the 'b'-vector
  //     [d] calculate the additional flow step and possibly terminate the iteration
  int p, subpixel_factor, x, y, it, step_threshold, step_x, step_y, v_x, v_y, Det;
  int b_x, b_y, patch_size, padded_patch_size, error, step_size;
  unsigned int ix1, ix2;
  int* I_padded_neighborhood; int* I_neighborhood; int* J_neighborhood;
  int* DX; int* DY; int* ImDiff; int* IDDX; int* IDDY;
  int G[4];
  int error_threshold;

  // set the image width and height
  IMG_WIDTH = imW;
  IMG_HEIGHT = imH;
  // spatial resolution of flow is 1 / subpixel_factor
  subpixel_factor = 10;
  // determine patch sizes and initialize neighborhoods
  patch_size = (2*half_window_size + 1);
  error_threshold = (25 * 25) * (patch_size * patch_size);

  padded_patch_size = (2*half_window_size + 3);
  I_padded_neighborhood = (int *) malloc(padded_patch_size * padded_patch_size * sizeof(int));
  I_neighborhood = (int *) malloc(patch_size * patch_size * sizeof(int));
  J_neighborhood = (int *) malloc(patch_size * patch_size * sizeof(int));
  if(I_padded_neighborhood == 0 || I_neighborhood == 0 || J_neighborhood == 0)
    return NO_MEMORY;
  DX = (int *) malloc(patch_size * patch_size * sizeof(int));
  DY = (int *) malloc(patch_size * patch_size * sizeof(int));
  IDDX = (int *) malloc(patch_size * patch_size * sizeof(int));
  IDDY = (int *) malloc(patch_size * patch_size * sizeof(int));
  ImDiff = (int *) malloc(patch_size * patch_size * sizeof(int));
  if(DX == 0 || DY == 0 || ImDiff == 0 || IDDX == 0 || IDDY == 0)
    return NO_MEMORY;

  for(p = 0; p < n_found_points; p++)
  {
    //printf("*** NEW POINT ***\n\r");
    // status: point is not yet lost:
    status[p] = 1;

    //printf("Normal coordinate: (%d,%d)\n\r", p_x[p], p_y[p]);
    // We want to be able to take steps in the image of 1 / subpixel_factor:
    p_x[p] *= subpixel_factor;
    p_y[p] *= subpixel_factor;
    //printf("Subpixel coordinate: (%d,%d)\n\r", p_x[p], p_y[p]);

    // if the pixel is outside the ROI in the image, do not track it:
    if(!(p_x[p] > ((half_window_size+1) * subpixel_factor) && p_x[p] < ((int)IMG_WIDTH-half_window_size) * subpixel_factor && p_y[p] > ((half_window_size+1) * subpixel_factor) && p_y[p] < ((int)IMG_HEIGHT-half_window_size)*subpixel_factor))
    {
      //printf("Outside of ROI\n\r");
      status[p] = 0;
    }


    // (1) determine the subpixel neighborhood in the old image
    // we determine a padded neighborhood with the aim of subsequent gradient processing:
    getSubPixel(I_padded_neighborhood, old_image_buf, p_x[p], p_y[p], half_window_size+1, subpixel_factor);
    // Also get the original-sized neighborhood
    for(x = 1; x < padded_patch_size - 1; x++)
    {
      for(y = 1; y < padded_patch_size - 1; y++)
      {
        ix1 = (y * padded_patch_size + x);
        ix2 = ((y-1) * patch_size + (x-1));
        I_neighborhood[ix2] = I_padded_neighborhood[ix1];
      }
    }

    // (2) get the x- and y- gradients
    getGradientPatch(I_padded_neighborhood, DX, DY, half_window_size);

    // (3) determine the 'G'-matrix [sum(Axx) sum(Axy); sum(Axy) sum(Ayy)], where sum is over the window
    error = calculateG(G, DX, DY, half_window_size);
    if(error == NO_MEMORY) return NO_MEMORY;

    for(it = 0; it < 4; it++)
    {
      //printf("G[%d] = %d\n\r", it, G[it]);
      G[it] /= 255; // to keep values in range
      //printf("G[%d] = %d\n\r", it, G[it]);
    }
    // calculate G's determinant:
    Det = G[0] * G[3] - G[1] * G[2];
    //printf("Det = %d\n\r", Det);
    Det = Det / subpixel_factor; // so that the steps will be expressed in subpixel units
    //printf("Det = %d\n\r", Det);
    if(Det < 1)
    {
      status[p] = 0;
    }

    // (4) iterate over taking steps in the image to minimize the error:
    it = 0;
    step_threshold = 2; // 0.2 as smallest step (L1)
    v_x = 0;
    v_y = 0;
    step_size = step_threshold + 1;

    while(status[p] == 1 && it < max_iterations && step_size >= step_threshold)
    {
      //printf("it = %d, (p_x+v_x,p_y+v_y) = (%d,%d)\n\r", it, p_x[p]+v_x, p_y[p]+v_y);
      //printf("it = %d;", it);
      // if the pixel goes outside the ROI in the image, stop tracking:
      if(!(p_x[p]+v_x > ((half_window_size+1) * subpixel_factor) && p_x[p]+v_x < ((int)IMG_WIDTH-half_window_size) * subpixel_factor && p_y[p]+v_y > ((half_window_size+1) * subpixel_factor) && p_y[p]+v_y < ((int)IMG_HEIGHT-half_window_size)*subpixel_factor))
      {
        //printf("Outside of ROI\n\r");
        status[p] = 0;
        break;
      }

      //     [a] get the subpixel neighborhood in the new image


      // clear J:
      for(x = 0; x < patch_size; x++)
      {
        for(y = 0; y < patch_size; y++)
        {
          ix2 = (y * patch_size + x);
          J_neighborhood[ix2] = 0;
        }
      }


      getSubPixel(J_neighborhood, new_image_buf, p_x[p]+v_x, p_y[p]+v_y, half_window_size, subpixel_factor);
      //     [b] determine the image difference between the two neighborhoods
      //printf("I = ");
      //printIntMatrix(I_neighborhood, patch_size, patch_size);
      //printf("J = ");
      //printIntMatrix(J_neighborhood, patch_size, patch_size);
      //getSubPixel(J_neighborhood, new_image_buf, subpixel_factor * ((p_x[p]+v_x)/subpixel_factor), subpixel_factor * ((p_y[p]+v_y) / subpixel_factor), half_window_size, subpixel_factor);
      //printf("J2 = ");
      //printIntMatrix(J_neighborhood, patch_size, patch_size);
      //printf("figure(); subplot(1,2,1); imshow(I/255); subplot(1,2,2); imshow(J/255);\n\r");
      getImageDifference(I_neighborhood, J_neighborhood, ImDiff, patch_size, patch_size);
      //printf("ImDiff = ");
      //printIntMatrix(ImDiff, patch_size, patch_size);
      error = calculateError(ImDiff, patch_size, patch_size);
      if(error > error_threshold && it > max_iterations / 2)
      {
        status[p] = 0;
        break;
      }
      //printf("error(%d) = %d;\n\r", it+1, error);
      //     [c] calculate the 'b'-vector
      //printf("DX = ");
      //printIntMatrix(DX, patch_size, patch_size);
      multiplyImages(ImDiff, DX, IDDX, patch_size, patch_size);
      //printf("IDDX = ");
      //printIntMatrix(IDDX, patch_size, patch_size);
      multiplyImages(ImDiff, DY, IDDY, patch_size, patch_size);
      //printf("DY = ");
      //printIntMatrix(DY, patch_size, patch_size);
      //printf("IDDY = ");
      //printIntMatrix(IDDY, patch_size, patch_size);
      //printf("figure(); subplot(2,3,1); imagesc(ImDiff); subplot(2,3,2); imagesc(DX); subplot(2,3,3); imagesc(DY);");
      //printf("subplot(2,3,4); imagesc(IDDY); subplot(2,3,5); imagesc(IDDX);\n\r");
      // division by 255 to keep values in range:
      b_x = getSumPatch(IDDX, patch_size) / 255;
      b_y = getSumPatch(IDDY, patch_size) / 255;
      //printf("b_x = %d; b_y = %d;\n\r", b_x, b_y);
      //     [d] calculate the additional flow step and possibly terminate the iteration
      step_x = (G[3] * b_x - G[1] * b_y) / Det;
      step_y = (G[0] * b_y - G[2] * b_x) / Det;
      v_x += step_x;
      v_y += step_y; // - (?) since the origin in the image is in the top left of the image, with y positive pointing down
      //printf("step = [%d,%d]; v = [%d,%d];\n\r", step_x, step_y, v_x, v_y);
      //printf("pause(0.5);\n\r");
      // next iteration
      it++;
      step_size = abs(step_x);
      step_size += abs(step_y);
      //printf("status = %d, it = %d, step_size = %d\n\r", status[p], it, step_size);
    } // iteration to find the right window in the new image

    //printf("figure(); plot(error(1:(it+1)));\n\r");

    new_x[p] = (p_x[p] + v_x) / subpixel_factor;
    new_y[p] = (p_y[p] + v_y) / subpixel_factor;
    p_x[p] /= subpixel_factor;
    p_y[p] /= subpixel_factor;
  }



  // free all allocated variables:
  free((char*) I_padded_neighborhood);
  free((char*) I_neighborhood);
  free((char*) J_neighborhood);
  free((char*) DX);
  free((char*) DY);
  free((char*) ImDiff);
  free((char*) IDDX);
  free((char*) IDDY);
  // no errors:
  return OK;
}

extern void showFlow(unsigned char * frame_buf, int* x, int* y, int* status, int n_found_points, int* new_x, int* new_y, int imgW, int imgH)
{
  int p, i, j;
  unsigned int ix;

  IMG_WIDTH = imgW;
  IMG_HEIGHT = imgH;
  // in this simple version, we do not draw lines:
  for(p = 0; p < n_found_points; p++)
  {
    //printf("Point %d, status = %d, (x, y) = (%d, %d), (new_x, new_y) = (%d, %d)\n\r", p, status[p], x[p], y[p], new_x[p], new_y[p]);
    if(x[p] >= 1 && y[p] >= 1 && x[p] < (int)IMG_WIDTH - 1 && y[p] < (int)IMG_HEIGHT - 1)
    {
      for(i = -1; i <= 1; i++)
      {
        for(j = -1; j <= 1; j++)
        {
          if(status[p] == 1)
          {
            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
            redPixel(frame_buf, ix);
            ix = uint_index((unsigned int) (new_x[p]+i), (unsigned int) (new_y[p]+j));
            greenPixel(frame_buf, ix);
          }
          else
          {
            ix = uint_index((unsigned int) (x[p]+i), (unsigned int) (y[p]+j));
            bluePixel(frame_buf, ix);
          }
        }
      }
    }
  }
}
