/*****************************************************************************
				Notice
								------

This software is a modification of the "SFR Measurement Algorithm C-code",
copyright PIMA 1998, appearing in Annex A of ISO standard 12233-2000,
"Photography - Electronic Still Picture Cameras - Resolution Measurement".
Permission to modify this software, and use and distribute the modified
software, was granted by I3A (the successor organization of PIMA) to the
MITRE Corporation in 2006.

This MITRE Corporation-modified SFR software was produced for the U.S.
Government under Contract numbers J-FBI-12-128 and W15P7T-05-C-F600 and is
subject to the Rights in Data-General clause FAR 52.227-l4, Alt. IV (DEC 2007).

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

-Redistribution of source code must retain the above copyright notice,
 this list of conditions, and the following disclaimer.

-Redistribution in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

---------------------
  Code modifications by Margaret A. Lepley (MITRE), mlepley@mitre.org
---------------------

*****************************************************************************/
/*****************************************************************************/
/*                                                                           */
/* Revisions to sfr_iso.c                                                    */
/*                                                                           */
/* V1.1 Added user_angle flag to avoid centroid computation                  */
/*                                                           mal 9/06        */
/*                                                                           */
/* V1.3 Removed low contrast check (being performed in wrapper code)         */
/*      Processing can continue even for low/high angles.  Warning printed   */
/*      if more lines of data needed for accurate processing.                */
/*      If less than one phase rotation included in ROI, continue processing */
/*      with all the lines present.                                          */
/*                                                           mal 11/06       */
/*                                                                           */
/*****************************************************************************/

#include "mitre_sfr.h"

/*****************************************************************************/

//#define MITRE_PI  3.14159265358979323846
#define MITRE_PI 3.1415926535897932384626433832795
#define ALPHA 4.0
#define DER3 4
#define PEAK 2
#define ROUND 1

//#ifdef NDEBUG
//#define PRINT(fmt,...)
//#else
//#define PRINT(fmt,...) printf(fmt, ##__VA_ARGS__)
//#endif
#define PRINT(fmt, ...)

static double sqrarg;
#define SQR(a) ((sqrarg=(a)) == 0.0 ? 0.0: sqrarg*sqrarg)

/*****************************************************************************/
/* Data passed to this function is assumed to be radiometrically corrected,  */
/* and oriented vertically, with black on left, white on right. The black to */
/* white orientation doesn't appear to be terribly important in this code.   */
/* Radiometric correct data (or at least 0-1 scaled) is important for the    */
/* contrast check.                                                           */
/*     Parameters:
		  Input:  farea  = radiometric image data in ROI
				  size_x = number of columns in ROI
		  nrows  = number of rows in ROI

	  Output: freq   = new array of relative spatial frequencies
			  sfr    = new array of computed SFR at each freq
		  len    = length of freq & sfr arrays
		  slope  = estimated slope value
		  numcycles = number of full periods included in SFR
		  pcnt2  = location of edge mid-point in oversample space
		  off    = shift to center edge in original rows
		  version = 0 = default ([-1 1] deriv, no rounding, no peak)
					1 = add rounding
				2 = add peak
				4 = [-1 0 1] derivative, rounding, no peak
		  iterate = 0 do just a single run
					1 means more runs after this, don't change farea
					   and let numcycles go as low as 1.0
		  user_angle = flag to indicate if the line fit has
					   been precomputed

				  farea  = size_x*4 of ESF and len*2 of LSF values
							 (if iterate = 0)
*/
/*****************************************************************************/

int sfr_proc(double** freq, double** sfr, int* len,
	double* farea, int size_x, int* nrows,
	double* slope, int* numcycles, int* pcnt2,
	double* off, double* r2,
	int version, int iterate)
{
	/* Verify input selection dimensions are EVEN */
	if (size_x % 2 != 0)
	{
		PRINT("ROI width is not even.  Does this really matter???\n");
		return 1;
	}

	int i = 0, j = 0, size_y = *nrows;

	//每行与中心行的距离
	double* distance = (double*)malloc(size_y * sizeof(double));

	//每行质心与中心行质心的距离
	double* shifts = (double*)malloc(size_y * sizeof(double));

	double avar = 0, bvar = 0, offset1 = 0, offset2 = 0;
	if (!locate_centroids(farea, distance, shifts, size_x, size_y, &offset1))
	{
		free(distance);
		free(shifts);
		return 2;
	}

	/* Calculate the best fit line to the centroids */
	linear_fitting(size_y, distance, shifts, slope, &offset2, r2, &avar, &bvar);
	free(distance);

	if (version)
	{
		PRINT("\nLinear Fit:  R2 = %.3f SE_intercept = %.2f  SE_angle = %.3f\n",
			*r2, avar, atan(bvar) * (double)(180.0 / MITRE_PI));
	}

	/* At least this many cycles required. */
	/* For special iterative versions of the code, it can go lower */
	double cycle_limit = iterate ? 1.0 : 5.0;

	/* Check slope is OK, and set size_y to be full multiple of cycles */
	if (!check_slope(*slope, &size_y, numcycles, cycle_limit, 1))
	{
		free(shifts);
		/* Slopes are bad. But send back enough data, so a diagnostic image has a chance. */
		*pcnt2 = 2 * size_x;  /* Ignore derivative peak */
		return 3;
	}

	/* Start image at new location, so that same row is center */
	int center_row = *nrows / 2;
	int start_row = center_row - size_y / 2;
	double* farea_old = farea;
	farea = farea + start_row * size_x;

	/* On center row how much shift to get edge centered in row. */
	/* offset = 0.;  Original code effectively used this (no centering)*/
	double offset = offset1 + 0.5 + offset2 - (double)size_x / 2.0;

	*off = offset;
	if (version & ROUND || version & DER3)
	{
		offset += 0.125;
	}

	/* 
	  reference the temp and shifts values to the new y center
	  Instead of using the values in shifts, synthesize new ones based on
	  the best fit line.
	*/
	int col = size_y / 2;
	for (i = 0; i < size_y; ++i)
	{
		shifts[i] = (*slope) * (double)(i - col) + offset;
	}

	/* Don't normalize the data.  It gets normalized during dft process */
	/* To normalize here, set dt = min and dt1 = max of farea data      */
	double dt = 0.0, dt1 = 1.0;

	double* edgex = (double*)malloc(size_y * size_x * sizeof(double));
	double* signal = (double*)malloc(size_y * size_x * sizeof(double));
	/* Calculate a long paired list of x values and signal values */
	int pcnt = 0;
	for (j = 0; j < size_y; ++j)
	{
		for (i = 0; i < size_x; ++i)
		{
			edgex[pcnt] = (double)i - shifts[j];
			signal[pcnt] = (farea[j * size_x + i] - dt) / (dt1 - dt);
			++pcnt;
		}
	}

	free(shifts);

	/* Allocate more memory */
	int bin_len = (int)(ALPHA * size_x);
	double* AveEdge = (double*)malloc(bin_len * sizeof(double));
	double* AveTmp = (double*)malloc(bin_len * sizeof(double));
	int* counts = (long*)malloc(bin_len * sizeof(long));
	/* Project ESF(Edge Spread Function,边缘扩展函数) data into supersampled bins */
	int nzero = bin_to_regular_xgrid(ALPHA, edgex, signal, AveEdge, counts, size_x, size_y);
	free(counts);
	free(edgex);
	free(signal);

	double centroid = 0.0;
	/* Compute LSF(LineSpread Function,线扩展函数) from ESF.  Not yet centered or windowed. */
	calculate_derivative(bin_len, AveTmp, AveEdge, &centroid, version & DER3);

	if (iterate == 0)
	{
		/* Copy ESF to output area */
		for (i = 0; i < bin_len; ++i)
		{
			farea_old[i] = AveTmp[i];
		}
	}

	/* Find the peak/center of LSF */
	locate_max_psf(bin_len, AveEdge, &pcnt);


	if (version)
	{
		PRINT("Off center distance (1/4 pixel units): Peak %ld  Centroid %.2f\n",
			pcnt - bin_len / 2, centroid - bin_len / 2);
	}

	if ((version & PEAK) == 0)
	{
		pcnt = bin_len / 2;  /* Ignore derivative peak */
	}
	else
	{
		PRINT("Shifting peak to center\n");
	}

	/* 
	Here the array length is shortened to ww_in_pixels*ALPHA,
	and the LSF peak is centered and Hamming windowed. 
	*/
	apply_hamming_window((int)ALPHA, bin_len, size_x, AveEdge, &pcnt);

	/* From now on this is the length used. */
	*len = bin_len / 2;

	if (iterate == 0)
	{
		/* Copy LSF_w to output area */
		for (i = 0; i < bin_len; i++)
		{
			farea_old[size_x * (int)ALPHA + i] = AveEdge[i];
		}
	}

	double tmp = 1.0;
	double tmp2 = 1.0 / (double)bin_len;

	/* Now perform the DFT on AveEdge */
	/* discrete_fourier_transform ( nx, dx, lsf(x), nf, df, sfr(f) ) */
	discrete_fourier_transform(bin_len, tmp, AveEdge, *len, tmp2, AveTmp);

	if (*freq == NULL)
	{
		*freq = (double*)malloc((*len) * sizeof(double));
	}

	if (*sfr == NULL)
	{
		*sfr = (double*)malloc((*len) * sizeof(double));
	}

	for (i = 0; i < (*len); i++)
	{
		(*freq)[i] = (double)i / (double)size_x;
		(*sfr)[i] = AveTmp[i] / AveTmp[0];
	}

	/* Free */
	free(AveEdge);
	free(AveTmp);

	*nrows = size_y;
	*pcnt2 = pcnt;

	return 0;
}

const char* get_version()
{
	return "1.4.2";
}

/*****************************************************************************/
bool locate_centroids(const double* farea, double* distances, double* shifts, int size_x, int size_y, double* offset)
{
	/* 
	* Compute the first difference on each line. 
	* Interpolate to find the centroid of the first derivatives. 
	*/

	/*
	* 计算原理:
	* dt1 = Σ(farea[i + 1] - farea[i])
	* dt = Σ[(farea[i + 1] - farea[i]) * i]
	* 矩心对应位置:
	* shift[i] = dt/dt1
	* 
	* temp是temp到质心的距离数[比如图像24*22,则temp[0]=(22/2)-22,temp[n-1]=22/2-1]
	* 此处shifts为每行的质心位置
	*/
	for (int j = 0; j < size_y; ++j)
	{
		double dt = 0, dt1 = 0;
		for (int i = 0; i < size_x - 1; ++i)
		{
			double temp = farea[j * size_x + (i + 1)] - farea[j * size_x + i];
			dt += temp * (double)i;
			dt1 += temp;
		}
		shifts[j] = dt / dt1;
	}

	/*
		check again to be sure we aren't too close to an edge on the corners.
		If the black to white transition is closer than 2 pixels from either
		side of the data box, return an error of 5; the calling program will
		display an error message (the same one as if there were not a difference
		between the left and right sides of the box )
	*/
	if (shifts[size_y - 1] < 2 || size_x - shifts[size_y - 1] < 2)
	{
		PRINT("** WARNING: Edge comes too close to the ROI corners.\n");
		return false;
	}

	if (shifts[0] < 2 || size_x - shifts[0] < 2)
	{
		PRINT("** WARNING: Edge comes too close to the ROI corners.\n");
		return false;
	}

	/* Reference rows to the vertical centre of the data box */
	int half_y_size = size_y / 2;
	
	/* 获取中心矩心 */
	double cc = shifts[half_y_size];
	for (int i = 0; i < size_y; ++i)
	{
		distances[i] = (double)i - (double)half_y_size;
		shifts[i] -= cc;
		//此处的shifts是与中心质心的偏移距离
	}
	*offset = cc;
	return true;
}

/***************************************************************************/
void linear_fitting(int ndata, const double* x, const double* y, double* b, double* a, double* r2, double* avar, double* bvar)
{
	int i = 0;
	double t = 0, sxoss = 0, syoss = 0, sx = 0.0, sy = 0.0, st2 = 0.0;
	double ss = 0, sst = 0, sigdat = 0, chi2 = 0, siga = 0, sigb = 0;

	*b = 0.0;
	for (i = 0; i < ndata; ++i)
	{
		sx += x[i];
		sy += y[i];
	}
	ss = (double)ndata;
	sxoss = sx / ss;
	syoss = sy / ss;
	for (i = 0; i < ndata; ++i)
	{
		t = x[i] - sxoss;
		st2 += t * t;
		*b += t * y[i];
	}
	*b /= st2;         /* slope  */
	*a = (sy - sx * (*b)) / ss; /* intercept */
	siga = sqrt((1.0 + sx * sx / (ss * st2)) / ss);
	sigb = sqrt(1.0 / st2);
	chi2 = 0.0;
	sst = 0.0;
	for (i = 0; i < ndata; i++)
	{
		chi2 += SQR(y[i] - (*a) - (*b) * x[i]);
		sst += SQR(y[i] - syoss);
	}
	sigdat = sqrt(chi2 / (ndata - 2));
	siga *= sigdat;
	sigb *= sigdat;
	*r2 = 1.0 - chi2 / sst;
	*avar = siga;
	*bvar = sigb;
	return;
}

/****************************************************************************/
bool check_slope(double slope, int* size_y, int* numcycles, double mincyc, int errflag)
{
	double absslope = fabs(slope);

	if (*numcycles <= 0)
	{
		(*numcycles) = (int)((*size_y) * absslope);
	}

	/* If the slope is too small not enough rows for mincy (typically 5)
	   full periods, then alert the user. */
	if (absslope < mincyc / (double)(*size_y))
	{
		if (errflag == 1)
		{
			PRINT("WARNING: Edge angle (%f) is so shallow it needs\n", atan(slope) * 180 / MITRE_PI);
			PRINT("  %d lines of data (%.1f cycles) for accurate results\n", (int)ceil(mincyc / absslope), mincyc);
			return true;
		}
		else
		{
			return false;
		}
	}

	if (absslope > (double)(1.0 / 4.0))
	{
		int rows_per_col;
		double bestcycle, x;

		if (absslope > (double)(5.0 / 4.0))
		{
			PRINT("ERROR: Edge angle (%f) is too large\n", atan(slope) * 180 / MITRE_PI);
			return false;
		}

		rows_per_col = (int)floor(1 / absslope + 0.5);
		x = fabs(1 / (double)rows_per_col - absslope);
		bestcycle = 4 * rows_per_col * x * ceil(1.0 / x / (double)rows_per_col / (double)rows_per_col - 1.0);
		if ((int)ceil(mincyc * bestcycle) > *size_y)
		{
			if (errflag == 1)
			{
				PRINT("WARNING: Edge angle (%f) will reduce SFR accuracy\n", atan(slope) * 180 / MITRE_PI);
				PRINT("   if %g * %f = %d lines of data are not in ROI\n\n",
					mincyc, bestcycle, (int)ceil(mincyc * bestcycle));
				return true;
			}
			else
			{
				return false;
			}
		}
	}

	/* 
	   Figure out how many lines to use for size_y: new window will start at
	   top and go down that number of lines < size_y such that an integer
	   number of x-transitions are made by the edge; for example, if we had a
	   slope of 10 (the edge goes down 10 lines before jumping over one pixel
	   horizontally), and size_y = 35, the new size_y is going to be 30 (an
	   integer multiple of 10, less than 35). 
	*/

	if (((*numcycles) / absslope) <= *size_y)
	{
		*size_y = (int)((*numcycles) / absslope);
	}

	return true;
}

/*****************************************************************************/
/* 
Notes: this part gets averages and puts them in a number of bins, equal to
size_x times alpha.  Next a long check is done in case one bin gets no values
put into it: if this is the case, it will keep checking previous bins until it
finds one with non-zero counts and will use that value as its current bin
average. If the first bin has zero counts the program checks bins in the
forward rather than reverse direction. If, in any case, the end of the array
of bins is reached before finding a non-zero count, the program starts
checking in the opposite direction. A bin with zero counts is not allowed,
since each bin will be divided by counts at the end.
*/

int bin_to_regular_xgrid(double alpha, double* edgex, double* signal,
	double* AveEdge, int* counts, int size_x, int size_y)
{
	int i, j, k, bin_number, bin_len;

	bin_len = size_x * (int)alpha;

	for (i = 0; i < bin_len; ++i)
	{
		AveEdge[i] = 0;
		counts[i] = 0;
	}

	int total = size_x * size_y;
	for (i = 0; i < total; ++i)
	{
		bin_number = (long)floor(alpha * edgex[i]);
		if (bin_number >= 0)
		{
			if (bin_number <= (bin_len - 1))
			{
				AveEdge[bin_number] = AveEdge[bin_number] + signal[i];
				counts[bin_number] = counts[bin_number] + 1;
			}
		}
	}

	int nzeros = 0;
	for (i = 0; i < bin_len; ++i)
	{
		j = 0;
		k = 1;
		if (counts[i] == 0)
		{
			nzeros++;
			if (i == 0)
			{
				while (!j)
				{
					if (counts[i + k] != 0)
					{
						AveEdge[i] = AveEdge[i + k] / (double)counts[i + k];
						j = 1;
					}
					else k++;
				}
			}
			else
			{
				while (!j && ((i - k) >= 0))
				{
					if (counts[i - k] != 0)
					{
						/* Don't divide by counts since it already happened in previous iteration */
						AveEdge[i] = AveEdge[i - k];
						j = 1;
					}
					else
					{
						k++;
					}
				}

				if ((i - k) < 0)
				{
					k = 1;
					while (!j)
					{
						if (counts[i + k] != 0)
						{
							AveEdge[i] = AveEdge[i + k] / (double)counts[i + k];
							j = 1;
						}
						else
						{
							k++;
						}
					}
				}
			}
		}
		else
		{
			AveEdge[i] /= (double)counts[i];
		}
	}

	if (nzeros > 0)
	{
		PRINT("\nWARNING: %d Zero counts found during projection binning.\n", nzeros);
		PRINT("The edge angle may be large, or you may need more lines of data.\n\n");
	}
	return nzeros;
}

/*****************************************************************************/
/* This has been modified from Annex A, to more closely match Annex D and
   reduce finite difference errors.  Now allows either [-1 1] derivative
   (when separation = 0) or [-1/2 0 1/2] derivative (when separation=1)

   Inputs:   len          length of ESF array
			 AveEdge      array of ESF values
			 separation   type of derivative
						0 = [-1 1]
				1 = [-1/2 0 1/2]

   Outputs:  AveTmp       array of original ESF values
			 AveEdge      array of derivative (LSF) values
			 centroid     centroid of the derivative
*/
void calculate_derivative(int len, double* AveTmp, double* AveEdge, double* centroid, int separation)
{
	int i = 0;
	double dt = 0.0, dt1 = 0.0;

	for (i = 0; i < len; ++i)
	{
		AveTmp[i] = AveEdge[i];
	}

	for (i = 1; i < len - separation; ++i)
	{
		/* Not wasting time with division by 2 since constant factors don't change SFR computation */
		AveEdge[i] = (AveTmp[i + separation] - AveTmp[i - 1]);
		if (separation == 1)
		{
			AveEdge[i] /= 2.0;
		}
		dt += AveEdge[i] * (double)i;
		dt1 += AveEdge[i];
	}

	*centroid = dt / dt1;

	AveEdge[0] = AveEdge[1];
	if (separation == 1)
	{
		AveEdge[len - 1] = AveEdge[len - 2];
	}
	return;
}

/*****************************************************************************/
void locate_max_psf(int len, const double* AveEdge, int* pcnt2)
{
	double dt = 0.0, dt_new = 0.0;
	int i = 0, left = -1, right = -1;

	/* find maximim value in Point Spread Function array */
	for (i = 0; i < len; ++i)
	{
		dt_new = fabs(AveEdge[i]);
		if (dt_new > dt)
		{
			(*pcnt2) = (long)i;
			dt = dt_new;
		}
	}

	/* find leftmost and rightmost occurrence of maximum */
	for (i = 0; i < len; ++i)
	{
		dt_new = fabs(AveEdge[i]);
		if (dt_new == dt)
		{
			if (left < 0)
			{
				left = i;
			}
			right = i;
		}
	}
	/* find centre of maxima */
	(*pcnt2) = (right + left) / 2;
	return;
}

/*****************************************************************************/
void apply_hamming_window(int alpha, int oldlen, int newxwidth, double* AveEdge, int* pcnt2)
{
	int i, j, k;

	/* Shift the AvgEdge[i] vector to centre the lsf in the transform window */
	int edge_offset = (*pcnt2) - (oldlen / 2);
	if (edge_offset != 0)
	{
		if (edge_offset < 0)
		{
			for (i = oldlen - 1; i > -edge_offset - 1; --i)
			{
				AveEdge[i] = (AveEdge[i + edge_offset]);
			}

			for (i = 0; i < -edge_offset; ++i)
			{
				AveEdge[i] = 0.00; /* last operation */
			}
		}
		else
		{
			for (i = 0; i < oldlen - edge_offset; ++i)
			{
				AveEdge[i] = (AveEdge[i + edge_offset]);
			}

			for (i = oldlen - edge_offset; i < oldlen; ++i)
			{
				AveEdge[i] = 0.00;
			}
		}
	}
	/* Multiply the LSF data by a Hamming window of width NEWXWIDTH*alpha */
	int begin = (oldlen / 2) - (newxwidth * alpha / 2);
	if (begin < 0)
	{
		begin = 0;
	}

	int end = (oldlen / 2) + (newxwidth * alpha / 2);
	if (end > oldlen)
	{
		end = oldlen;
	}

	for (i = 0; i < begin; ++i)
	{
		AveEdge[i] = 0.0;
	}

	for (i = end; i < oldlen; ++i)
	{
		AveEdge[i] = 0.0;
	}

	for (i = begin, j = -newxwidth * alpha / 2; i < end; ++i, ++j)
	{
		double sfrc = 0.54 + 0.46 * cos((MITRE_PI * (double)j) / (newxwidth * alpha / 2));
		AveEdge[i] *= sfrc;
	}

	if (begin != 0) /* Shift LSF to begin at index 0 (rather than begin) */
	{
		for (k = 0, i = begin; k < newxwidth * alpha; ++i, ++k)
		{
			AveEdge[k] = AveEdge[i];
		}
	}
	return;
}

/*****************************************************************************/
/* This is the DFT magnitude code                                            */
void discrete_fourier_transform(int number, double dx, const double* lsf, int ns, double ds, double* sfr)
{
	double a = 0, b = 0;
	double twopi = 2.0 * MITRE_PI;
	int j = 0, i = 0;
	for (j = 0; j < ns; ++j)
	{
		double g = twopi * dx * ds * (double)j;
		for (i = 0, a = 0, b = 0; i < number; ++i)
		{
			a += lsf[i] * cos(g * (double)i);
			b += lsf[i] * sin(g * (double)i);
		}
		sfr[j] = sqrt(a * a + b * b);
	}
	return;
}

