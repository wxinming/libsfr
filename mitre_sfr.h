/*****************************************************************************
mitre_sfr.h

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
This file pertains to the wrapper code and is written by MITRE.
Code predominantly modified from IS 12233:2000 Annex A is in sfr_iso.c.
---------------------
*****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/* Revisions to sfr.h                                                        */
/*                                                                           */
/* V1.2 PIV spec check added as default, plus option to avoid the check      */
/*                                                           mal 10/06       */
/*                                                                           */
/*****************************************************************************/

#ifndef __MITRE_SFR_H__
#define __MITRE_SFR_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
	const char* get_version();

	void discrete_fourier_transform(int, double, const double*, int, double, double*);

	void apply_hamming_window(int, int, int, double*, int*);

	void locate_max_psf(int, const double*, int*);

	void calculate_derivative(int, double*, double*, double*, int);

	int bin_to_regular_xgrid(double, double*, double*, double*, int*, int, int);

	bool locate_centroids(const double*, double*, double*, int, int, double*);

	void linear_fitting(int, const double*, const double*, double*, double*, double*, double*, double*);

	bool check_slope(double, int*, int*, double, int);

	int sfr_proc(double** freq, double** sfr, int* len, double* farea, int size_x,
		int* nrows, double* slope, int* numcycles, int* pcnt2, double* off, double* r2,
		int version, int iterate);

#ifdef __cplusplus
}
#endif //!__cplusplus

#endif //!__MITRE_SFR_H__
