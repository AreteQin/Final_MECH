
// computer vision functions

#include <iostream>
#include <fstream>

#include <cmath>
#include <Windows.h>

using namespace std;

// include this header file for basic image transfer functions
#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

void pause()
{
	while( !KEY(VK_SPACE) ) {}
	Sleep(300); // give some time to release space bar
}

int copy(image &a, image &b)
// copy an image
// a or b can be either RGB or greyscale image types
// this function can be used to convert between different image types
// a - input
// b - output (copy)
{
	i4byte i, j, size;
	ibyte *pa_GREY, *pb_GREY, R, G, B;

	// check for size compatibility
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in copy: sizes of a, b are not the same!";
		return 1;
	}

	// initialize pointers
	pa_GREY = a.pdata;
	pb_GREY = b.pdata;
	
	// number of pixels
	size = (i4byte)a.width * a.height;

	if( a.type==RGB_IMAGE && b.type==RGB_IMAGE ) {
		memcpy((void *)(b.pdata),(void *)(a.pdata),(size_t)(3*size));
	} else if (a.type==GREY_IMAGE && b.type==GREY_IMAGE) {
		memcpy((void *)(b.pdata),(void *)(a.pdata),(size_t)size);
	} else if (a.type==RGB_IMAGE && b.type==GREY_IMAGE) {
		// convert colour image to grey scale
		for(i=0,j=0;i<size;i++,j+=3) {
			B = pa_GREY[j];
			G = pa_GREY[j+1];
			R = pa_GREY[j+2];
			pb_GREY[i] = (ibyte)(0.299*R + 0.587*G + 0.114*B);
		}	
	} else if (a.type==GREY_IMAGE && b.type==RGB_IMAGE) {
		// convert grey scale image to colour
		for(i=0,j=0;i<size;i++,j+=3) {
			pb_GREY[j]   = pa_GREY[i];
			pb_GREY[j+1] = pa_GREY[i];
			pb_GREY[j+2] = pa_GREY[i];
		}
	} else {
		cout << "\nerror in copy: invalid types";
		return 1;
	}

	return 0;
}

int invert(image &a, image &b)
// invert an image
// a - input greyscale image
// b - output greyscale image
{
	i4byte size, i;
	ibyte *pa, *pb;

	// initialize pointers
	pa = a.pdata;
	pb = b.pdata;

	// check for compatibility of a, b
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in invert: sizes of a, b are not the same!";
		return 1;
	}

	if ( a.type != GREY_IMAGE || b.type != GREY_IMAGE ) {
		cout << "\nerror in invert: input types are not valid!";
		return 1;
	}

	// number of pixels
	size = (i4byte)a.width * a.height;

	// invert the image
	for(i=0;i<size;i++) pb[i] = 255 - pa[i];

	return 0;
}


int scale(image &a, image &b)
// scale the image intensity so that the pixels
// have the full range of intensity values (0-255)
// a, b - RGB or greyscale images
// a - input
// b - output
{
	i4byte size,i;
	ibyte *pa, *pb, min, max;

	// initialize pointers
	pa = a.pdata;
	pb = b.pdata;

	// check for compatibility of a, b
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in scale: sizes of a, b are not the same!";
		return 1;
	}
	if( a.type != b.type ) {
		cout << "\nerror in scale: types of a, b are not the same!";
		return 1;
	}

	min = 255;
	max = 0;

	if( a.type == RGB_IMAGE ) {
		size = (i4byte)a.width * a.height * 3; // number of bytes
		// find min and max pixel values
		for(i=0;i<size;i+=3) {
			if( pa[i] < min ) min = pa[i]; // B
			if( pa[i] > max ) max = pa[i]; // B
			if( pa[i+1] < min ) min = pa[i+1]; // G
			if( pa[i+1] > max ) max = pa[i+1]; // G
			if( pa[i+2] < min ) min = pa[i+2]; // R
			if( pa[i+2] > max ) max = pa[i+2]; // R
		}
		// scale the intensity
		for(i=0;i<size;i+=3) {
			pb[i]   = (ibyte)( 255.0*(pa[i]   - min)/(max - min) );
			pb[i+1] = (ibyte)( 255.0*(pa[i+1] - min)/(max - min) );
			pb[i+2] = (ibyte)( 255.0*(pa[i+2] - min)/(max - min) );
		}
	} else if( a.type == GREY_IMAGE ) {
		size = (i4byte)a.width * a.height; // number of bytes
		// find min and max pixel values
		for(i=0;i<size;i++) {
			if( pa[i] < min ) min = pa[i];
			if( pa[i] > max ) max = pa[i];
		}
		// scale the intensity
		for(i=0;i<size;i++) {
			pb[i] = (ibyte)( 255.0*(pa[i] - min)/(max - min) );
		}
	} else {
		cout << "\nerror in scale: type is not valid!";
		return 1;
	}
	
	return 0;
}


int convolution(image &a, image &b, int *k, double s)
// perform a 3 X 3 convolution filter with the following kernel
// k1 k2 k3
// k4 k5 k6
// k7 k8 k9
{
	i4byte size,i,j;
	ibyte *pa,*pa1,*pa2,*pa3,*pa4,*pa5,*pa6,*pa7,*pa8,*pa9,*pb;
	i2byte width,height;
	int x;

	// check for compatibility of a, b
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in convolution: sizes of a, b are not the same!";
		return 1;
	}

	if ( a.type != GREY_IMAGE || b.type != GREY_IMAGE ) {
		cout << "\nerror in convolution: input types are not valid!";
		return 1;
	}

	width  = a.width;
	height = a.height;

	// initialize pointers
	pa = a.pdata + width + 1;
	pb = b.pdata + width + 1;

	// set neighbourhood pointers
	// make sure they don't point outside of the images at the boundaries
	// when you use them
	// pa7 pa8 pa9
	// pa4 pa5 pa6
	// pa1 pa2 pa3
	
	pa1 = pa - width - 1;
	pa2 = pa - width;
	pa3 = pa - width + 1;
	pa4 = pa - 1;
	pa5 = pa;
	pa6 = pa + 1;
	pa7 = pa + width - 1;
	pa8 = pa + width;
	pa9 = pa + width + 1;

	// number of pixels to process
	size = (i4byte)a.width * a.height - 2*width - 2;

	// perform 3x3 convolution filter
	for(i=0;i<size;i++) {
		// set the center pixel equal to the weighted sum of the pixels
		x = k[1]*(*pa1) + k[2]*(*pa2) + k[3]*(*pa3) +
            k[4]*(*pa4) + k[5]*(*pa5) + k[6]*(*pa6) +
            k[7]*(*pa7) + k[8]*(*pa8) + k[9]*(*pa9);

		x = (int)(s*x); // apply the scale factor

		// check for out of range values
		if(x < 0)   x = 0;
		if(x > 255) x = 255;

		*pb = (ibyte)x;

		// increment pointers -> move the neighbourhood right
		pa1++; pa2++; pa3++; pa4++; pa5++;
		pa6++; pa7++; pa8++; pa9++; pb++;
	}

	// initialize pointers
	pa = a.pdata;
	pb = b.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	// process the image borders
	for(i=0;i<width;i++) {
		pb[i] = pb[i+width]; // bottom
		pb[size-i-1] = pb[size-i-1-width]; // top
	}
	for(i=0,j=0;i<height;i++,j+=width) {
		pb[j] = pb[j+1]; // left
		pb[size-j-1] = pb[size-j-2]; // right
	}
	
	// the corner calculations below are redundant
	// -- the copies above basically copy the corners indirectly
	pb[0] = pb[width+1]; // bottom left corner
	pb[width-1] = pb[width-1+width-1]; // bottom right corner
	pb[size-width] = pb[size-width-width+1]; // top left corner
	pb[size-1] = pb[size-1-width-1]; // top right corner

	return 0;
}


int lowpass_filter(image &a, image &b)
{
	int k[10];
	double s;

	// define convolution kernel
	// k1 k2 k3
	// k4 k5 k6
	// k7 k8 k9

	k[1] = 1;
	k[2] = 1;
	k[3] = 1;
	k[4] = 1;
	k[5] = 1;
	k[6] = 1;
	k[7] = 1;
	k[8] = 1;
	k[9] = 1;

	s = 1.0/9;

	convolution(a,b,k,s);

	return 0;
}


int highpass_filter(image &a, image &b)
{
	int k[10];
	double s;

	// define convolution kernel
	// k1 k2 k3
	// k4 k5 k6
	// k7 k8 k9

	k[1] = -1;
	k[2] = -1;
	k[3] = -1;
	k[4] = -1;
	k[5] = 9;
	k[6] = -1;
	k[7] = -1;
	k[8] = -1;
	k[9] = -1;

	s = 1.0;

	convolution(a,b,k,s);

	return 0;
}


int gaussian_filter(image &a, image &b)
{
	int k[10];
	double s;

	// define convolution kernel
	// k1 k2 k3
	// k4 k5 k6
	// k7 k8 k9

	k[1] = 1;
	k[2] = 2;
	k[3] = 1;
	k[4] = 2;
	k[5] = 4;
	k[6] = 2;
	k[7] = 1;
	k[8] = 2;
	k[9] = 1;

	s = 1.0/16;

	convolution(a,b,k,s);

	return 0;
}


int threshold(image &a, image &b, int tvalue)
// binary threshold operation
// a - greyscale image
// b - binary image
// tvalue - threshold value
{
	i4byte size, i;
	ibyte *pa, *pb;

	// initialize pointers
	pa = a.pdata;
	pb = b.pdata;

	// check for compatibility of a, b
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in threshold: sizes of a, b are not the same!";
		return 1;
	}

	if ( a.type != GREY_IMAGE || b.type != GREY_IMAGE ) {
		cout << "\nerror in threshold: input types are not valid!";
		return 1;
	}

	// number of bytes
	size = (i4byte)a.width * a.height;

	// threshold operation
	for(i=0;i<size;i++) {
		if( pa[i] < tvalue ) pb[i] = 0;
		else pb[i] = 255;
	}

	return 0;
}


int histogram(image &a, double *hist, int nhist, double &hmin, double &hmax)
{
	int j;
	i4byte i,size;
	ibyte *pa,min,max;
	double dn,x1,x2,r,dmin,dmax;

	// initialize pointer
	pa = a.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	// find min/max
	max = 0; min = 255;
	for(i=0;i<size;i++) {
		if(*pa > max) max = *pa;
		if(*pa < min) min = *pa;
		pa++;
	}
	
	cout << "\nmax = " << (int)max << " , min = " << (int)min;

	dmin = (double)min - 1.0e-6;
	dmax = (double)max + 1.0e-6;

	// return parameters
	hmin = (double)min;
	hmax = (double)max;

	dn = (dmax - dmin)/nhist;

	// initialize bins
	for(j=0;j<nhist;j++) hist[j] = 0.0;

	// initialize pointer
	pa = a.pdata;

	for(i=0;i<size;i++) {
		for(j=0;j<nhist;j++) {
			x1 = dmin + j*dn;
			x2 = x1 + dn;
			r = (double)(*pa);
			if( r >= x1 && r < x2 ) {
				hist[j]++;
				break;
			}
		}
		pa++;
	}
	
	return 0;
}


int dialate(image &a, image &b)
// four pixel neighbourhood dialation
{
	i4byte size,i,j;
	ibyte *pa,*pa1,*pa2,*pa3,*pa4,*pb,max;
	i2byte width,height;

	// check for compatibility of a, b
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in dialate: sizes of a, b are not the same!";
		return 1;
	}

	if ( a.type != GREY_IMAGE || b.type != GREY_IMAGE ) {
		cout << "\nerror in dialate: input types are not valid!";
		return 1;
	}

	width  = a.width;
	height = a.height;

	// initialize pointers
	pa = a.pdata + width;
	pb = b.pdata + width;

	// set neighbourhood pointers
	// make sure they don't point outside of the images at the boundaries
	// when you use them
	//		pa2
	//	pa3 pa	pa1
	//      pa4
	pa1 = pa + 1;
	pa2 = pa - width;
	pa3 = pa - 1;
	pa4 = pa + width;

	// number of pixels to process
	size = (i4byte)a.width * a.height - 2*width;

	// perform greyscale dialation filter
	for(i=0;i<size;i++) {
		max = 0;
		if( *pa  > max ) max = *pa;
		if( *pa1 > max ) max = *pa1;
		if( *pa2 > max ) max = *pa2;
		if( *pa3 > max ) max = *pa3;
		if( *pa4 > max ) max = *pa4;

		// set the center pixel equal to the maximum in the neighbourhood
		*pb = max;

		// increment pointers
		pa++; pa1++; pa2++; pa3++; pa4++; pb++;
	}

	// initialize pointer
	pb = b.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	// erase the image borders to prevent wrap around effects
	for(i=0;i<width;i++) {
		pb[i] = 0; // top
		pb[size-i-1] = 0; // bottom
	}
	for(i=0,j=0;i<height;i++,j+=width) {
		pb[j] = 0; // left
		pb[size-j-1] = 0; // right
	}

	return 0;
}


int dialate2(image &a, image &b)
// improved efficieny approx dialation
// also compatible with masked processing
{
	i4byte size,i,j;
	ibyte *pa,*pb1,*pb2,*pb3,*pb4,*pb;
	i2byte width,height;

	// check for compatibility of a, b
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in dialate: sizes of a, b are not the same!";
		return 1;
	}

	if ( a.type != GREY_IMAGE || b.type != GREY_IMAGE ) {
		cout << "\nerror in dialate: input types are not valid!";
		return 1;
	}

	// copy a into b first
	size = (i4byte)a.width * a.height;
	memcpy((void *)(b.pdata),(void *)(a.pdata),(size_t)size);

	width  = a.width;
	height = a.height;

	// initialize pointers
	pa = a.pdata + width;
	pb = b.pdata + width;

	// set neighbourhood pointers
	// make sure they don't point outside of the images at the boundaries
	// when you use them
	//		pb2
	//	pb3 pb	pb1
	//      pb4
	pb1 = pb + 1;
	pb2 = pb - width;
	pb3 = pb - 1;
	pb4 = pb + width;

	// number of pixels to process
	size = (i4byte)a.width * a.height - 2*width;

	// perform fast greyscale dialation filter
	for(i=0;i<size;i++) {
		if(*pa > 0) {
			*pb1 = *pa;
			*pb2 = *pa;
			*pb3 = *pa;
			*pb4 = *pa;
		}

		// increment pointers
		pa++; pb1++; pb2++; pb3++; pb4++; pb++;
	}

	// initialize pointer
	pb = b.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	// erase the image borders to prevent wrap around effects
	for(i=0;i<width;i++) {
		pb[i] = 0; // top
		pb[size-i-1] = 0; // bottom
	}
	for(i=0,j=0;i<height;i++,j+=width) {
		pb[j] = 0; // left
		pb[size-j-1] = 0; // right
	}

	return 0;
}


int erode(image &a, image &b)
// four pixel neighbourhood erosion
// morphological function
{
	i4byte size,i,j;
	ibyte *pa,*pa1,*pa2,*pa3,*pa4,*pb,min;
	i2byte width,height;

	// check for compatibility of a, b
	if( a.height != b.height || a.width != b.width ) {
		cout << "\nerror in erode: sizes of a, b are not the same!";
		return 1;
	}

	if ( a.type != GREY_IMAGE || b.type != GREY_IMAGE ) {
		cout << "\nerror in erode: input types are not valid!";
		return 1;
	}

	width  = a.width;
	height = a.height;

	// initialize pointers
	pa = a.pdata + width;
	pb = b.pdata + width;

	// set neighbourhood pointers
	// make sure they don't point outside of the images at the boundaries
	// when you use them
	//		pa2
	//	pa3 pa	pa1
	//      pa4
	pa1 = pa + 1;
	pa2 = pa - width;
	pa3 = pa - 1;
	pa4 = pa + width;

	// number of pixels to process
	size = (i4byte)a.width * a.height - 2*width;

	// perform greyscale dialation filter
	for(i=0;i<size;i++) {		
		if(*pa > 0) {
			min = 255;
			if( *pa  < min ) min = *pa;
			if( *pa1 < min ) min = *pa1;
			if( *pa2 < min ) min = *pa2;
			if( *pa3 < min ) min = *pa3;
			if( *pa4 < min ) min = *pa4;

			// set the center pixel equal to the maximum in the neighbourhood
			*pb = min;
		} else {
			*pb = 0;
		}
		
		// increment pointers
		pa++; pa1++; pa2++; pa3++; pa4++; pb++;
	}

	// initialize pointer
	pb = b.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	// erase the image borders
	for(i=0;i<width;i++) {
		pb[i] = 0; // top
		pb[size-i-1] = 0; // bottom
	}
	for(i=0,j=0;i<height;i++,j+=width) {
		pb[j] = 0; // left
		pb[size-j-1] = 0; // right
	}

	return 0;
}

int centroid(image &a, image &label, int nlabel, double &ic, double &jc)
// calculate the greyscale centroid of a labelled object
// a - GREY_IMAGE type
// label - LABEL_IMAGE type (pixel values 0-65535)
// nlabel - label number
// ic - i centroid coordinate (x direction / right)
// jc - j centroid coordinate (y direction / up)
//
// note: for image_transfer2.lib
// the origin (0,0) of the image is in the lower left corner, ip is 
// in the horizontal direction and jp is in the vertical direction.
{
	ibyte *pa;
	i2byte *pl;
	i4byte i,j,width,height;
	double mi,mj,m,rho;

	// check for compatibility of a, label
	if( a.height != label.height || a.width != label.width ) {
		cout << "\nerror in centroid: sizes of a, label are not the same!";
		return 1;
	}

	if ( a.type != GREY_IMAGE || label.type != LABEL_IMAGE ) {
		cout << "\nerror in centroid: input types are not valid!";
		return 1;
	}

	pa = a.pdata;
	pl = (i2byte *)label.pdata;

	// number of pixels
	width  = a.width;
	height = a.height;

	mi = mj = m = 0.0;

	for(j=0;j<height;j++) { // y-dir
		for(i=0;i<width;i++) { // x-dir
			if( pl[j*width+i] == nlabel ) {
				rho = pa[j*width+i];
				m  += rho; 
				// assume pixel has area of 1 so m = rho * A = rho
				mi += rho*i;
				mj += rho*j;
			}
		}
	}

	ic = mi/m;
	jc = mj/m;

	return 0;
}


int draw_point(image &a, int ip, int jp, int value)
// draw a point at pixel location (ip,jp)
// note: for image_transfer2.lib
// the origin (0,0) of the image is in the lower left corner, ip is 
// in the horizontal direction and jp is in the vertical direction.
{
	ibyte *pa;
	int i,j,w=2;

	// initialize pointer
	pa  = a.pdata;

	if ( a.type != GREY_IMAGE ) {
		cout << "\nerror in draw_point: input type not valid!";
		return 1;
	}

	// limit out of range values
	if( ip < w ) ip = w;
	if( ip > a.width-w-1 ) ip = a.width-w-1;
	if( jp < w ) jp = w;
	if( jp > a.height-w-1 ) jp = a.height-w-1;

	for(i=-w;i<=w;i++) {
		for(j=-w;j<=w;j++) {
			pa[a.width*(jp+j)+(ip+i)] = value;
		}
	}

	return 0;
}


int draw_point_rgb(image &rgb, int ip, int jp, int R, int G, int B)
{
	ibyte *p;
	int i,j,w=2,pixel;

	// initialize pointer
	p  = rgb.pdata;

	if ( rgb.type != RGB_IMAGE ) {
		cout << "\nerror in draw_point_RGB: input type not valid!\n";
		return 1;
	}

	// limit out of range (i,j) values
	// NOTE: this part is important to avoid wild pointers
	if( ip < w ) ip = w;
	if( ip > rgb.width-w-1 ) ip = rgb.width-w-1;
	if( jp < w ) jp = w;
	if( jp > rgb.height-w-1 ) jp = rgb.height-w-1;

	for(i=-w;i<=w;i++) {
		for(j=-w;j<=w;j++) {
			pixel = rgb.width*(jp+j)+(ip+i);
			p[3*pixel]   = B;
			p[3*pixel+1] = G;
			p[3*pixel+2] = R;
		}
	}

	return 0;
}


int label_image(image &a, image &b, int &nlabels)
// labels a binary image
// labels go from 1 to nlabels
// a - binary GREY_SCALE image type (pixel values must be 0 or 255)
// b - LABEL_IMAGE type (pixel values are 0 to 65535)
{ 
	long k,startpixel,endpixel;
	i2byte i,j,nlabel,m;
	ibyte *k1,*k2,*k3,*k4,*k5,*k6,*ip;
	i2byte *gp,*gp2,*L,*L2;

	i2byte *Pgrid; // label image pointer
	i2byte Nobjects;
	i2byte Hdim,Vdim;
	long Isize;

	Pgrid = (i2byte *)b.pdata;
	Hdim = a.width;
	Vdim = a.height;
	Isize = (long)Hdim*Vdim;

	L = (i2byte *)malloc( 65535*sizeof(i2byte) );
	if(L==NULL) {
		cout << "\nmemory allocation error in labelgrid::Label!";
	}
	for(k=0;k<65535;k++) L[k]=(i2byte)k;

	// remove 1st layer and 1st pixel of 2nd from binary image
	ip  = a.pdata;
	for(k=0;k<=Hdim;k++) *ip++ = 0;

	// set up pointers for 2 X 3 kernel

	// k1 k2 k3
	// k4 k5 k6
  
	gp = Pgrid + Hdim;
	k5 = a.pdata + Hdim; // start on 2nd row
	k1 = k5 - Hdim - 1;
	k2 = k5 - Hdim;
	k3 = k5 - Hdim + 1;
	k4 = k5 - 1;
	k6 = k5 + 1;
	startpixel = Hdim+1;
	endpixel = (long)Hdim*(Vdim-1)-1;
	nlabel = 0;

	// label 4-connected white objects
	for(k=startpixel;k<endpixel;k++) {

		k1++; k2++;	k3++; k4++; k5++; k6++; gp++;

		if(*k5) {

			if(*k2 && *k4) {
				i = *(gp-1); j = *(gp-Hdim); *gp = i;
				if(L[i]!=L[j]) {
					if (L[i]<L[j]) {
						L[L[j]] = L[i];
						L[j] = L[i];
					} else if(L[j]<L[i]) {
						L[L[i]] = L[j];
						L[i] = L[j];
					}
      			}
			} else if(*k2 && !*k4) {
      			*gp = *(gp - Hdim);
			} else if(!*k2 && *k4) {
				*gp = *(gp - 1);
			} else {
				if(*k3 && *k6) {
					*gp = *(gp - Hdim + 1);
				} else {
					*gp = ++nlabel;
					if(nlabel>65533) break;
				}
			}
		} 
		else *gp=0;
	}

	// find redundant labels
	for(k=nlabel;k>0;k--) {
		while(L[L[k]]!=L[k]) L[k]=L[L[k]];
 	}

	L2 = (i2byte *)malloc( (nlabel+1)*sizeof(i2byte) );

	if(L2==NULL) {
		cout << "\nmemory allocation error in labelgrid::Label!";
	}
	for(k=0;k<=nlabel;k++) L2[k]=0;

	m = 0;
	for(k=1;k<=nlabel;k++) {
		if( L2[L[k]]==0 ) L2[L[k]] = ++m;
	}

	Nobjects = m;

	// resolve label redundancy
 	gp = Pgrid + Hdim;
	for(k=startpixel;k<endpixel;k++) {
		gp++;
		*gp = L2[L[*gp]];
	}

	// erase borders
	gp  = Pgrid;
	gp2 = Pgrid+Hdim-1;
	for(k=0;k<Vdim;k++) {
		*gp = 0;
		*gp2 = 0;
		gp += Hdim;
		gp2 += Hdim;
	}

	gp  = Pgrid;
	gp2 = Pgrid+(long)Hdim*(Vdim-1);
	for(k=0;k<Hdim;k++) {
		*gp++ = 0;
		*gp2++ = 0;
	}

	free(L);
	free(L2);

	b.nlabels = Nobjects;	

	nlabels = Nobjects;

	return 0;
}
