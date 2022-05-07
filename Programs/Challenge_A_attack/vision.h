
// computer vision functions header

// copy a into b
// a or b can be either RGB or greyscale image types
// can be used to convert between different image types
int copy(image &a, image &b);

// scale the image intensity so that the pixels
// have the full range of intensity values (0-255)
// a, b - RGB or greyscale images
int scale(image &a, image &b);

int invert(image &a, image &b);

int convolution(image &a, image &b, int *k, double s);

int lowpass_filter(image &a, image &b);

int highpass_filter(image &a, image &b);

int gaussian_filter(image &a, image &b);

int threshold(image &a, image &b, int tvalue);

int histogram(image &a, double *hist, int nhist, double &hmin, double &hmax);

int dialate(image &a, image &b);

int dialate2(image &a, image &b);

int erode(image &a, image &b);

// labels a binary image
// labels go from 1 to nlabels
// a - binary GREY_SCALE image type (pixel values must be 0 or 255)
// b - LABEL_IMAGE type (pixel values are 0 to 65535)
int label_image(image &a, image &b, int &nlabels);

// calculate the greyscale centroid of a labelled object
// a - GREY_IMAGE type
// label - LABEL_IMAGE type (pixel values 0-65535)
// nlabel - label number
// ic - i centroid coordinate (x-dir / right)
// jc - j centroid coordinate (y-dir / left)
//
// note: for image_transfer2.lib
// the origin (0,0) of the image is in the lower left corner, ip is 
// in the horizontal direction and jp is in the vertical direction.
int centroid(image &a, image &label, int nlabel, double &ic, double &jc);

// draw a point at pixel location (ip,jp) on a greyscale image
int draw_point(image &a, int ip, int jp, int value);

// draw a point on an RGB image
int draw_point_rgb(image &rgb, int ip, int jp, int R, int G, int B);

// wait for space key to continue
void pause();
