
// image types
// RGB_IMAGE   is 3 bytes per pixel
// GREY_IMAGE  is 1 bytes per pixel
// LABEL_IMAGE is 2 bytes per pixel
#define RGB_IMAGE 1
#define GREY_IMAGE 2
#define LABEL_IMAGE 3 

// define a variable type that holds a single byte of data
typedef unsigned char ibyte;
typedef unsigned short int i2byte;
typedef unsigned long int i4byte;

// define a structure that stores an image
typedef struct {
	int type;   // image type
	i2byte height; // image height
	i2byte width;  // image width
	ibyte *pdata; // pointer to image data
	i2byte nlabels;  // number of labels (used for LABEL_IMAGE type)
} image;

// size of shared memory block
// 170 MB (10,000 x 5,650 pixel -- RGB image)
const DWORD SMAX = 170000000; 

// NODE_A = image transfer lib program
// NODE_B = image_view program
const i2byte NODE_A = 1, NODE_B = 2;

///////

int activate_vision();

int activate_camera(int cam_number, int height, int width);

int acquire_image(image &a, int cam_number);

int deactivate_vision();

int stop_camera(int cam_number);

int start_camera(int cam_number);

///////

int view_rgb_image(image &a);

int save_rgb_image(char *file_name, image &a);

int load_rgb_image(char *file_name, image &a);

int set_rgb_image(char *file_name, image &a);

int allocate_image(image &a);

int free_image(image &a);

// video I/O functions ///////

// restrictions:
// 1) only one input and output file can be opened at a time
// 2) an input video file must currently be opened in order to open and write 
// to an output file

int open_video_input(char *file_name, double &t_duration, int &width, int &height);

int close_video_input();

int open_video_output(char *file_name);

int close_video_output();

int read_video_input(image &a, double &t_sample);

int position_video_input(double t_position);

int write_video_output(image &a, double t_sample);
