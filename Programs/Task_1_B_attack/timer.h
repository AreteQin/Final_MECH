
// windows time measurement functions

// windows high resolution time function
// typically has 0.1 us resolution (i.e. intervals),
// but not very accurate over long periods
double high_resolution_time();

// returns the integer count of the high resolution clock
// 1 count typically = 0.1 us.
// note that the count will roll at around 4.3 billion
// (every 430 seconds / 7 minutes).
// this function is potentially useful for measuring small
// intervals in 0.1 us increments or seeding a random number
// generator with fairly random initial value (eg. use count % 100 
// to get a "one time" random number between 0 and 99 )
// note: that high_resolution_time() reads an additional
// variable to account for roll over of the count.
unsigned int high_resolution_count();

