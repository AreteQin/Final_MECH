
// function that uses the Windows function CreateFileMapping(...) to create/access
// a shared memory block between two or more programs.
// returns a char pointer to the shared memory block (returns NULL if it fails).
// name - a string indicating the name of the memory block
// n - the size of the memory block in bytes
char * shared_memory(char *name, int n);