
// function that uses the Windows function CreateFileMapping(...) to create/access
// a shared memory block between two or more programs

#include <iostream>
#include <windows.h>
#include "shared_memory.h"

using namespace std;

char * shared_memory(char *name, int n)
{
	char *p; // pointer to the shared memory block
	HANDLE hFileMapping; // handle to file mapped memory

	// create a file map
	hFileMapping = CreateFileMapping((HANDLE)0xffffffff, // segment independant of disk file
									NULL,            // no security attributes
									PAGE_READWRITE,  // full access to memory
									(DWORD)0,        // less than 4Gb in size
									(DWORD)n,		 // size of buffer
									name);			 // name of the map

	if (hFileMapping != 0)
	{
		p = (char *)MapViewOfFile(hFileMapping,FILE_MAP_ALL_ACCESS, 0, 0, 0);
	}

	if( !p ) {
		CloseHandle(hFileMapping);
		cout << "\nshared memory file mapping error\n";
	}
			
	return p;
}

