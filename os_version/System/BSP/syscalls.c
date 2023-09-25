#include <sys/stat.h>
#include "bluetooth.h"

// Status of an open file. For consistency with other minimal implementations in these examples, all files are regarded as character special devices. The sys/stat.h header file required is distributed in the include subdirectory for this C library.
int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

// Query whether output stream is a terminal. For consistency with the other minimal implementations, which only support output to stdout, this minimal implementation is suggested:
int _isatty(int file)
{
  return 1;
}

#if defined(USE_SELF_SYSCALL)

// Close a file. Minimal implementation:
int _close(int file)
{
  return -1;
}

// Set position in a file. Minimal implementation:
int _lseek(int file, int ptr, int dir)
{
  return 0;
}

//Write to a file. libc subroutines will use this system routine for output to all files, including stdout—so if you need to generate any output, for example to a serial port for debugging, you should make your minimal write capable of doing this. The following minimal implementation is an incomplete example; it relies on a outbyte subroutine (not shown; typically, you must write this in assembler from examples provided by your hardware manufacturer) to actually perform the output.
int _write(int file, char *ptr, int len)
{
  int todo;

  for (todo = 0; todo < len; todo++) {
    // outbyte (*ptr++);
	Bluetooth_SendByte((*ptr++));
  }
  return len;
}

// Read from a file. Minimal implementation:
int _read(int file, char *ptr, int len)
{
  return 0;
}

// Increase program data space. As malloc and related functions depend on this, it is useful to have a working implementation. The following suffices for a standalone system; it exploits the symbol _end automatically defined by the GNU linker.

caddr_t _sbrk(int incr) {
  extern char _end;		/* Defined by the linker */
  static char *heap_end;
  char *prev_heap_end;
 
  if (heap_end == 0) {
    heap_end = &_end;
  }
  prev_heap_end = heap_end;
//   if (heap_end + incr > stack_ptr) {
    // write (1, "Heap and stack collision\n", 25);
    // abort ();
//   }

  heap_end += incr;
  return (caddr_t) prev_heap_end;
}

#endif
