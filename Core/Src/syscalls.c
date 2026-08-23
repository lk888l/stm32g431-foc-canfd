#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "main.h"

/* No console, filesystem, process model or heap exists in the motor firmware. */
int _close(int file)
{
    (void)file;
    errno = ENOSYS;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    (void)file;
    if (status != NULL) {
        status->st_mode = S_IFCHR;
        return 0;
    }
    errno = EINVAL;
    return -1;
}

int _isatty(int file)
{
    (void)file;
    return 0;
}

off_t _lseek(int file, off_t offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    errno = ENOSYS;
    return (off_t)-1;
}

int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

int _write(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = ENOSYS;
    return -1;
}

void *_sbrk(ptrdiff_t increment)
{
    (void)increment;
    errno = ENOMEM;
    return (void *)-1;
}

int _getpid(void)
{
    return 1;
}

int _kill(int process_id, int signal)
{
    (void)process_id;
    (void)signal;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    Error_Handler();
    for (;;) {
    }
}
