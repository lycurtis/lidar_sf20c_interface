/**
 * @file    syscall.c
 * @brief   Minimal newlib system call stubs
 */

#include <sys/stat.h>
#include <errno.h>
#include "bsp_pinmap.h"
#include "usart.h"

int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        usart_write_byte(BSP_USART_DEBUG, (uint8_t)buf[i]);
    }
    return len;
}

int _read(int fd, char *buf, int len)  { (void)fd; (void)buf; (void)len; return 0; }
int _close(int fd)                     { (void)fd; return -1; }
int _lseek(int fd, int ptr, int dir)   { (void)fd; (void)ptr; (void)dir; return 0; }
int _fstat(int fd, struct stat *st)    { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)                    { (void)fd; return 1; }

void *_sbrk(int incr)
{
    extern char _end;
    static char *heap_end = 0;
    char *prev;
    if (heap_end == 0) heap_end = &_end;
    prev = heap_end;
    heap_end += incr;
    return (void *)prev;
}