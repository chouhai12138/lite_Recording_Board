/**
 * @file    syscalls.c
 * @brief   newlib 最小系统调用实现 + printf 重定向到 USART1
 *
 * 有了这个文件，-specs=nosys.specs 的弱符号会被覆盖，
 * printf / puts / putchar 的输出都会走到串口上。
 */
#include <errno.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <unistd.h>
#include "bsp_uart.h"

#undef errno
extern int errno;

extern int _end;        /* 链接脚本里的堆起点 */
extern int _estack;     /* 栈顶 */

/* ---------- 输出：printf -> USART1 ---------- */
int _write(int file, char *ptr, int len)
{
    (void)file;
    BSP_UART_Write((const uint8_t *)ptr, (uint16_t)len);
    return len;
}

/* ---------- 输入：从串口接收缓冲取数据（配合 scanf/getchar） ---------- */
int _read(int file, char *ptr, int len)
{
    int n = 0;

    (void)file;
    while (n < len) {
        uint8_t c;
        if (!BSP_UART_ReadByte(&c)) {
            break;
        }
        ptr[n++] = (char)c;
    }
    return n;
}

/* ---------- 堆 ---------- */
void *_sbrk(ptrdiff_t incr)
{
    static char *heap_end = NULL;
    char *prev_heap_end;

    if (heap_end == NULL) {
        heap_end = (char *)&_end;
    }

    prev_heap_end = heap_end;

    if (heap_end + incr > (char *)&_estack) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    while (1) {
    }
}
