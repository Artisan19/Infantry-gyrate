/**
 * @file    circular_buffer.h
 * @brief   circular buffer
 * @author  Mark Xu<scmarxx@gmail.com>
 * @version 1.1
 * @date    2018-08-01
 */
#ifndef __CIRC_BUF__
#define __CIRC_BUF__

#ifdef __cplusplus
extern "C" {
#endif

#define MIN(a, b)       (((a) > (b)) ? (b) : (a))
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))

typedef struct CIRC_BUF
{
    unsigned int Size;
    unsigned int Header;
    unsigned int Tailer;
    unsigned char *Buffer;
} CircBuf_t;

unsigned long long IsPowerOf2 (unsigned long long Num);
unsigned long RoundUp_PowerOf2 (unsigned long Num);
unsigned long RoundDown_PowerOf2(unsigned long Num);

int          CircBuf_Init(CircBuf_t *CBuf, unsigned char *Buff, unsigned int Size);
int          CircBuf_Alloc (CircBuf_t *CBuf, unsigned int Size);
void         CircBuf_Free (CircBuf_t *CBuf);
unsigned int CircBuf_Push (CircBuf_t *CBuf, unsigned char *data, unsigned int LenToPush);
unsigned int CircBuf_Pop  (CircBuf_t *CBuf, unsigned char *data, unsigned int LenToPop);
unsigned int CircBuf_PopOneChar (CircBuf_t *CBuf, unsigned char *data);
unsigned char CircBuf_At(CircBuf_t *CBuf, unsigned int offset);
void         CircBuf_Drop(CircBuf_t *CBuf, unsigned int LenToDrop);
unsigned int CircBuf_GetAvalaibleSize (CircBuf_t *CBuf);
unsigned int CircBuf_GetUsedSize (CircBuf_t *CBuf);
unsigned int CircBuf_IsEmpty (CircBuf_t *CBuf);
unsigned int CircBuf_IsFull (CircBuf_t *CBuf);

#ifdef __cplusplus
}
#endif

#endif

