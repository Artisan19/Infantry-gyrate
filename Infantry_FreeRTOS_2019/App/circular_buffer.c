/**
 * @file    circular_buffer.c
 * @brief   circular buffer
 * @author  Mark Xu<scmarxx@gmail.com>
 * @version 1.2
 * @date    2018-08-01
 */
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "circular_buffer.h"

/**
 * @brief     Check if Num is power of 2
 *
 * @param[in] Num   the number to check
 *
 * @return          1 if Num is power of 2
 */
unsigned long long IsPowerOf2(unsigned long long Num)
{
    return (Num > 0 && !(Num & (Num - 1)));
}

/**
 * @brief     calculate the minimum number that round up to the next power of 2
 *
 * @param[in] Num   the number to calculate
 *
 * @return          the number that round up to the next power of 2 (0x100 if Num is 0xf0, 0x81, 0xa3 ... )
 */
unsigned long RoundUp_PowerOf2(unsigned long Num)
{
    unsigned long result = 1;

    if (IsPowerOf2(Num) || Num == 0)
        return Num;
    else if (Num > LONG_MAX)
        return (LONG_MAX ^ ULONG_MAX);   // WARN: if Num biger than (LONG_MAX+1) then result will equals to (LONG_MAX+1)

    while (Num)
    {
        Num >>= 1;
        result <<= 1;
    }

    return result;
}

/**
 * @brief     calculate the minimum number that round down to the next power of 2
 *
 * @param[] Num the number to check
 *
 * @return    the number that round up to the last power of 2 (4 if Num is 5,6,7, 8 if Num is 9,10,11 ... )
 */
unsigned long RoundDown_PowerOf2(unsigned long Num)
{
    unsigned long result = 1;

    if (IsPowerOf2(Num) || Num == 0)
        return Num;
    else if (Num > LONG_MAX)
        return (LONG_MAX ^ ULONG_MAX);   // WARN: if Num biger than (LONG_MAX+1) then result will equals to (LONG_MAX+1)

    while (Num)
    {
        Num >>= 1;
        result <<= 1;
    }

    return result >> 1;
}

/**
 * @brief     Init the Circular buffer with a array
 *
 * @param[in] CBuf      The circular buffer to initial
 * @param[in] Buff      the buffer for circular buffer to store data
 * @param[in] Size      the size of buffer
 *
 * @return      the Round Down(Power Of 2) size that the circular  buffer to be used
 */
int CircBuf_Init(CircBuf_t *CBuf, unsigned char *Buff, unsigned int Size)
{
    CBuf->Buffer = Buff;

    if(!IsPowerOf2(Size))
    {
        if (Size > INT_MAX)
            Size = (INT_MAX ^ UINT_MAX);
        else
            Size = (int) RoundDown_PowerOf2(Size);
    }

    CBuf->Size = Size;
    CBuf->Tailer = 0;
    CBuf->Header = 0;

    return Size;
}

/**
 * @brief     Circular Buffer initialization
 *
 * @param[in] CBuf      the circular buffer to initialization
 * @param[in] Size      size of the circular buffer
 *
 * @return    1 if memery allocation success
 *            0 if fail
 */
int CircBuf_Alloc(CircBuf_t *CBuf, unsigned int Size)
{
    int result = 0;

    if(!IsPowerOf2(Size))
    {
        if(Size > INT_MAX)
            Size = (INT_MAX ^ UINT_MAX);
        else
            Size = (int)RoundUp_PowerOf2(Size);
    }
    CBuf->Buffer = (unsigned char *) calloc(Size, sizeof(char));    // Buffer will set to 0

    CBuf->Tailer = 0;
    CBuf->Header = 0;

    if(CBuf->Buffer != NULL)
    {
        CBuf->Size = Size;
        result = 1;
    }

    return result;
}

/**
 * @brief     delete circular buffer and release the memery
 *
 * @param[in] CBuf  the circular buffer to delete
 */
void CircBuf_Free(CircBuf_t *CBuf)
{
    free(CBuf->Buffer);
    CBuf = NULL;
}

/**
 * @brief     put data into the circular buffer
 *
 * @param[in] CBuf      the circular buffer that will store the data
 * @param[in] data      the data to store into circular buffer
 * @param[in] LenToPush  the length of data to store into circular buffer
 *
 * @return      the actual size stored into circular buffer
 */
unsigned int CircBuf_Push(CircBuf_t *CBuf, unsigned char *data, unsigned int LenToPush)
{
    unsigned int len;

    LenToPush = MIN(LenToPush, (CBuf->Size - (CBuf->Header - CBuf->Tailer)));

    len = MIN(LenToPush, CBuf->Size - (CBuf->Header & (CBuf->Size - 1)));

    memcpy(CBuf->Buffer + (CBuf->Header & CBuf->Size - 1), data, len);
    memcpy(CBuf->Buffer, data+len, LenToPush - len);

    CBuf->Header += LenToPush;

    return LenToPush;
}

/**
 * @brief     get data from circular buffer
 *
 * @param[in] CBuf      the circular buffer that stored data
 * @param[in] data      target buffer that will store the data that from circular buffer
 * @param[in] LenToPop  the length that wan't to get from circular buffer
 *
 * @return      actual length that get from circular buffer
 */
unsigned int CircBuf_Pop(CircBuf_t *CBuf, unsigned char *data, unsigned int LenToPop)
{
    unsigned int len;

    LenToPop = MIN(LenToPop, CBuf->Header - CBuf->Tailer);

    len = MIN(LenToPop, CBuf->Size - (CBuf->Tailer & (CBuf->Size - 1)));

    memcpy(data, CBuf->Buffer + (CBuf->Tailer & (CBuf->Size - 1)), len);
    memcpy(data + len, CBuf->Buffer, LenToPop - len);

    CBuf->Tailer += LenToPop;

    return LenToPop;
}

/**
 * @brief     get one char from circular buffer
 *
 * @param[in] CBuf      the circular buffer that stored data
 * @param[n] data       target buffer that will store the data that from circular buffer
 *
 * @return              actual length that get from circular buffer
 */
unsigned int CircBuf_PopOneChar(CircBuf_t *CBuf, unsigned char *data)
{
    return CircBuf_Pop(CBuf, data, 1);
}

/**
 * @brief     for access data at Tailer + offset
 *
 * @param[in] CBuf      the circular buffer that stored data
 * @param[in] offset    the offset of Tailer
 *
 * @return              the data at Buffer[Tailer + offset]
 */
unsigned char CircBuf_At(CircBuf_t *CBuf, unsigned int offset)
{
    unsigned int index = (CBuf->Tailer + offset) & (CBuf->Size - 1);
    return CBuf->Buffer[index];
}


/**
 * @brief     drop the the size of data at tailer
 *
 * @param[in] CBuf          the circular buffer that stored data
 * @param[in] LenToDrop     the size of data at tailer of circular_buffer to drop
 */
void CircBuf_Drop(CircBuf_t *CBuf, unsigned int LenToDrop)
{
	if(CBuf->Tailer <= CBuf->Header - LenToDrop)	
		CBuf->Tailer += LenToDrop;
	else 
		CBuf->Tailer = CBuf->Header;
}

/**
 * @brief     get the Available memery size of circular buffer
 *
 * @param[in] CBuf  the circular buffer to get size
 *
 * @return          Available size of the circular buffer
 */
unsigned int CircBuf_GetAvalaibleSize(CircBuf_t *CBuf)
{
    return ((CBuf->Size > 0) ? (CBuf->Size - (CBuf->Header - CBuf->Tailer)) : 0);
}

/**
* @brief     get the used memery size of circular buffer
*
* @param[in] CBuf  the circular buffer to get size
*
* @return          used size of the circular buffer
*/
unsigned int CircBuf_GetUsedSize(CircBuf_t *CBuf)
{
    return (CBuf->Header - CBuf->Tailer);
}

/**
 * @brief     check if the circular buffer is empty
 *
 * @param[in] CBuf  the circular buffer to check
 *
 * @return          1   if no data stored in the circular buffer
 *                  0   if the size of circular buffer equals to 0
 *                      or some data stored in the circular buffer
 */
unsigned int CircBuf_IsEmpty(CircBuf_t *CBuf)
{
    return ((CBuf->Size > 0) && (CBuf->Header == CBuf->Tailer));
}

/**
 * @brief     check if the circular buffer is full
 *
 * @param[in] CBuf  the circular buffer to check
 *
 * @return      1 if the size of circular buffer equals to 0
 *                  or no Available space of circular buffer
 */
unsigned int CircBuf_IsFull(CircBuf_t *CBuf)
{
    return ((CBuf->Size == 0) || (CBuf->Size == (CBuf->Header - CBuf->Tailer)));
}
