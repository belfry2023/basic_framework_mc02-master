#include "buffer.h"
#include "stdlib.h"
buf_t *BUFRegister()
{
    buf_t *_buf = (buf_t *)malloc(sizeof(buf_t));
    memset(_buf, 0, sizeof(buf_t));
    _buf->id = 0;
    _buf->od = 1;
    return _buf;
}

float BUFUpdata(buf_t *_buf, float n, uint8_t time)
{
    _buf->od %= time + 1;
    _buf->quene[_buf->id++] = n;
    _buf->id %= time + 1;
    return _buf->quene[_buf->od++];
}
