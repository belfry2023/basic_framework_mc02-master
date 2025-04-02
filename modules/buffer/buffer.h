#ifndef BUFFER_H
#define BUFFER_H

#include "stdint.h"
#include "string.h"

typedef struct 
{
    float quene[25];
    uint8_t id, od;
}buf_t;

buf_t *BUFRegister();
float BUFUpdata(buf_t *_buf, float n, uint8_t time);

#endif