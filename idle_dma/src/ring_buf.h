#ifndef RING_BUF_H
#define RING_BUF_H

#include <stdint.h>

// #define RB_LOCK()   __disable_irq()
// #define RB_UNLOCK()   __enable_irq()

typedef struct _rb
{
    // uint32_t arg;   //id / port /...
    int wi;  // write index
    int ri;  // read index
    int cnt; // aka used
    // int cntr;    //not use.
    uint16_t buffer_size;
    uint8_t *buffer;
} rb_t;

// int ring_buffer_init(rb_t *rb, uint8_t *buf, uint16_t size);
// int ring_buffer_put(rb_t *rb, uint8_t *buf, uint16_t size);
// int ring_buffer_get(rb_t *rb, uint8_t *buf, uint16_t size);

static inline void ring_buffer_init(rb_t *rb, uint8_t *buffer, uint16_t buffer_size)
{
    rb->buffer = buffer;
    rb->buffer_size = buffer_size;
}

static inline uint8_t ring_buffer_peek(rb_t *rb)
{
    return rb->buffer[rb->ri];
}

static inline void ring_buffer_peeks(rb_t *rb, uint8_t *buf, uint16_t size)
{
    uint16_t old_ri = rb->ri;
    for (int i = 0; i < size; ++i)
    {
        buf[i] = rb->buffer[rb->ri++];
        rb->ri &= rb->buffer_size - 1;
    }
    rb->ri = old_ri;
}

static inline uint8_t ring_buffer_pop(rb_t *rb)
{
    uint8_t read = 0;
    if (rb->cnt)
    {
        rb->cnt--;
        read = rb->buffer[rb->ri++];
        rb->ri &= rb->buffer_size - 1;
    }
    return read; // not safe!
}

static inline int ring_buffer_put(rb_t *rb, uint8_t *buf, uint16_t size)
{
    int max_write_size;
    int size2end;
    // 做一下overlap 保护, 单圈情况 wi最大只能=ri-1
    int wi, ri;
    wi = rb->wi;
    ri = rb->ri;
    if (wi >= ri)
    {
        size2end = rb->buffer_size - 1 - wi;
        max_write_size = size2end + (ri > 0 ? ri - 1 : 0);
    }
    else
    { // wi < ri
        max_write_size = ri - 1 - wi;
    }

    if (max_write_size < size)
    { // error! no enough size for put
        return -1;
    }
    for (int i = 0; i < size; ++i)
    {
        rb->buffer[rb->wi++] = buf[i];
        rb->wi &= rb->buffer_size - 1;
        rb->cnt++;
    }

    wi = rb->wi; // 调试时出现过cnt和wi、ri无法对应起来，此处进行一次修正
    rb->cnt = (wi >= ri) ? (wi - ri) : (wi + rb->buffer_size - ri);

    return 0;
}

static inline int ring_buffer_get(rb_t *rb, uint8_t *buf, uint16_t size)
{
    int read_size;
    if (size <= rb->cnt)
    {
        read_size = size;
    }
    else
    {
        read_size = 0; // 不满要读的先不读
    }

    for (int i = 0; i < read_size; ++i)
    {
        buf[i] = rb->buffer[rb->ri++];
        rb->ri &= rb->buffer_size - 1;
    }
    rb->cnt -= read_size;
    return read_size;
    // LOGI("%s rx:%d cnt:%d", __FUNCTION__, size, rb->cnt);
}

#endif