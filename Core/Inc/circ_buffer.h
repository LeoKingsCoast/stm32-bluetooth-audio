#include <stdint.h>
#include <stdbool.h>

#ifndef BUFFER_CIRCULAR
#define BUFFER_CIRCULAR

typedef struct CircBuffer {
  char *data;
  unsigned int size;
  unsigned int head;
  unsigned int tail;
} CircBuffer;

typedef enum {
  CB_OK,
  CB_ERROR
} Func_status_CB; // Dá feedback do sucesso ou insucesso das funções de buffer circular

Func_status_CB InitCircBuffer(CircBuffer *buffer, unsigned int size);

void CircBufferReset(CircBuffer *buffer);

void CircBufferFree(CircBuffer *buffer);

Func_status_CB CircBufferInsert(CircBuffer *buffer, char value);

Func_status_CB CircBufferPop(CircBuffer *buffer, char *var_to_receive_data);

bool CircBufferIsFull(CircBuffer *buffer);

bool CircBufferIsEmpty(CircBuffer *buffer);

#endif
