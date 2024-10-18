#include <stdlib.h>
#include <stddef.h>
#include "circ_buffer.h"

Func_status_CB InitCircBuffer(CircBuffer *buffer, unsigned int size){
  buffer->data = malloc(size * sizeof(char));
  if(buffer->data == NULL){
    return CB_ERROR;
  }

  buffer->size = size;
  buffer->head = 0;
  buffer->tail = 0;

  return CB_OK;
}

void CircBufferReset(CircBuffer *buffer){
  buffer->head = 0;
  buffer->tail = 0;
}

void CircBufferFree(CircBuffer *buffer){
  buffer->head = 0;
  buffer->tail = 0;
  buffer->size = 0;
  free(buffer->data);
  free(buffer);
}

Func_status_CB CircBufferInsert(CircBuffer *buffer, char value){
  if(CircBufferIsFull(buffer)){
    return CB_ERROR;
  }
  
  buffer->data[buffer->head] = value;

  unsigned int next = buffer->head + 1;
  if(next == buffer->size) next = 0;
  buffer->head = next;

  return CB_OK;
}

Func_status_CB CircBufferPop(CircBuffer *buffer, char *var_to_receive_data){
  if(CircBufferIsEmpty(buffer)){
    return CB_ERROR;
  }

  *var_to_receive_data = buffer->data[buffer->tail];

  unsigned int next = buffer->tail + 1;
  if(next == buffer->size) next = 0;
  buffer->tail = next;

  return CB_OK;
}

bool CircBufferIsFull(CircBuffer *buffer){
  unsigned int next = buffer->head + 1;
  if(next == buffer->size) next = 0;

  if(next == buffer->tail) return true;

  return false;
}

bool CircBufferIsEmpty(CircBuffer *buffer){
  if(buffer->tail == buffer->head) return true;

  return false;
}
