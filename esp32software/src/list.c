#include "list.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

List *init_list()
{
    List *result = malloc(sizeof(List));
    result->allocatedsize = 2;
    result->data = malloc(result->allocatedsize * sizeof(void *));
    result->rear = 0;
    result->front = 0;
    result->size = 0;
    return result;
}

void push_back(List *list, void *item)
{
    if (list->size == list->allocatedsize)
    {
        resize(list);
    }
    list->data[list->rear] = item;
    list->rear = (list->rear + 1) % list->allocatedsize;
    ++list->size;
}

void push_front(List *list, void *item)
{
    if (list->size == list->allocatedsize)
    {
        resize(list);
    }
    list->front = (list->front - 1 + list->allocatedsize) % list->allocatedsize;
    list->data[list->front] = item;
    ++list->size;
}

void *list_get(List *list, int idx)
{
    if (list->size == 0)
        return NULL;
    return list->data[(list->front + idx) % list->allocatedsize];
}

void *pop(List *list)
{
    if (list->size == 0)
        return NULL;
    list->rear = (list->rear - 1 + list->allocatedsize) % list->allocatedsize;
    void *result = list->data[list->rear];
    --list->size;
    return result;
}

void *popstart(List *list)
{
    if (list->size == 0)
        return NULL;
    void *result = list->data[list->front];
    list->front = (list->front + 1) % list->allocatedsize;
    --list->size;
    return result;
}

bool list_isempty(List *list)
{
    return list->size == 0;
}

void resize(List *list)
{
    int oldsize = list->allocatedsize;
    int newsize = oldsize * 2;

    void **resultdata = calloc(newsize, sizeof(void *));
    int index = list->front;

    // Copy data from old array to new array
    for (int i = 0; i < list->size; i++)
    {
        resultdata[i] = list->data[index];
        index = (index + 1) % oldsize;
    }

    free(list->data);
    list->data = resultdata;
    list->allocatedsize = newsize;

    // Reset the front and rear to proper values after resizing
    list->front = 0;
    list->rear = list->size; // Rear should be size, since it's the next free slot
}

void freelist(List *list)
{
    free(list->data);
    free(list);
}

void list_remove(List *list, int idx)
{
    if (idx < 0 || idx >= list->size)
        return; // invalid index, do nothing or handle error

    // Calculate the real index in data array
    int real_idx = (list->front + idx) % list->allocatedsize;

    // Decide whether to shift elements left (towards front) or right (towards rear)
    int dist_front = idx;                 // how many elements before idx
    int dist_rear = list->size - 1 - idx; // how many elements after idx

    if (dist_front < dist_rear)
    {
        // Shift elements before idx forward (towards rear)
        // Move elements from front..real_idx-1 one step right
        int cur = real_idx;
        while (cur != list->front)
        {
            int prev = (cur - 1 + list->allocatedsize) % list->allocatedsize;
            list->data[cur] = list->data[prev];
            cur = prev;
        }
        list->front = (list->front + 1) % list->allocatedsize;
    }
    else
    {
        // Shift elements after idx backward (towards front)
        // Move elements from real_idx+1..rear one step left
        int cur = real_idx;
        while (cur != (list->rear - 1 + list->allocatedsize) % list->allocatedsize)
        {
            int next = (cur + 1) % list->allocatedsize;
            list->data[cur] = list->data[next];
            cur = next;
        }
        list->rear = (list->rear - 1 + list->allocatedsize) % list->allocatedsize;
    }

    list->size--;
}