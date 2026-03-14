#ifndef LIST_H
#define LIST_H
#include <stdbool.h>
typedef struct List
{
    void **data;
    int front;
    int rear;
    int allocatedsize;
    int size;
}List;

List *init_list();
void push_back(List *list,void *item);
void push_front(List *list, void* item);
void *pop(List *list);
void *popstart(List *list);
bool list_isempty(List *list);
void resize(List *list);
void freelist(List *list);
void *list_get(List *list, int idx);
void list_remove(List *list, int idx);

#endif