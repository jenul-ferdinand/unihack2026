#include "list.h"
#include <stdlib.h>
#include <string.h>

#ifndef STRINGBUILDER_H
#define STRINGBUILDER_H

// only compatable with char, other unicode encodings are incompatable

typedef struct stringbuilder
{
    char *data;     // we dont store a \0, this should not be used as a string and exists only to hold data
    int writtenlen; // copy data out of here with memcpy(dst, data, writtenlen); dst[strlen(dst)]='\0'
    int allocated;
} stringbuilder;
stringbuilder *init_builder();
void free_builder(stringbuilder *builder);
void expand(stringbuilder *builder, int amt);
void append(stringbuilder *builder, const char *toappend);
char *safestr(const char *str);
void append_bytes(stringbuilder *builder, const char *src, int len);
void append_byte(stringbuilder *b, const char src);
char *strndup(const char *s, size_t n);
char *read_file(const char *fullpath, int *out_len);

#endif