#include "stringbuilder.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

stringbuilder *init_builder()
{
    stringbuilder *sb = malloc(sizeof(stringbuilder));
    sb->allocated = 16; // start small but nonzero
    sb->writtenlen = 0;
    sb->data = malloc(sb->allocated);
    return sb;
}

void free_builder(stringbuilder *b)
{
    if (!b)
        return;
    free(b->data);
    free(b);
}

/* ensure capacity for at least amt more bytes */
static void ensure_capacity(stringbuilder *b, int amt)
{
    if (b->writtenlen + amt <= b->allocated)
        return;

    size_t new_alloc = (b->allocated + amt) * 2; // geometric growth
    char *newdata = realloc(b->data, new_alloc);
    b->data = newdata;
    b->allocated = new_alloc;
}

void append(stringbuilder *b, const char *str)
{
    int len = strlen(str);
    ensure_capacity(b, len);
    memcpy(b->data + b->writtenlen, str, len);
    b->writtenlen += len;
}

/* read entire file into heap-allocated buffer with debug logs */
char *read_file(const char *fullpath, int *out_len)
{
    printf("Reading file :D\n");
    if (!fullpath)
    {
        printf("[read_file] fullpath is NULL!\n");
        return NULL;
    }
    printf("[read_file] Opening file: %s\n", fullpath);

    FILE *f = fopen(fullpath, "rb");
    if (!f)
    {
        printf("[read_file] Failed to open file: %s\n", fullpath);
        return NULL;
    }
    printf("[read_file] File opened successfully\n");

    if (fseek(f, 0, SEEK_END) != 0)
    {
        printf("[read_file] fseek failed\n");
        fclose(f);
        return NULL;
    }

    long size = ftell(f);
    if (size < 0)
    {
        printf("[read_file] ftell failed\n");
        fclose(f);
        return NULL;
    }
    printf("[read_file] File size: %ld bytes\n", size);

    rewind(f);

    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf)
    {
        printf("[read_file] malloc failed\n");
        fclose(f);
        return NULL;
    }

    size_t read_bytes = fread(buf, 1, (size_t)size, f);
    if (read_bytes != (size_t)size)
    {
        printf("[read_file] fread mismatch: expected %ld, got %zu\n", size, read_bytes);
        fclose(f);
        free(buf);
        return NULL;
    }

    fclose(f);
    buf[size] = '\0'; // null-terminate
    if (out_len)
        *out_len = (int)size;

    printf("[read_file] Successfully read file: %s (%ld bytes)\n", fullpath, size);

    return buf;
}

void append_bytes(stringbuilder *b, const char *src, int len)
{
    ensure_capacity(b, len);
    memcpy(b->data + b->writtenlen, src, len);
    b->writtenlen += len;
}

void append_byte(stringbuilder *b, char c)
{
    ensure_capacity(b, 1);
    b->data[b->writtenlen++] = c;
}

char *safestr(const char *str)
{
    if (!str)
        return NULL;
    size_t len = strlen(str) + 1;
    char *out = malloc(len);
    strcpy(out, str);
    return out;
}

char *strndup(const char *s, size_t n)
{
    size_t len = 0;
    while (len < n && s[len])
        len++;
    char *copy = malloc(len + 1);
    memcpy(copy, s, len);
    copy[len] = '\0';
    return copy;
}