#include "json.h"
#include "stringbuilder.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <math.h>

/*
 * Internal recursive free function:
 *   free_self == 1 -> free the json_data pointer itself at the end
 *   free_self == 0 -> do not free the json_data pointer (used for elements
 *                     stored by-value inside an allocated array)
 */
static void json_free_internal(json_data *root, int free_self)
{
    if (!root)
        return;

    switch (root->type)
    {
    case JSON_STRING:
        free(root->as.string.data);
        break;

    case JSON_NUMBER:
        /* nothing to free */
        break;

    case JSON_BOOLEAN:
        /* nothing to free */
        break;

    case JSON_NULL:
        /* nothing to free */
        break;

    case JSON_ARRAY:
        if (root->as.array.data)
        {
            for (int i = 0; i < root->as.array.len; i++)
            {
                /* array elements are stored BY VALUE in parse_array(), so do not free the element pointer itself */
                json_free_internal(&root->as.array.data[i], 0);
            }
            free(root->as.array.data);
        }
        break;

    case JSON_OBJECT:
        if (root->as.object.pairs)
        {
            for (int i = 0; i < root->as.object.len; i++)
            {
                struct json_kvp *kv = root->as.object.pairs[i];
                if (!kv)
                    continue;
                free(kv->key);
                /* kv->value is a json_data* allocated by parse_value(), so free it as an owned pointer */
                if (kv->value)
                    json_free_internal(kv->value, 1);
                free(kv);
            }
            free(root->as.object.pairs);
        }
        break;
    }

    if (free_self)
    {
        free(root);
    }
}

/* public wrapper: free the root and everything it owns */
void json_free(json_data *root)
{
    json_free_internal(root, 1);
}

/* append a JSON-escaped representation of `s` (including leading/trailing quotes) */
static void append_escaped_string(stringbuilder *sb, const char *s)
{
    append_byte(sb, '"');

    for (const unsigned char *p = (const unsigned char *)s; *p; ++p)
    {
        unsigned char c = *p;
        switch (c)
        {
        case '\"':
            append(sb, "\\\"");
            break;
        case '\\':
            append(sb, "\\\\");
            break;
        case '\b':
            append(sb, "\\b");
            break;
        case '\f':
            append(sb, "\\f");
            break;
        case '\n':
            append(sb, "\\n");
            break;
        case '\r':
            append(sb, "\\r");
            break;
        case '\t':
            append(sb, "\\t");
            break;
        default:
            if (c < 0x20)
            {
                /* control character: use \u00XX */
                char buf[7];
                snprintf(buf, sizeof(buf), "\\u%04x", c);
                append(sb, buf);
            }
            else
            {
                append_byte(sb, (char)c);
            }
        }
    }

    append_byte(sb, '"');
}

static bool is_effective_integer(double x)
{
    if (!isfinite(x))
        return false;

    double intpart;
    return modf(x, &intpart) == 0.0;
}
static void json_to_string_internal(json_data *root, stringbuilder *sb)
{
    if (!root)
        return;

    switch (root->type)
    {
    case JSON_STRING:
        append_escaped_string(sb, root->as.string.data ? root->as.string.data : "");
        break;

    case JSON_NUMBER:
    {
        char buf[128];
        double v = root->as.number.data;

        int len;
        if (is_effective_integer(v))
        {
            /* Print integers with full precision, no scientific notation */
            len = snprintf(buf, sizeof(buf), "%.0f", v);
        }
        else
        {
            /* Print decimals compactly, but allow up to 20 decimal places */
            len = snprintf(buf, sizeof(buf), "%.20g", v);
        }

        append_bytes(sb, buf, len);
        break;
    }

    case JSON_BOOLEAN:
        if (root->as.boolean.data)
            append(sb, "true");
        else
            append(sb, "false");
        break;

    case JSON_NULL:
        append(sb, "null");
        break;

    case JSON_ARRAY:
        append_byte(sb, '[');
        for (int i = 0; i < root->as.array.len; i++)
        {
            if (i > 0)
                append_byte(sb, ',');
            /* elements in arrays are stored by value in your parser */
            json_to_string_internal(&root->as.array.data[i], sb);
        }
        append_byte(sb, ']');
        break;

    case JSON_OBJECT:
        append_byte(sb, '{');
        for (int i = 0; i < root->as.object.len; i++)
        {
            if (i > 0)
                append_byte(sb, ',');
            struct json_kvp *kv = root->as.object.pairs[i];
            if (!kv)
                continue;
            /* key must be an escaped string */
            append_escaped_string(sb, kv->key ? kv->key : "");
            append_byte(sb, ':');
            /* kv->value is a json_data * (owned pointer from parse_value) */
            json_to_string_internal(kv->value, sb);
        }
        append_byte(sb, '}');
        break;
    }
}

char *json_tostring(json_data *root)
{
    stringbuilder *sb = init_builder();
    json_to_string_internal(root, sb);

    /* allocate a null-terminated copy for the caller */
    char *out = malloc(sb->writtenlen + 1);
    if (!out)
    {
        free_builder(sb);
        return NULL;
    }
    memcpy(out, sb->data, sb->writtenlen);
    out[sb->writtenlen] = '\0';

    free_builder(sb);
    return out;
}

json_data *init_json_object_populated(int count, ...)
{
    if (count < 0)
        return NULL;

    json_data *j = malloc(sizeof(json_data));
    if (!j)
        return NULL;

    j->type = JSON_OBJECT;
    j->as.object.len = count;
    if (count == 0)
    {
        j->as.object.pairs = NULL;
        return j;
    }

    j->as.object.pairs = malloc(sizeof(json_kvp *) * count);
    if (!j->as.object.pairs)
    {
        free(j);
        return NULL;
    }

    va_list args;
    va_start(args, count);

    for (int i = 0; i < count; i++)
    {
        json_kvp *kvp = va_arg(args, json_kvp *);
        j->as.object.pairs[i] = kvp; // store pointer to json_kvp
    }

    va_end(args);
    return j;
}

json_data *init_json_object()
{
    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_OBJECT;
    j->as.object.len = 0;
    j->as.object.pairs = NULL;
    return j;
}

json_data *init_json_string(char *str)
{
    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_STRING;
    j->as.string.data = safestr(str);
    return j;
}

json_data *init_json_array(int count, ...)
{
    if (count < 0)
        return NULL;

    json_data *j = malloc(sizeof(json_data));
    if (!j)
        return NULL;

    j->type = JSON_ARRAY;
    j->as.array.len = count;
    if (count == 0)
    {
        j->as.array.data = NULL;
        return j;
    }

    j->as.array.data = malloc(sizeof(json_data) * count);
    if (!j->as.array.data)
    {
        free(j);
        return NULL;
    }

    va_list args;
    va_start(args, count);

    for (int i = 0; i < count; i++)
    {
        json_data *elem = va_arg(args, json_data *);
        j->as.array.data[i] = *elem; // copy the json_data struct
    }

    va_end(args);
    return j;
}

json_data *init_json_number(double num)
{
    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_NUMBER;
    j->as.number.data = num;
    return j;
}
json_data *init_json_boolean(bool boolean)
{
    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_NUMBER;
    j->as.boolean.data = boolean;
    return j;
}

json_kvp *init_kvp(char *key, json_data *value) // ASSUMES NON MALLOCED STRING
{

    json_kvp *j = malloc(sizeof(json_kvp));
    j->key = safestr(key);
    j->value = value;
    return j;
}

void json_push(json_data *obj, json_kvp *kvp)
{
    obj->as.object.len++;
    printf("objlen: %d\n", obj->as.object.len);

    if (!obj->as.object.pairs)
    {
        obj->as.object.pairs = malloc(sizeof(json_kvp *));
    }
    else
    {
        obj->as.object.pairs = realloc(obj->as.object.pairs, sizeof(json_kvp *) * obj->as.object.len);
    }
    obj->as.object.pairs[obj->as.object.len - 1] = kvp;
}

void json_push_arr(json_data *arr, json_data *data)
{
    if (!arr || arr->type != JSON_ARRAY || !data)
        return;

    int new_len = arr->as.array.len + 1;

    if (!arr->as.array.data)
    {
        arr->as.array.data = malloc(sizeof(json_data));
        if (!arr->as.array.data)
            return;
    }
    else
    {
        json_data *tmp = realloc(
            arr->as.array.data,
            sizeof(json_data) * new_len
        );
        if (!tmp)
            return;

        arr->as.array.data = tmp;
    }

    arr->as.array.data[new_len - 1] = *data; // copy struct
    arr->as.array.len = new_len;
}


json_data *get_value(json_data *obj, char *key)
{
    if(obj->type != JSON_OBJECT)
    {
        return NULL;
    }

    for(int i = 0; i < obj->as.object.len; ++i)
    {
        json_kvp *kvp = obj->as.object.pairs[i];
        if(strcmp(kvp->key, key) == 0) //Same key?
        {
            return kvp->value;
        }
    }
    return NULL;
}