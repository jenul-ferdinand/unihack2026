#ifndef JSON_H
#define JSON_H
#include <stdbool.h>

typedef enum jsondatatype
{
    JSON_STRING,
    JSON_NUMBER,
    JSON_ARRAY,
    JSON_OBJECT,
    JSON_BOOLEAN,
    JSON_NULL
} jsondatatype;

typedef struct json_data json_data;

typedef struct json_kvp
{
    char *key;
    json_data *value;
} json_kvp;

typedef struct json_object
{
    json_kvp **pairs;
    int len;
} json_object;

typedef struct json_string
{
    char *data;
} json_string;

typedef struct json_number
{
    double data;
} json_number;

typedef struct json_bool
{
    bool data;
} json_bool;

typedef struct json_array
{
    json_data *data;
    int len;
} json_array;

struct json_data
{
    jsondatatype type;
    union
    {
        json_string string;
        json_number number;
        json_bool boolean;
        json_array array;
        json_object object;
    } as;
};

char *json_tostring(json_data *root);
void json_free(json_data *root);
json_kvp *init_kvp(char *key, json_data *value);
void json_push(json_data *obj, json_kvp *kvp);

json_data *init_json_object();
json_data *init_json_string(char *str);
json_data *init_json_array(int count, ...);
json_data *init_json_number(double num);
json_data *init_json_boolean(bool boolean);
json_data *init_json_object_populated(int count, ...);
json_data *get_value(json_data *obj, char *key);
void json_push_arr(json_data *arr, json_data *data);


#endif