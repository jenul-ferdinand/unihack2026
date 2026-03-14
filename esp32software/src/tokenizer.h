#include "json.h"
#include "list.h"

#ifndef TOKENIZER_H
#define TOKENIZER_H

typedef enum
{
    TOK_L_CURLY,
    TOK_R_CURLY,
    TOK_L_SQUARE,
    TOK_R_SQUARE,
    TOK_COLON,
    TOK_COMMA,
    TOK_STRING,
    TOK_NUMBER,
    TOK_TRUE,
    TOK_FALSE,
    TOK_NULL,
    TOK_EOF
} tokentype;

typedef struct
{
    tokentype type;
    const char *start;
    int length;
} token;

typedef struct
{
    List *tokens;
    int index;
} parser;

// Tokenization
void tokenize(List *tokens, const char *s);
void push_token(List *tokens, token t);
void free_tokens(List *tokens);

// Parsing functions
json_data *parse_value(parser *p);
json_data *parse_string(parser *p);
json_data *parse_number(parser *p);
json_data *parse_literal(parser *p, enum jsondatatype type, int val);
json_data *parse_array(parser *p);
json_data *parse_object(parser *p);

// Convenience entry point
json_data *parse_json(char *input);

#endif