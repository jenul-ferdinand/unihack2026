#include "json.h"
#include "stringbuilder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tokenizer.h"
#include "list.h"

// ---------------- Debug ----------------
#define DEBUG_PRINT(...) // printf(__VA_ARGS__)  // uncomment for verbose output
void push_token(List *tokens, token t)
{
    token *tok = malloc(sizeof(token));
    if (!tok)
    {
        perror("malloc");
        exit(1);
    }
    *tok = t;
    push_back(tokens, tok);
}
void tokenize(List *tokens, const char *s)
{
    while (*s)
    {
        switch (*s)
        {
        case '{':
            push_token(tokens, (token){TOK_L_CURLY, s, 1});
            s++;
            break;
        case '}':
            push_token(tokens, (token){TOK_R_CURLY, s, 1});
            s++;
            break;
        case '[':
            push_token(tokens, (token){TOK_L_SQUARE, s, 1});
            s++;
            break;
        case ']':
            push_token(tokens, (token){TOK_R_SQUARE, s, 1});
            s++;
            break;
        case ':':
            push_token(tokens, (token){TOK_COLON, s, 1});
            s++;
            break;
        case ',':
            push_token(tokens, (token){TOK_COMMA, s, 1});
            s++;
            break;
        case ' ':
        case '\n':
        case '\t':
        case '\r':
            s++;
            break;
        case '"':
        {
            const char *start = ++s;
            while (*s)
            {
                if (*s == '\\' && *(s + 1)) // escape sequence
                {
                    s += 2; // skip both '\' and escaped char
                }
                else if (*s == '"') // actual end of string
                {
                    break;
                }
                else
                {
                    s++;
                }
            }
            push_token(tokens, (token){TOK_STRING, start, (int)(s - start)});
            if (*s == '"')
                s++;
            break;
        }

        default:
            if ((*s >= '0' && *s <= '9') || *s == '-')
            {
                const char *start = s;
                while ((*s >= '0' && *s <= '9') || *s == '.' || *s == '-')
                    s++;
                push_token(tokens, (token){TOK_NUMBER, start, (int)(s - start)});
            }
            else if (!strncmp(s, "true", 4))
            {
                push_token(tokens, (token){TOK_TRUE, s, 4});
                s += 4;
            }
            else if (!strncmp(s, "false", 5))
            {
                push_token(tokens, (token){TOK_FALSE, s, 5});
                s += 5;
            }
            else if (!strncmp(s, "null", 4))
            {
                push_token(tokens, (token){TOK_NULL, s, 4});
                s += 4;
            }
            else
            {
                s++;
            }
        }
    }
    push_token(tokens, (token){TOK_EOF, s, 0});
}

// ---------------- Parser helpers ----------------
static token *parser_cur(parser *p)
{
    if (!p || !p->tokens)
        return NULL;
    if ((size_t)p->index >= p->tokens->size)
        return NULL;
    return (token *)list_get(p->tokens, p->index);
}

static void parser_adv(parser *p)
{
    if (!p || !p->tokens)
        return;
    p->index++;
}

// ---------------- Escaped string handling ----------------
static char *unescape_string(const char *start, int len)
{
    stringbuilder *sb = init_builder();
    for (int i = 0; i < len; i++)
    {
        char c = start[i];
        if (c == '\\' && i + 1 < len)
        {
            i++;
            switch (start[i])
            {
            case '"':
                append_byte(sb, '"');
                break;
            case '\\':
                append_byte(sb, '\\');
                break;
            case '/':
                append_byte(sb, '/');
                break;
            case 'b':
                append_byte(sb, '\b');
                break;
            case 'f':
                append_byte(sb, '\f');
                break;
            case 'n':
                append_byte(sb, '\n');
                break;
            case 'r':
                append_byte(sb, '\r');
                break;
            case 't':
                append_byte(sb, '\t');
                break;
            default:
                append_byte(sb, start[i]);
                break;
            }
        }
        else
        {
            append_byte(sb, c);
        }
    }
    // null-terminate
    char *out = malloc(sb->writtenlen + 1);
    memcpy(out, sb->data, sb->writtenlen);
    out[sb->writtenlen] = '\0';
    free_builder(sb);
    return out;
}

// ---------------- Parser ----------------
json_data *parse_value(parser *p);

json_data *parse_string(parser *p)
{
    token *t = parser_cur(p);
    if (!t)
        return NULL;
    parser_adv(p);

    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_STRING;
    j->as.string.data = unescape_string(t->start, t->length);

    DEBUG_PRINT("Parsed string: %s\n", j->as.string.data);
    return j;
}

json_data *parse_number(parser *p)
{
    token *t = parser_cur(p);
    if (!t)
        return NULL;
    parser_adv(p);

    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_NUMBER;
    j->as.number.data = strtod(t->start, NULL);

    DEBUG_PRINT("Parsed number: %g\n", j->as.number.data);
    return j;
}

json_data *parse_literal(parser *p, enum jsondatatype type, int val)
{
    parser_adv(p);
    json_data *j = malloc(sizeof(json_data));
    j->type = type;
    if (type == JSON_BOOLEAN)
        j->as.boolean.data = val;

    DEBUG_PRINT("Parsed literal: %s\n", type == JSON_BOOLEAN ? (val ? "true" : "false") : "null");
    return j;
}

json_data *parse_array(parser *p)
{
    parser_adv(p); // skip '['
    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_ARRAY;

    List *children = init_list();
    while (1)
    {
        token *t = parser_cur(p);
        if (!t || t->type == TOK_R_SQUARE)
            break;
        json_data *child = parse_value(p);
        push_back(children, child);
        t = parser_cur(p);
        if (t && t->type == TOK_COMMA)
            parser_adv(p);
    }
    parser_adv(p); // skip ']'

    j->as.array.len = children->size;
    j->as.array.data = malloc(sizeof(json_data) * children->size);
    for (int i = 0; i < children->size; i++)
        j->as.array.data[i] = *(json_data *)list_get(children, i);

    for (int i = 0; i < children->size; i++)
        free(list_get(children, i));
    freelist(children);

    DEBUG_PRINT("Parsed array of %d elements\n", j->as.array.len);
    return j;
}

json_data *parse_object(parser *p)
{
    parser_adv(p); // skip '{'
    json_data *j = malloc(sizeof(json_data));
    j->type = JSON_OBJECT;

    List *pairs = init_list();
    while (1)
    {
        token *t = parser_cur(p);
        if (!t || t->type == TOK_R_CURLY)
            break;

        token *keytok = parser_cur(p);
        parser_adv(p); // skip string token
        parser_adv(p); // skip ':'

        json_data *value = parse_value(p);

        struct json_kvp *kv = malloc(sizeof(struct json_kvp));
        kv->key = strndup(keytok->start, keytok->length);
        kv->value = value;

        push_back(pairs, kv);

        t = parser_cur(p);
        if (t && t->type == TOK_COMMA)
            parser_adv(p);
    }
    parser_adv(p); // skip '}'

    j->as.object.len = pairs->size;
    j->as.object.pairs = malloc(sizeof(struct json_kvp *) * pairs->size);
    for (int i = 0; i < pairs->size; i++)
        j->as.object.pairs[i] = (struct json_kvp *)list_get(pairs, i);

    freelist(pairs);

    DEBUG_PRINT("Parsed object with %d pairs\n", j->as.object.len);
    return j;
}

json_data *parse_value(parser *p)
{
    token *t = parser_cur(p);
    if (!t)
        return NULL;

    switch (t->type)
    {
    case TOK_STRING:
        return parse_string(p);
    case TOK_NUMBER:
        return parse_number(p);
    case TOK_TRUE:
        return parse_literal(p, JSON_BOOLEAN, 1);
    case TOK_FALSE:
        return parse_literal(p, JSON_BOOLEAN, 0);
    case TOK_NULL:
        return parse_literal(p, JSON_NULL, 0);
    case TOK_L_SQUARE:
        return parse_array(p);
    case TOK_L_CURLY:
        return parse_object(p);
    default:
        return NULL;
    }
}


bool validate_json(char *s)
{
    int curly = 0, square = 0;
    bool in_string = false;
    bool escape = false;

    for (int i = 0; s[i]; i++)
    {
        char c = s[i];

        if (in_string)
        {
            if (escape)
            {
                escape = false; // skip the next char, whatever it is
            }
            else if (c == '\\')
            {
                escape = true;
            }
            else if (c == '"')
            {
                in_string = false;
            }
            continue;
        }

        switch (c)
        {
        case '{':
            curly++;
            break;
        case '}':
            if (curly == 0)
                return false;
            curly--;
            break;
        case '[':
            square++;
            break;
        case ']':
            if (square == 0)
                return false;
            square--;
            break;
        case '"':
            in_string = true;
            break;
        case ':':
        case ',':
        case ' ':
        case '\n':
        case '\t':
        case '\r':
            // allowed structural characters, ignore
            break;
        default:
            // for simplicity, we don't validate numbers/literals in this pass
            break;
        }
    }

    // After processing everything, all brackets must be closed and not in a string
    if (curly != 0 || square != 0 || in_string)
        return false;

    return true;
}


// ---------------- JSON entry ----------------
json_data *parse_json(char *input)
{
    if(!validate_json(input))
    {
        return NULL;
    }
    List *tokens = init_list();
    tokenize(tokens, input);

    parser p = {tokens, 0};
    json_data *root = parse_value(&p);

    // free tokens
    while (!list_isempty(tokens))
        free(popstart(tokens));
    freelist(tokens);

    return root;
}