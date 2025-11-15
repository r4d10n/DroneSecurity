/**
 * @file common.c
 * @brief Common definitions and utilities implementation
 */

#include "common.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Global log level */
log_level_t g_log_level = LOG_LEVEL_INFO;

/* CP Lengths from helpers.py */
const int CP_LENGTHS[MAX_SYMBOLS] = {
    72 + 8,  /* Symbol 0 */
    72,      /* Symbol 1 */
    72,      /* Symbol 2 */
    72,      /* Symbol 3 */
    72,      /* Symbol 4 */
    72,      /* Symbol 5 */
    72,      /* Symbol 6 */
    72,      /* Symbol 7 */
    72 + 8   /* Symbol 8 */
};

const int CP_LENGTHS_LEGACY[8] = {
    72 + 8, 72, 72, 72, 72, 72, 72, 72 + 8
};

const int CP_LENGTHS_C2[7] = {
    72 + 8, 72, 72, 72, 72, 72, 72 + 8
};

/* ZC symbol indices */
const int ZC_SYMBOL_IDX[2] = {3, 5};
const int ZC_SYMBOL_IDX_LEGACY[2] = {2, 4};
const int ZC_SYMBOL_IDX_C2[2] = {0, 6};

/* Buffer management */
int buffer_init(complex_buffer_t* buf, size_t capacity) {
    if (!buf) return DRONEID_ERROR_INVALID_ARG;

    buf->data = (complex_t*)malloc(capacity * sizeof(complex_t));
    if (!buf->data) return DRONEID_ERROR_MEMORY;

    buf->size = 0;
    buf->capacity = capacity;
    buf->owns_memory = true;

    return DRONEID_SUCCESS;
}

void buffer_destroy(complex_buffer_t* buf) {
    if (!buf) return;

    if (buf->owns_memory && buf->data) {
        free(buf->data);
    }

    buf->data = NULL;
    buf->size = 0;
    buf->capacity = 0;
    buf->owns_memory = false;
}

int buffer_resize(complex_buffer_t* buf, size_t new_size) {
    if (!buf || !buf->owns_memory) return DRONEID_ERROR_INVALID_ARG;

    if (new_size <= buf->capacity) {
        buf->size = new_size;
        return DRONEID_SUCCESS;
    }

    /* Need to reallocate */
    size_t new_capacity = new_size * 2; /* Grow by 2x */
    complex_t* new_data = (complex_t*)realloc(buf->data, new_capacity * sizeof(complex_t));
    if (!new_data) return DRONEID_ERROR_MEMORY;

    buf->data = new_data;
    buf->size = new_size;
    buf->capacity = new_capacity;

    return DRONEID_SUCCESS;
}
