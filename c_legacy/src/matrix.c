#include "fieldnav/c_legacy/matrix.h"

#include <stdlib.h>

Matrix *matrix_create(size_t rows, size_t cols) {
    if (rows == 0 || cols == 0) {
        return NULL;
    }

    Matrix *m = (Matrix *)malloc(sizeof(Matrix));
    if (m == NULL) {
        return NULL;
    }

    m->rows = rows;
    m->cols = cols;
    m->data = (double *)calloc(rows * cols, sizeof(double));
    if (m->data == NULL) {
        free(m);
        return NULL;
    }

    return m;
}

void matrix_destroy(Matrix *m) {
    if (m == NULL) {
        return;
    }
    free(m->data);
    free(m);
}

double matrix_get(const Matrix *m, size_t row, size_t col) {
    return m->data[row * m->cols + col];
}

void matrix_set(Matrix *m, size_t row, size_t col, double value) {
    m->data[row * m->cols + col] = value;
}

Matrix *matrix_mul(const Matrix *a, const Matrix *b) {
    if (a->cols != b->rows) {
        return NULL;
    }

    Matrix *result = matrix_create(a->rows, b->cols);
    if (result == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < a->rows; ++i) {
        for (size_t j = 0; j < b->cols; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < a->cols; ++k) {
                sum += matrix_get(a, i, k) * matrix_get(b, k, j);
            }
            matrix_set(result, i, j, sum);
        }
    }

    return result;
}

Matrix *matrix_transpose(const Matrix *m) {
    Matrix *result = matrix_create(m->cols, m->rows);
    if (result == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < m->rows; ++i) {
        for (size_t j = 0; j < m->cols; ++j) {
            matrix_set(result, j, i, matrix_get(m, i, j));
        }
    }

    return result;
}
