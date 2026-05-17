/**
 * @file matrix.h
 * @brief Minimal dynamic matrix library in C (Phase 1 educational).
 *
 * Storage is row-major: element (row, col) is at data[row * cols + col].
 *
 * @note This is a learning module. Production code uses Eigen via fieldnav_core.
 */

#ifndef FIELDNAV_C_LEGACY_MATRIX_H
#define FIELDNAV_C_LEGACY_MATRIX_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    size_t rows;
    size_t cols;
    double *data; /**< Row-major storage, length = rows * cols. */
} Matrix;

/**
 * Allocate a zero-initialized rows x cols matrix.
 * @return pointer to the new matrix, or NULL on allocation failure
 *         or if rows == 0 || cols == 0. Caller must free with matrix_destroy.
 */
Matrix *matrix_create(size_t rows, size_t cols);

/**
 * Free a matrix and its underlying storage. Safe to call with NULL.
 */
void matrix_destroy(Matrix *m);

/**
 * Read element at (row, col). No bounds checking.
 */
double matrix_get(const Matrix *m, size_t row, size_t col);

/**
 * Write element at (row, col). No bounds checking.
 */
void matrix_set(Matrix *m, size_t row, size_t col, double value);

/**
 * Multiply: result = a * b. Returns NULL on dimension mismatch
 * (a->cols != b->rows) or allocation failure.
 */
Matrix *matrix_mul(const Matrix *a, const Matrix *b);

/**
 * Transpose: result has shape (m->cols, m->rows). Returns NULL on allocation
 * failure.
 */
Matrix *matrix_transpose(const Matrix *m);

#ifdef __cplusplus
}
#endif

#endif /* FIELDNAV_C_LEGACY_MATRIX_H */
