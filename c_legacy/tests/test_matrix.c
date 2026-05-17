/**
 * @file test_matrix.c
 * @brief Bare-bones tests for the c_legacy matrix library.
 *
 * Uses a minimal assertion framework (no gtest dependency at this stage).
 * The executable returns 0 on success, non-zero on failure (ctest convention).
 */

#include "fieldnav/c_legacy/matrix.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int tests_run = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond)                                                                          \
    do {                                                                                           \
        ++tests_run;                                                                               \
        if (!(cond)) {                                                                             \
            (void)fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                  \
            ++tests_failed;                                                                        \
        }                                                                                          \
    } while (0)

#define ASSERT_NEAR(a, b, tol) ASSERT_TRUE(fabs((a) - (b)) < (tol))

static void test_create_and_destroy(void) {
    Matrix *m = matrix_create(3, 4);
    ASSERT_TRUE(m != NULL);
    ASSERT_TRUE(m->rows == 3);
    ASSERT_TRUE(m->cols == 4);
    /* calloc initializes to 0 */
    ASSERT_NEAR(matrix_get(m, 0, 0), 0.0, 1e-12);
    ASSERT_NEAR(matrix_get(m, 2, 3), 0.0, 1e-12);
    matrix_destroy(m);

    /* destroy NULL must be safe */
    matrix_destroy(NULL);

    /* zero dim returns NULL */
    ASSERT_TRUE(matrix_create(0, 5) == NULL);
    ASSERT_TRUE(matrix_create(5, 0) == NULL);
}

static void test_get_set(void) {
    Matrix *m = matrix_create(2, 2);
    matrix_set(m, 0, 0, 1.5);
    matrix_set(m, 1, 1, 2.5);
    ASSERT_NEAR(matrix_get(m, 0, 0), 1.5, 1e-12);
    ASSERT_NEAR(matrix_get(m, 1, 1), 2.5, 1e-12);
    ASSERT_NEAR(matrix_get(m, 0, 1), 0.0, 1e-12);
    ASSERT_NEAR(matrix_get(m, 1, 0), 0.0, 1e-12);
    matrix_destroy(m);
}

static void test_multiply_2x2(void) {
    /* [1 2] * [5 6] = [19 22]
       [3 4]   [7 8]   [43 50]                                           */
    Matrix *a = matrix_create(2, 2);
    matrix_set(a, 0, 0, 1);
    matrix_set(a, 0, 1, 2);
    matrix_set(a, 1, 0, 3);
    matrix_set(a, 1, 1, 4);

    Matrix *b = matrix_create(2, 2);
    matrix_set(b, 0, 0, 5);
    matrix_set(b, 0, 1, 6);
    matrix_set(b, 1, 0, 7);
    matrix_set(b, 1, 1, 8);

    Matrix *c = matrix_mul(a, b);
    ASSERT_TRUE(c != NULL);
    ASSERT_NEAR(matrix_get(c, 0, 0), 19.0, 1e-12);
    ASSERT_NEAR(matrix_get(c, 0, 1), 22.0, 1e-12);
    ASSERT_NEAR(matrix_get(c, 1, 0), 43.0, 1e-12);
    ASSERT_NEAR(matrix_get(c, 1, 1), 50.0, 1e-12);

    matrix_destroy(a);
    matrix_destroy(b);
    matrix_destroy(c);
}

static void test_multiply_dim_mismatch(void) {
    Matrix *a = matrix_create(2, 3);
    Matrix *b = matrix_create(2, 2);
    Matrix *c = matrix_mul(a, b);
    ASSERT_TRUE(c == NULL);
    matrix_destroy(a);
    matrix_destroy(b);
}

static void test_transpose(void) {
    Matrix *m = matrix_create(2, 3);
    for (size_t i = 0; i < 2; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            matrix_set(m, i, j, (double)(i * 3 + j));
        }
    }

    Matrix *t = matrix_transpose(m);
    ASSERT_TRUE(t != NULL);
    ASSERT_TRUE(t->rows == 3);
    ASSERT_TRUE(t->cols == 2);
    ASSERT_NEAR(matrix_get(t, 0, 0), 0.0, 1e-12);
    ASSERT_NEAR(matrix_get(t, 0, 1), 3.0, 1e-12);
    ASSERT_NEAR(matrix_get(t, 2, 1), 5.0, 1e-12);

    matrix_destroy(m);
    matrix_destroy(t);
}

int main(void) {
    test_create_and_destroy();
    test_get_set();
    test_multiply_2x2();
    test_multiply_dim_mismatch();
    test_transpose();

    (void)printf("c_legacy/matrix tests: %d run, %d failed\n", tests_run, tests_failed);
    return (tests_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
