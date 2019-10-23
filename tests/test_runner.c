/*
 * Master Test Harness Runner
 * Compiles and executes all unit, integration, and regression suites in a single process.
 */

#include <stdio.h>
#include <stdlib.h>

extern int run_test_fixed_math(void);
extern int run_test_vector_matrix(void);
extern int run_test_quaternion(void);
extern int run_test_foc(void);
extern int run_test_ekf(void);
extern int run_test_closed_loop(void);
extern int run_test_edge_cases(void);

int main(void) {
    printf("==============================================================\n");
    printf("       Gimbal Master Test Harness and Verification Suite      \n");
    printf("==============================================================\n");
    
    int ret = 0;
    ret |= system("./tests/unit/test_fixed_math");
    ret |= system("./tests/unit/test_vector_matrix");
    ret |= system("./tests/unit/test_quaternion");
    ret |= system("./tests/unit/test_foc");
    ret |= system("./tests/unit/test_ekf");
    ret |= system("./tests/integration/test_closed_loop");
    ret |= system("./tests/regression/test_edge_cases");
    
    printf("--------------------------------------------------------------\n");
    if (ret == 0) {
        printf("ALL 7 TEST SUITES PASSED CLEANLY (100%% SUCCESS RATE).\n");
        return 0;
    } else {
        printf("TEST SUITE FAILURES DETECTED.\n");
        return 1;
    }
}
