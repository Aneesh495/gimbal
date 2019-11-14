CC ?= clang
CFLAGS ?= -O3 -Wall -Wextra -std=c99 -I./xc/include -I./sim/include
LDFLAGS ?= -lm
XC_FLAG = -x c

SIM_SRCS = sim/src/xc_runtime.c sim/src/gimbal_physics.c sim/src/sensor_models.c sim/src/scenario_runner.c
SIM_BIN = bin/gimbal_sim

TEST_BINS = \
	bin/test_fixed_math \
	bin/test_vector_matrix \
	bin/test_quaternion \
	bin/test_foc \
	bin/test_ekf \
	bin/test_closed_loop \
	bin/test_edge_cases \
	bin/test_runner

.PHONY: all test sim lint bench clean

all: $(SIM_BIN) $(TEST_BINS)

bin:
	mkdir -p bin

$(SIM_BIN): sim/main_sim.c $(SIM_SRCS) | bin
	$(CC) $(CFLAGS) $< $(SIM_SRCS) -o $@ $(LDFLAGS)

bin/test_fixed_math: tests/unit/test_fixed_math.c | bin
	$(CC) $(CFLAGS) $< $(XC_FLAG) xc/core/fixed_math.xc -o $@ $(LDFLAGS)

bin/test_vector_matrix: tests/unit/test_vector_matrix.c | bin
	$(CC) $(CFLAGS) $< $(XC_FLAG) xc/core/fixed_math.xc xc/core/vector3.xc xc/core/matrix3x3.xc -o $@ $(LDFLAGS)

bin/test_quaternion: tests/unit/test_quaternion.c | bin
	$(CC) $(CFLAGS) $< $(XC_FLAG) xc/core/fixed_math.xc xc/core/vector3.xc xc/core/matrix3x3.xc xc/core/quaternion.xc -o $@ $(LDFLAGS)

bin/test_foc: tests/unit/test_foc.c | bin
	$(CC) $(CFLAGS) $< $(XC_FLAG) xc/core/fixed_math.xc xc/foc/clarke_park.xc xc/foc/svpwm.xc xc/foc/current_loop.xc -o $@ $(LDFLAGS)

bin/test_ekf: tests/unit/test_ekf.c | bin
	$(CC) $(CFLAGS) $< $(XC_FLAG) xc/core/fixed_math.xc xc/core/vector3.xc xc/core/matrix3x3.xc xc/core/quaternion.xc xc/estimation/es_ekf.xc xc/estimation/ekf_covariance.xc -o $@ $(LDFLAGS)

bin/test_closed_loop: tests/integration/test_closed_loop.c $(SIM_SRCS) | bin
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

bin/test_edge_cases: tests/regression/test_edge_cases.c | bin
	$(CC) $(CFLAGS) $< $(XC_FLAG) xc/core/fixed_math.xc xc/core/vector3.xc xc/core/matrix3x3.xc xc/core/quaternion.xc xc/kinematics/gimbal_kinematics.xc xc/fdir/limit_checker.xc xc/fdir/health_monitor.xc -o $@ $(LDFLAGS)

bin/test_runner: tests/test_runner.c | bin
	$(CC) $(CFLAGS) $< -o $@

test: $(TEST_BINS)
	@echo "Executing Unit and Integration Test Suites..."
	@./bin/test_fixed_math
	@./bin/test_vector_matrix
	@./bin/test_quaternion
	@./bin/test_foc
	@./bin/test_ekf
	@./bin/test_closed_loop
	@./bin/test_edge_cases
	@echo "All 7 verification suites passed with zero failures."

sim: $(SIM_BIN)
	@./$(SIM_BIN)

lint:
	@python3 scripts/check_no_em_dashes.py

bench:
	@python3 tools/generate_benchmarks.py

clean:
	rm -rf bin build *.o
