# 獨立純邏輯測試（純 host 編譯，與 firmware 共用同一份 header / .c）
#   make run    編譯並依序執行全部測試（任一失敗即中止、非零退出）
#   make clean  清除
CC      ?= cc
CFLAGS  := -I../Main_Code/Core/Inc -Wall -Wextra -O2 -std=c11
LDLIBS  := -lm

BINS := test_sensor_axis test_fsm test_ekf_guard

.PHONY: run clean

run: $(BINS)
	@for b in $(BINS); do \
		echo "================ $$b ================"; \
		./$$b || exit 1; \
	done

test_sensor_axis: test_sensor_axis.c ../Main_Code/Core/Inc/sensor_axis.h ../Main_Code/Core/Inc/attitude_math.h
	$(CC) $(CFLAGS) -o $@ test_sensor_axis.c $(LDLIBS)

test_fsm: test_fsm.c ../Main_Code/Core/Src/fsm.c ../Main_Code/Core/Inc/fsm.h
	$(CC) $(CFLAGS) -o $@ test_fsm.c ../Main_Code/Core/Src/fsm.c $(LDLIBS)

test_ekf_guard: test_ekf_guard.c ../Main_Code/Core/Inc/ekf_guard.h
	$(CC) $(CFLAGS) -o $@ test_ekf_guard.c $(LDLIBS)

clean:
	rm -f $(BINS)
