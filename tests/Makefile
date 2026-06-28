# 獨立純邏輯測試（純 host 編譯，與 firmware 共用同一份 header / .c）
#   make run    編譯並依序執行全部測試（任一失敗即中止、非零退出）
#   make clean  清除
CC      ?= cc
CFLAGS  := -I../Main_Code/Core/Inc -Wall -Wextra -O2 -std=c11
LDLIBS  := -lm

BINS := test_sensor_axis test_fsm test_ekf_guard test_sensor_health test_telemetry test_telem_rx test_gs_timesync test_gs_log test_gps test_flash_ring test_link_proto test_link test_lora_calc

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

test_sensor_health: test_sensor_health.c ../Main_Code/Core/Inc/sensor_health.h
	$(CC) $(CFLAGS) -o $@ test_sensor_health.c $(LDLIBS)

test_telemetry: test_telemetry.c ../Main_Code/Core/Inc/telemetry.h ../Main_Code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_telemetry.c $(LDLIBS)

test_telem_rx: test_telem_rx.c ../Main_Code/Core/Inc/telem_rx.h ../Main_Code/Core/Inc/telemetry.h ../Main_Code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_telem_rx.c $(LDLIBS)

test_gs_timesync: test_gs_timesync.c ../Main_Code/Core/Inc/gs_timesync.h
	$(CC) $(CFLAGS) -o $@ test_gs_timesync.c $(LDLIBS)

test_gs_log: test_gs_log.c ../Main_Code/Core/Src/gs_log.c ../Main_Code/Core/Inc/gs_log.h ../Main_Code/Core/Inc/telemetry.h ../Main_Code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_gs_log.c ../Main_Code/Core/Src/gs_log.c $(LDLIBS)

test_gps: test_gps.c ../Main_Code/Core/Inc/gps_parse.h
	$(CC) $(CFLAGS) -o $@ test_gps.c $(LDLIBS)

test_flash_ring: test_flash_ring.c ../Main_Code/Core/Inc/flash_ring_math.h
	$(CC) $(CFLAGS) -o $@ test_flash_ring.c $(LDLIBS)

test_link_proto: test_link_proto.c ../Main_Code/Core/Src/link_proto.c ../Main_Code/Core/Inc/link_proto.h ../Main_Code/Core/Inc/crc16.h ../Main_Code/Core/Inc/telemetry.h
	$(CC) $(CFLAGS) -o $@ test_link_proto.c ../Main_Code/Core/Src/link_proto.c $(LDLIBS)

test_link: test_link.c ../Main_Code/Core/Src/link.c ../Main_Code/Core/Src/link_proto.c ../Main_Code/Core/Inc/link.h ../Main_Code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -o $@ test_link.c ../Main_Code/Core/Src/link.c ../Main_Code/Core/Src/link_proto.c $(LDLIBS)

test_lora_calc: test_lora_calc.c ../Main_Code/Core/Inc/lora_calc.h
	$(CC) $(CFLAGS) -o $@ test_lora_calc.c $(LDLIBS)

clean:
	rm -f $(BINS)
