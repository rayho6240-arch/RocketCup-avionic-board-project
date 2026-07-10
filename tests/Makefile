# 獨立純邏輯測試（純 host 編譯，與 firmware 共用同一份 header / .c）
#   make run    編譯並依序執行全部測試（任一失敗即中止、非零退出）
#   make clean  清除
CC      ?= cc
CFLAGS  := -I../firmware/main_flight_code/Core/Inc -Wall -Wextra -O2 -std=c11
LDLIBS  := -lm

BINS := test_sensor_axis test_fsm test_ekf_guard test_sensor_health test_telemetry test_telem_rx test_gs_timesync test_gs_log test_gps test_flash_ring test_link_proto test_link test_lora_calc test_uplink_proto

.PHONY: run clean

run: $(BINS)
	@for b in $(BINS); do \
		echo "================ $$b ================"; \
		./$$b || exit 1; \
	done

test_sensor_axis: test_sensor_axis.c ../firmware/main_flight_code/Core/Inc/sensor_axis.h ../firmware/main_flight_code/Core/Inc/attitude_math.h
	$(CC) $(CFLAGS) -o $@ test_sensor_axis.c $(LDLIBS)

test_fsm: test_fsm.c ../firmware/main_flight_code/Core/Src/fsm.c ../firmware/main_flight_code/Core/Inc/fsm.h
	$(CC) $(CFLAGS) -o $@ test_fsm.c ../firmware/main_flight_code/Core/Src/fsm.c $(LDLIBS)

test_ekf_guard: test_ekf_guard.c ../firmware/main_flight_code/Core/Inc/ekf_guard.h
	$(CC) $(CFLAGS) -o $@ test_ekf_guard.c $(LDLIBS)

test_sensor_health: test_sensor_health.c ../firmware/main_flight_code/Core/Inc/sensor_health.h
	$(CC) $(CFLAGS) -o $@ test_sensor_health.c $(LDLIBS)

test_telemetry: test_telemetry.c ../firmware/main_flight_code/Core/Inc/telemetry.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_telemetry.c $(LDLIBS)

test_telem_rx: test_telem_rx.c ../firmware/main_flight_code/Core/Inc/telem_rx.h ../firmware/main_flight_code/Core/Inc/telemetry.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_telem_rx.c $(LDLIBS)

test_gs_timesync: test_gs_timesync.c ../firmware/main_flight_code/Core/Inc/gs_timesync.h
	$(CC) $(CFLAGS) -o $@ test_gs_timesync.c $(LDLIBS)

test_gs_log: test_gs_log.c ../firmware/main_flight_code/Core/Src/gs_log.c ../firmware/main_flight_code/Core/Inc/gs_log.h ../firmware/main_flight_code/Core/Inc/telemetry.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_gs_log.c ../firmware/main_flight_code/Core/Src/gs_log.c $(LDLIBS)

test_gps: test_gps.c ../firmware/main_flight_code/Core/Inc/gps_parse.h
	$(CC) $(CFLAGS) -o $@ test_gps.c $(LDLIBS)

test_flash_ring: test_flash_ring.c ../firmware/main_flight_code/Core/Inc/flash_ring_math.h
	$(CC) $(CFLAGS) -o $@ test_flash_ring.c $(LDLIBS)

test_link_proto: test_link_proto.c ../firmware/main_flight_code/Core/Src/link_proto.c ../firmware/main_flight_code/Core/Inc/link_proto.h ../firmware/main_flight_code/Core/Inc/crc16.h ../firmware/main_flight_code/Core/Inc/telemetry.h
	$(CC) $(CFLAGS) -o $@ test_link_proto.c ../firmware/main_flight_code/Core/Src/link_proto.c $(LDLIBS)

test_link: test_link.c ../firmware/main_flight_code/Core/Src/link.c ../firmware/main_flight_code/Core/Src/link_proto.c ../firmware/main_flight_code/Core/Inc/link.h ../firmware/main_flight_code/Core/Inc/board_config.h
	$(CC) $(CFLAGS) -o $@ test_link.c ../firmware/main_flight_code/Core/Src/link.c ../firmware/main_flight_code/Core/Src/link_proto.c $(LDLIBS)

test_lora_calc: test_lora_calc.c ../firmware/main_flight_code/Core/Inc/lora_calc.h
	$(CC) $(CFLAGS) -o $@ test_lora_calc.c $(LDLIBS)

test_uplink_proto: test_uplink_proto.c ../firmware/main_flight_code/Core/Inc/uplink_proto.h ../firmware/main_flight_code/Core/Inc/crc16.h
	$(CC) $(CFLAGS) -o $@ test_uplink_proto.c $(LDLIBS)

clean:
	rm -f $(BINS)
