# 獨立軸向 / 姿態邏輯測試（純 host 編譯，與 firmware 共用同一份 header）
#   make run    編譯並執行
#   make clean  清除
CC      ?= cc
CFLAGS  := -I../Main_Code/Core/Inc -Wall -Wextra -O2 -std=c11
LDLIBS  := -lm

BIN := test_sensor_axis

.PHONY: run clean
$(BIN): $(BIN).c ../Main_Code/Core/Inc/sensor_axis.h ../Main_Code/Core/Inc/attitude_math.h
	$(CC) $(CFLAGS) -o $(BIN) $(BIN).c $(LDLIBS)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
