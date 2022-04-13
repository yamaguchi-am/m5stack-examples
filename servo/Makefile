SOURCES=servo.ino

CLI=arduino-cli
FQBN=m5stack:esp32:m5stick-c-plus
PORT=/dev/ttyUSB0

.PHONY: all compile upload

all: compile

format: $(SOURCES)
	clang-format -style=google -i $^
	nkf --in-place --unix $^

compile: $(SOURCES)
	$(CLI) compile --fqbn $(FQBN) .

upload: compile
	$(CLI) upload -p $(PORT) --fqbn $(FQBN) .
