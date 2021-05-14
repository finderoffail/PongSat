arduino-cli compile -p /dev/ttyUSB0 --fqbn esp32:esp32:tinypico -v audio_detect.ino && \
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:tinypico -v audio_detect.ino
