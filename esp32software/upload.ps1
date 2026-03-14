pio run

pio run --target clean
pio run -t upload --upload-port COM6
pio device monitor -p COM6 -b 115200

pio run -e esp32dev