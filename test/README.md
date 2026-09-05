# Host tests

Verification of the algorithms that need no hardware: the Modbus CRC, the
BMP280 compensation formulas and the NMEA parser. The functions under test are
copied verbatim from the firmware sources.

```bash
cc -std=c11 -Wall -Wextra -Wshadow -O2 -o test_logic test_logic.c -lm && ./test_logic
```

Reference values used:

| Test | Source |
| --- | --- |
| `CRC("123456789") == 0x4B37` | CRC-16/MODBUS catalogue check value |
| `11 03 006B 0003 -> 0x8776` | Modbus application protocol spec example |
| `t_fine = 128422`, `T = 25.08 °C`, `P = 100653.27 Pa` | Bosch BMP280 datasheet worked example |
| GGA/RMC checksums | NMEA 0183 reference sentences |
