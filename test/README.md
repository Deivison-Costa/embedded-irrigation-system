# Testes de host

Verificação dos algoritmos que não dependem de hardware.

```bash
cc -std=c11 -Wall -Wextra -Wshadow -O2 -I../components/nmea_gps/include \
   -o test_logic test_logic.c ../components/nmea_gps/nmea_parse.c -lm && ./test_logic
```

O parser NMEA **não é copiado**: `components/nmea_gps/nmea_parse.c` é compilado
junto, então o teste exercita exatamente o código que roda na placa. Foi por isso
que a lógica pura foi separada de `nmea_gps.c` — o que sobrou lá é só UART,
tarefa e mutex, que precisam de FreeRTOS.

O CRC do Modbus e a compensação do BMP280 ainda são cópias das funções do
firmware (são pequenas e estáveis); se forem alteradas, ajuste aqui também.

Valores de referência usados:

| Teste | Origem |
| --- | --- |
| `CRC("123456789") == 0x4B37` | valor de verificação do catálogo CRC-16/MODBUS |
| `11 03 006B 0003 -> 0x8776` | exemplo da especificação Modbus |
| `t_fine = 128422`, `T = 25,08 °C`, `P = 100653,27 Pa` | exemplo resolvido do datasheet do BMP280 |
| sentenças GGA/RMC | NMEA 0183, coordenadas do Triângulo Mineiro |

## Por que isto importa para o GPS

Um fix de GPS não pode ser validado dentro de um prédio. Estes testes cobrem
tudo o que vem depois da antena: checksum, separação de campos, conversão
`ddmm.mmmm` → graus decimais com hemisfério, altitude, satélites, HDOP, data e
hora, além dos casos de borda que mais causam bug em campo — `fix_quality = 0`,
status RMC `V` (void), sentenças de tipo não tratado que não podem apagar um fix
bom, e talker `GN` (multi-constelação) em vez de `GP`.

O que resta sem validação é apenas a recepção via satélite.
