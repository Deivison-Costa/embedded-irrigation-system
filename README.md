# embedded-irrigation-system

Firmware ESP-IDF para a estação de sensoriamento do sistema de irrigação.
Porte do sketch Arduino original. O `.ino` foi removido nesta branch; ele
continua acessível no histórico, em `main`:

```bash
git show main:embedded-irrigation-system/embedded-irrigation-system.ino
```

* **ESP-IDF:** v5.5.5
* **Alvo:** `esp32` (Xtensa, clássico)
* **Sensores:** anemômetro RS-485/Modbus RTU, DHT22, BH1750, BMP280, comparador LM393, GPS NMEA
* **Transporte:** MQTT sobre TLS

---

## Broker MQTT (EMQX Serverless)

Já configurado no projeto:

| Item | Valor |
| --- | --- |
| URI | `mqtts://eef86b6f.ala.us-east-1.emqxsl.com:8883` |
| CA | `main/certs/mqtt_ca.pem` — DigiCert Global Root G2, válida até 2038-01-15 |

A cadeia foi verificada contra o broker real: o servidor envia o intermediário
(*Encryption Everywhere DV TLS CA - G2*) e o folha é o curinga
`*.ala.us-east-1.emqxsl.com`, que casa com o host. Só a raiz basta —
`Verify return code: 0 (ok)`. O broker aceita TLS 1.2 com
`ECDHE-RSA-AES128-GCM-SHA256` e `ECDHE-RSA-AES256-GCM-SHA384`, ambas ativas por
padrão no mbedTLS do ESP-IDF (TLS 1.3 vem desligado por padrão no IDF, então a
negociação cai em 1.2 — testado, funciona).

**Falta definir usuário e senha.** O EMQX Serverless recusa cliente anônimo:
o TLS fecha normalmente e a conexão só é derrubada depois, no CONNACK `0x05`
(*not authorized*) — o que é fácil de confundir com problema de certificado. Crie
as credenciais no console do EMQX (*Authentication*) e coloque em:

```bash
idf.py menuconfig
```

`Irrigation System Configuration` → `MQTT` → *Username* / *Password*.
Elas ficam no `sdkconfig`, que está no `.gitignore` — não vão para o repositório.

## Compilar e gravar

```bash
. ~/esp/esp-idf/export.sh
```

Configure o Wi-Fi e as credenciais MQTT em `menuconfig`, confira os pinos e então:

```bash
idf.py build && idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Estrutura

```
CMakeLists.txt            projeto
partitions.csv            tabela de partições (app de 2,75 MB - TLS não cabe no padrão de 1 MB)
sdkconfig.defaults        configuração base (mbedTLS, watchdog, tick de 1 kHz, ...)
main/
  main.c                  inicialização e tarefa de amostragem
  app_wifi.c/.h           Wi-Fi station, backoff de reconexão, SNTP
  app_mqtt.c/.h           cliente MQTT/TLS, publicação, last will
  app_power.c/.h          motivo do reset, guarda contra loop de reset por brownout
  Kconfig.projbuild       todas as opções de menuconfig
  certs/mqtt_ca.pem       certificado da CA do broker (preencher)
components/
  modbus_rtu/             mestre Modbus RTU sobre RS-485  (substitui ModbusMaster)
  dht/                    DHT11/DHT22                     (substitui Adafruit DHT/DHT_U)
  bh1750/                 luminosidade I2C                (substitui BH1750)
  bmp280/                 pressão/temperatura I2C          (substitui Adafruit_BMP280 + Adafruit_Sensor)
  nmea_gps/               parser NMEA 0183                (substitui TinyGPSPlus)
test/
  test_logic.c            testes de host: CRC, compensação BMP280, parser NMEA
```

## Mapeamento das bibliotecas Arduino

| Arduino | ESP-IDF |
| --- | --- |
| `WiFi.h` | `esp_wifi` + `esp_netif` + `esp_event` (`main/app_wifi.c`) |
| `WiFiClientSecure` + `PubSubClient` | `esp-mqtt` com `esp-tls`/mbedTLS (`main/app_mqtt.c`) |
| `ModbusMaster` | `components/modbus_rtu` sobre `driver/uart.h` em modo RS-485 half-duplex |
| `DHT` / `DHT_U` | `components/dht` |
| `BH1750` | `components/bh1750` sobre `driver/i2c_master.h` |
| `Adafruit_BMP280` + `Adafruit_Sensor` | `components/bmp280` sobre `driver/i2c_master.h` |
| `TinyGPSPlus` | `components/nmea_gps` |
| `Wire` | `driver/i2c_master.h` (a API `driver/i2c.h` legada está depreciada na v5.5) |
| `Serial` / `Serial.println` | `esp_log` (`ESP_LOGI`/`ESP_LOGW`/`ESP_LOGE`) |
| `String` | `snprintf` em buffers de pilha |
| `setup()` / `loop()` | `app_main()` + tarefas FreeRTOS |

---

## Ligações (padrão; tudo ajustável em `menuconfig`)

| Sinal | GPIO | Observação |
| --- | --- | --- |
| RS-485 TX -> MAX485 DI | 17 | |
| RS-485 RX <- MAX485 RO | 16 | |
| RS-485 DE + RE_NEG | **5** | pino próprio; ligue DE e RE_NEG juntos |
| GPS RX <- módulo TX | 18 | |
| DHT22 DATA | 4 | pull-up externo de 4,7 k para 3V3 |
| I2C SDA (BH1750 + BMP280) | 21 | |
| I2C SCL (BH1750 + BMP280) | 22 | |
| LM393 DO | 13 | |

> **Módulos WROVER / com PSRAM:** os GPIO 16 e 17 são usados pela PSRAM.
> Nesses módulos mova o RS-485 para outros pinos (por exemplo TX 32 / RX 33),
> senão o barramento não funciona.

---

## Tópicos MQTT

Prefixo configurável, `sensors` por padrão.

| Tópico | Conteúdo |
| --- | --- |
| `sensors/windSpeed` | m/s, 1 decimal |
| `sensors/temperature`, `sensors/humidity` | DHT22 |
| `sensors/bmpTemperature`, `sensors/pressure` | BMP280 (hPa) |
| `sensors/luminosity` | lux |
| `sensors/lm393` | nível bruto do pino |
| `sensors/rain` | 0/1 já normalizado por `LM393_ACTIVE_LOW` |
| `sensors/latitude`, `longitude`, `altitude`, `satellites` | GPS |
| `sensors/uptime`, `sensors/freeHeap` | diagnóstico, a cada 30 ciclos |
| `sensors/resetReason` | por que a placa reiniciou (retained, uma vez por boot) |
| `sensors/bootAttempts` | boots seguidos que não chegaram a 60 s no ar; `1` é o valor saudável |
| `sensors/status` | `online` / `offline` (retained, via last will) |
| `sensors/errors` | JSON: `{"sensor":...,"error":...,"uptime_ms":...}` |

> **Volume de mensagens.** São 12 publicações por ciclo; a 2 s isso dá 6 msg/s,
> ~518 mil mensagens por dia, ou algo em torno de 25 MB/dia de tráfego MQTT.
> Confira contra a cota do seu plano no EMQX antes de deixar rodando direto —
> para uma estação meteorológica, `IRRIG_SAMPLE_INTERVAL_MS` de 30 s ou 60 s
> costuma ser suficiente e derruba o tráfego em uma ou duas ordens de grandeza.

---

## Problemas do código original corrigidos

### Impediam o funcionamento

1. **Conflito de pinos no RS-485.** O sketch definia `MAX485_DE 17` / `MAX485_RE_NEG 16`
   e depois `Serial2.begin(4800, SERIAL_8N1, 17, 16)` — os mesmos dois pinos serviam
   ao mesmo tempo de controle de direção e de RX/TX da UART. O `pinMode(..., OUTPUT)`
   era desfeito pela UART ao assumir os pinos, e o transceptor nunca comutava.
   Agora DE/RE tem pino próprio (RTS) e a comutação é feita **pelo hardware**
   (`UART_MODE_RS485_HALF_DUPLEX`), sem os callbacks `preTransmission`/`postTransmission`
   e sem a corrida em que o driver era liberado antes do último stop bit sair.
   `modbus_rtu_create()` rejeita explicitamente uma configuração com pinos repetidos.

2. **GPS sem chance de funcionar.** `Serial1.available()` só era consultado na
   janela entre dois `delay(2000)`. A 9600 baud chegam ~200 bytes por segundo e
   a FIFO da UART tem 128 bytes: ela transbordava a cada ciclo e as sentenças
   chegavam truncadas, de modo que `gps.location.isValid()` quase nunca ficava
   verdadeiro. Agora uma tarefa dedicada drena a UART continuamente com buffer
   de 2 kB, e o consumidor lê um snapshot consistente protegido por mutex.

3. **Verificação de luminosidade que nunca disparava.** `uint16_t lux` comparado
   com `lux < 0 || lux > 65535`: os dois lados são inalcançáveis para um
   `uint16_t`, então o compilador eliminava o teste. Pior, os códigos de erro
   `-1`/`-2` que a biblioteca BH1750 devolve viravam `65535`. A leitura agora é
   `float` com `esp_err_t` separado para o erro.

4. **Loop de Wi-Fi que travava tudo.** `while (WiFi.status() != WL_CONNECTED) delay(500);`
   nunca desistia — com senha errada o firmware ficava preso para sempre, sem
   ler sensor nenhum. Idem para `reconnect()` com `delay(5000)`. Agora a
   reconexão é assíncrona, com backoff progressivo (0,5 s → 30 s), e a amostragem
   continua rodando enquanto o link está fora.

5. **TLS sem relógio.** Sem data de sistema definida não dá para validar
   *notBefore*/*notAfter* do certificado. Foi adicionado SNTP antes da primeira
   conexão MQTT, e ligadas duas opções que o IDF deixa desativadas por padrão:
   `CONFIG_LWIP_DHCP_GET_NTP_SRV` (sem ela `esp_netif_sntp_init()` devolve
   `ESP_ERR_INVALID_ARG` quando se pede servidor via DHCP, e o relógio nunca é
   acertado) e `CONFIG_MBEDTLS_HAVE_TIME_DATE` — sem esta última o mbedTLS
   **ignora a validade do certificado**: a cadeia é conferida por assinatura,
   mas um certificado expirado passaria despercebido. Verificado em hardware:
   `clock synchronised: 2026-09-05 21:41:32 UTC`.

6. **Certificado da CA vazio.** A constante `ca_cert` tinha só as linhas
   BEGIN/END. O firmware agora recusa iniciar uma conexão `mqtts://` sem
   certificado, com mensagem explícita, em vez de falhar no handshake.

### Robustez e correção

7. **Client ID MQTT fixo** (`"ESP32Client"`): duas placas no mesmo broker se
   desconectam mutuamente em loop. Agora o MAC é anexado ao ID.

8. **Modbus sem validação.** `ModbusMaster` não expunha erro de CRC nem exceção
   Modbus de forma utilizável. O componente novo valida CRC, trata frames de
   exceção (5 bytes), aplica timeout de resposta e respeita o silêncio de
   3,5 caracteres entre frames.

9. **`bmp.begin(0x76)` com retorno ignorado.** Se o sensor não respondesse, todas
   as leituras seguintes eram lixo silencioso. Agora o chip ID é conferido
   (0x58/0x60), a calibração é validada e a falha é reportada.

10. **Temperatura do BMP280 lida e descartada.** Agora é publicada em
    `sensors/bmpTemperature` — serve de conferência cruzada com o DHT22.

11. **Pressão e temperatura lidas em transações separadas** podiam misturar
    amostras de conversões diferentes (o termo `t_fine` da temperatura entra na
    fórmula da pressão). Agora é uma leitura em rajada de `0xF7..0xFC`.

12. **`delay(2000)` no fim do loop** fazia a cadência variar com o tempo de
    leitura. Agora é `xTaskDelayUntil` com período fixo e aviso se um ciclo
    estourar o período.

13. **Intervalo mínimo do DHT22 não respeitado.** O datasheet exige 2 s entre
    leituras; o driver agora garante isso e devolve a última amostra válida em
    vez de erro se for chamado antes.

14. **Sinal do DHT22 abaixo de zero.** O bit 15 é sinal, não magnitude —
    é mascarado antes da escala.

15. **Falha de um sensor derrubava o ciclo.** Cada sensor agora é opcional em
    tempo de execução: um dispositivo ausente é registrado e os outros seguem
    publicando.

16. **Sem watchdog.** As tarefas de longa duração assinam o TWDT (30 s).

17. **Publicação de valores não finitos.** `String(NAN)` gerava payload `"nan"`.
    Agora `isfinite()` barra NaN/Inf antes de publicar.

18. **Mensagens de erro em texto livre** (`"Error in sensor X: Y"`) viraram JSON
    com `sensor`, `error` e `uptime_ms`.

19. **Sem sinalização de disponibilidade.** Foi adicionado last will retained em
    `sensors/status`.

20. **Partição padrão de 1 MB** não comporta Wi-Fi + TLS + MQTT. Tabela própria
    com `factory` de 2,75 MB (uso atual: ~1,0 MB, 65 % livre).

### Detalhes específicos de ESP-IDF tratados

21. `driver/i2c.h` está depreciada na v5.5 — usada a API nova `driver/i2c_master.h`.
22. `nvs_flash_init()` trata `ESP_ERR_NVS_NO_FREE_PAGES` / `NEW_VERSION_FOUND`,
    que quebram `esp_wifi_init()` depois de uma troca de firmware.
23. A seção crítica do DHT não chama nada residente em flash: `gpio_set_level`/
    `gpio_get_level` e `esp_timer_get_time` **não** estão em IRAM e um cache miss
    no meio do frame estoura a discriminação de 27 µs vs 70 µs. São usados
    `gpio_ll_*` (`always_inline`) e `esp_cpu_get_cycle_count()` (registrador CCOUNT).
24. `gpio_set_direction()` não é chamada dentro da seção crítica — ela pega o
    spinlock do driver GPIO por dentro. O pino fica em `INPUT_OUTPUT_OD` o tempo
    todo: nível 0 puxa a linha, nível 1 solta para o pull-up.
25. O pulso de start do DHT ficou **fora** da seção crítica (só tem duração
    mínima), reduzindo o tempo com interrupções desligadas para ~5 ms.
26. A tarefa de sensores é fixada em APP_CPU (core 1), de modo que a seção
    crítica do DHT não atinge as tarefas de Wi-Fi/LWIP em PRO_CPU.
27. A reconexão de Wi-Fi usa `esp_timer` one-shot em vez de dormir dentro do
    handler de eventos — os handlers rodam na tarefa `sys_evt` e bloquear ali
    trava inclusive o `IP_EVENT_STA_GOT_IP` que encerraria a espera.
28. `CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN` elevado para 8 kB: 4 kB não comporta a
    cadeia de vários brokers públicos e o handshake falha com `-0x7200`.
29. Rede aberta (senha vazia) ajusta `threshold.authmode` para `WIFI_AUTH_OPEN`,
    senão o scan descarta o AP.
30. `esp_rom_get_cpu_ticks_per_us()` é lido uma vez e guardado. Se algum dia
    `CONFIG_PM_ENABLE` for ligado, é preciso um lock `ESP_PM_CPU_FREQ_MAX` em
    volta de `dht_read()` — está anotado no código.

---

## Loop de reset: diagnóstico e mitigação

**Sintoma.** Depois de algum tempo funcionando normalmente, a placa passava a
reiniciar sozinha e sem parar, sem nunca mais se recuperar.

**Evidência.** 45 s de console capturados durante o loop:

- 65 boots em 45 s — um reset a cada ~700 ms;
- **64 de 64** com `rst:0x1 (POWERON_RESET)`;
- **64 de 64** morrendo na mesma linha, sempre a última a sair:

```
I (827) phy_init: phy_version 4863,a3a4459,Oct 28 2025,14:30:06
E<NUL>ets Jul 29 2019 12:21:46

rst:0x1 (POWERON_RESET),boot:0x17 (SPI_FAST_FLASH_BOOT)
```

**Causa.** `phy_init` é a calibração de RF e a ligação do amplificador de
potência: o maior degrau de corrente de todo o boot, pico da ordem de 400 mA.
O trilho de 3,3 V não sustentava esse degrau.

Aquele `E` solto é o primeiro caractere de `E (nnn) brownout: Brownout detector
was triggered`. A ISR do detector chegou a escrever uma letra e a tensão
continuou caindo — passou também do limiar de power-on reset antes que a ISR
conseguisse concluir o reset de software que ela mesma dispara. É por isso que
o motivo registrado é `POWERON_RESET` (0x01) e não `SYS_BROWN_OUT` (0x0F): o
domínio RTC é apagado junto, e com ele a dica `ESP_RST_BROWNOUT`. O evento
chega ao boot seguinte disfarçado de "alguém acabou de ligar a placa".

E o loop se auto-alimenta: todo reset volta direto para o `phy_init`, que é
exatamente o passo que falha. Não é uma falha intermitente — é a mesma falha, no
mesmo ponto, 1,4 vez por segundo, indefinidamente.

**Nada disso é defeito de firmware; é entrega de energia.** Mas o firmware não
tinha defesa nem diagnóstico para o caso, e era isso que fazia o problema
parecer inexplicável.

### O que mudou

| Mudança | Por quê |
| --- | --- |
| `ESP_BROWNOUT_DET_LVL_SEL_7` (2,80 V), era o nível 0 (2,43 V) | dispara o detector **enquanto ainda há margem**, para que a ISR consiga gravar a dica e concluir um reset limpo em vez de ser atropelada pelo POR |
| `ESP_PHY_REDUCE_TX_POWER=y` | com a dica preservada, o próprio ESP-IDF sobe o rádio na potência mínima no boot seguinte a um brownout. Esse caminho de recuperação já existia e o nível 0 impedia de ser alcançado |
| `ESP_PHY_MAX_WIFI_TX_POWER=10` (era 20) | entra nos dados de inicialização do PHY, ou seja, reduz o pico do próprio `phy_init` |
| `ESP_DEFAULT_CPU_FREQ_MHZ_80` (era 240) | corrente de base sobre a qual o pico de RF se soma |
| `main/app_power.c` | conta boots em NVS (a memória RTC não sobrevive ao POR de um brownout real), reconhece o loop, imprime o diagnóstico e **espera** 5 s → 10 s → 20 s ... antes de ligar o rádio, em vez de insistir na cadência em que falha |

O boot passa a dizer o que aconteceu, e o motivo vai para o broker em
`sensors/resetReason` e `sensors/bootAttempts` — o que transforma "reiniciou
sozinho" em algo com causa anexada no histórico, em vez de um buraco na série.

### Medições

Cada linha é uma gravação real na placa, com a NVS apagada antes para zerar o
contador:

| CPU | Teto de TX do PHY | Resultado |
| --- | --- | --- |
| 240 MHz | 20 dBm (configuração original) | 64/64 boots morrem no `phy_init` |
| 240 MHz | 13 dBm | 5/5 morrem no `phy_init` |
| 240 MHz | 10 dBm | 4/4 morrem no `phy_init` |
| 160 MHz | 10 dBm | passa o `phy_init`, associa (RSSI −20), sincroniza SNTP, abre o TLS e inicializa os sensores — e morre ~2,4 s depois no loop `invalid header: 0xffffffff` do ROM, que é a própria flash ficando sem alimentação |
| **80 MHz** | **10 dBm** | **estável: 185 s sem um único reset, MQTT/TLS conectado, heap constante em 169 KB** |

A margem é fina o bastante para que a frequência da CPU decida o resultado
sozinha: a 240 MHz a placa não sobe nem na potência de RF mínima que o PHY do
ESP32 aceita.

### A correção definitiva é de hardware

80 MHz e 10 dBm compensam uma fonte fraca ao custo de desempenho e de alcance —
não são o conserto. Na ordem em que valem ser verificados:

1. **Cabo e porta USB.** Cabo fino, longo ou só de carga derruba volts sob
   carga; porta com proteção de sobrecorrente corta o VBUS inteiro no pico de
   RF, o que explica um colapso rápido demais até para a ISR do brownout.
   Trocar por um cabo curto e grosso, ou alimentar por um hub com fonte
   própria, é o teste mais barato.
2. **Capacitor de bulk.** 470 µF ou mais entre 3V3 e GND, o mais perto possível
   do módulo, absorve o degrau de RF que o regulador não acompanha.
3. **Regulador de 3,3 V.** O AMS1117 das placas de desenvolvimento esquenta e
   sua tensão de dropout sobe com a temperatura — o que explica a placa
   funcionar por um tempo e só depois entrar em loop.
4. **Periféricos no mesmo trilho.** GPS em aquisição e o módulo RS-485 tiram
   corrente do mesmo 3,3 V. Alimentá-los à parte devolve margem ao rádio.

Resolvida a alimentação, volte `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240` e
`CONFIG_ESP_PHY_MAX_WIFI_TX_POWER=20` em `sdkconfig.defaults`. As outras três
mudanças valem manter para sempre: são detecção e recuperação, não concessão.

---

## Verificação

Compilação limpa com ESP-IDF v5.5.5 para `esp32`, sem avisos, inclusive sob
`-Wextra -Wconversion -Wshadow -Wsign-compare -Wfloat-equal`.

A cadeia TLS do broker foi validada de ponta a ponta com `openssl s_client`
contra `eef86b6f.ala.us-east-1.emqxsl.com:8883`, e foi conferido que o
`EMBED_TXTFILES` inclui o NUL terminador no comprimento (1295 bytes para um
arquivo de 1294) — o mbedTLS exige isso para parsear PEM, e sem essa garantia o
handshake falharia com `MBEDTLS_ERR_X509_INVALID_FORMAT`.

Os algoritmos que não dependem de hardware têm teste de host (`test/`), aferidos
contra valores de referência publicados — valor de verificação do CRC-16/MODBUS
(`0x4B37`), exemplo resolvido do datasheet do BMP280 (25,08 °C / 100653,27 Pa) e
sentenças NMEA de referência.

### Validado em hardware (ESP32, 2026-09-05)

Gravado e executado numa placa real, com publicação confirmada no broker EMQX
por um assinante externo em `sensors/#`:

| Subsistema | Resultado |
| --- | --- |
| Wi-Fi | conecta; reconexão com backoff exercitada de verdade (desconexão reason 2/203 no primeiro `assoc`, recuperada sozinha) |
| SNTP | `clock synchronised: 2026-09-05 21:41:32 UTC` |
| MQTT/TLS | `connected to broker`, com validação de data do certificado ativa |
| BH1750 | `initialised at 0x23`, ~45 lx |
| BMP280 | `chip id 0x58`, 932,7 hPa / 29,6 °C |
| DHT22 | 27,8 °C / 53,5 %RH |
| LM393 | leitura estável |
| GPS | UART recebendo, `crc_err=0` em todas as sentenças; sem fix (teste em ambiente interno) |
| Anemômetro | 2,6 m/s — funcionando após alimentar o sensor com 12 V |

O anemômetro não respondia por dois motivos somados: o sensor RS-485 precisa de
12 V (estava em 5 V) e o módulo transceptor é de **direção automática**, sem
DE/RE. Uma varredura de 40 combinações (TX/RX nas duas ordens × 4 baud rates ×
endereços 1-5) não recebeu um único byte, nem mesmo o eco do próprio pedido —
o que descartou software e apontou para a alimentação.

Pendente: apenas o fix de GPS a céu aberto. Todo o processamento posterior à
antena está coberto pelos testes de host.
