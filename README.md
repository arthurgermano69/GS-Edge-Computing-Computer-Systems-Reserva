# Reserva d'Água

Controlador inteligente para caixa d'água residencial, baseado em **ESP32-S3** e **FreeRTOS**. Ele mede o nível, liga e desliga a bomba automaticamente, detecta falhas (boia, entrada de água, ausência de fluxo) e envia telemetria por **MQTT com TLS**.

> **Equipe:** [nomes dos integrantes] · **Instituição / disciplina:** [preencher]
> **ODS atendidos:** [ODS 6 – Água potável e saneamento] · [ODS 12 – Consumo e produção responsáveis]

## Problema e solução

**Beneficiado:** [famílias que dependem de caixa d'água, principalmente onde o abastecimento é irregular].
**Problema:** sem monitoramento, a caixa pode esvaziar sem ninguém perceber ou transbordar quando a boia falha, desperdiçando água tratada.
**Solução:** o sistema mede o nível, comanda a bomba, avisa no local (LEDs, buzzer e tela) e envia o estado pela internet.

## Funcionalidades

- Medição de nível por ultrassom (HC-SR04), com **boia de nível** como proteção redundante.
- Controle automático da bomba por relé, com histerese (a bomba só religa quando o nível volta ao mínimo).
- **Dois sensores de fluxo** (entrada e saída) para vazão, volume acumulado e autonomia estimada.
- Estados e alertas: nível máximo sem boia, entrada de água suspeita, sem entrada confirmada e sem fluxo.
- Sinalização local: barra de 8 LEDs, buzzer e dashboard animado na tela AMOLED.
- Telemetria e eventos por **Wi-Fi + MQTT (TLS)**, com Last Will e reconexão com espera crescente.
- A rede nunca comanda o relé: se o Wi-Fi cair, o controle da água continua funcionando.
- Versão para simulação no **Wokwi**.

## Hardware

| Componente | Função |
|---|---|
| LilyGO T-Display-S3 AMOLED (ESP32-S3) | Controlador, Wi-Fi e tela de 1,91" (536×240) |
| HC-SR04 | Distância até a água (montado no topo, virado para baixo) |
| Boia de nível | Detecta caixa cheia (backup do ultrassônico) |
| 2× YF-S201 | Vazão de entrada e de saída (7,5 Hz por L/min) |
| Módulo relé 2 canais | Aciona a bomba (canal 1) |
| Buzzer passivo | Avisos sonoros |
| Barra WS2812B (8 LEDs) | Indicação visual de nível e alertas |

### Ligações

| Sinal | GPIO | Observação |
|---|---|---|
| HC-SR04 TRIG | 12 | |
| HC-SR04 ECHO | 13 | **Divisor de tensão obrigatório** (1 kΩ em série + 2 kΩ para GND): o ECHO sai em 5 V |
| Boia | 14 | Um fio no GPIO14 (pull-up interno), outro no GND |
| Fluxo de entrada (YF-S201) | 1 | Fio amarelo; alimentação em 5 V |
| Fluxo de saída (YF-S201) | 10 | Fio amarelo; alimentação em 5 V |
| Relé IN1 | 15 | Ligar a bomba no contato **NO** (normalmente aberto) |
| Relé IN2 | 2 | Reservado, sem uso na lógica |
| Buzzer (+) | 16 | |
| WS2812B DIN | 39 | Resistor de ~330 Ω em série; GND comum |
| Display AMOLED | 6, 47, 18, 7, 48, 5, 17, 38 | Já ligado na placa |

> Os sensores YF-S201 medem de cerca de 1 a 30 L/min. Vazões abaixo de 1 L/min podem não gerar pulsos.

## Software

**Ambiente:** Arduino IDE 2.x com o core **ESP32 da Espressif 3.x**.

**Bibliotecas** (Library Manager): Adafruit NeoPixel, GFX Library for Arduino (Moon On Our Nation) e PubSubClient (Nick O'Leary).

**Configurações da placa:** `ESP32S3 Dev Module`, `USB CDC On Boot: Enabled` (senão o Serial Monitor fica vazio), `Flash Size: 16MB`, `PSRAM: OPI PSRAM`, `USB Mode: Hardware CDC and JTAG`, Serial Monitor em 115200.

### Estrutura do repositório

```
RESERVA_V2/
├── RESERVA_V2.ino     # firmware do equipamento (hardware real)
├── secrets.h          # credenciais Wi-Fi/MQTT (NÃO versionar)
├── ca_cert.h          # certificado CA do broker
└── wokwi/
    ├── sketch.ino     # versão para simulação
    ├── diagram.json   # circuito do Wokwi
    └── libraries.txt  # bibliotecas do Wokwi
```

### Credenciais

Crie `secrets.h` na pasta do sketch (**nunca envie este arquivo ao repositório**):

```cpp
#pragma once
#include <Arduino.h>
const char*    WIFI_SSID    = "sua-rede";
const char*    WIFI_SENHA   = "sua-senha";
const char*    MQTT_HOST    = "seu-broker";
const uint16_t MQTT_PORTA   = 8883;
const char*    MQTT_USUARIO = "seu-usuario";
const char*    MQTT_SENHA   = "sua-senha";
const char*    MQTT_TOPICO  = "seu/topico";
```

Em `ca_cert.h`, declare o certificado do broker em uma variável chamada `CA_CERT` (é o nome usado por `MQTT_CA_CERT` no sketch).

## Parâmetros principais

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `SENSOR_MOUNT_HEIGHT_CM` | 22,0 | Distância do sensor ao fundo; altura da água = este valor − distância medida |
| `LEVEL_MIN_CM` | 1,0 | Altura em que o relé liga |
| `LEVEL_MAX_CM` | 15,0 | Altura que dispara o alerta se a boia não ativou |
| `TANK_LITERS_PER_CM` | 10,0 | Litros por cm de altura (**ajustar** à caixa real) |
| `NOINFLOW_GRACE_MS` | 60000 | Tolerância do relé ligado sem vazão de entrada |
| `NOINFLOW_LEVEL_WINDOW_MS` | 180000 | Janela de confirmação pelo nível |
| `NOINFLOW_LEVEL_RISE_CM` | 0,5 | Subida mínima do nível na janela |
| `IDLE_NOFLOW_MS` | 60000 | Tempo sem pulso nos dois sensores para `sem_fluxo` |
| `RELAY_ACTIVE_LEVEL` | `LOW` | Nível do pino que energiza a bobina |
| `FLOAT_ACTIVE_LEVEL` | `HIGH` | Nível lido com a boia ativada |

## Lógica de controle

1. Altura ≤ mínimo: o relé **liga**.
2. Boia ativada: o relé **desliga** e o ultrassônico é ignorado. Depois que a boia solta, o relé só volta a ligar com altura ≤ mínimo.
3. Altura ≥ máximo com a boia livre: o sistema conclui que a boia falhou, **desliga** o relé e dispara o alerta.
4. Entre os limites, o relé mantém o último estado.
5. Os alertas de entrada e de fluxo **apenas informam**: nunca comandam o relé.

### Estados (ordem de prioridade)

| `estado` | Quando ocorre |
|---|---|
| `iniciando` | Ainda não houve leitura do ultrassônico |
| `falha_sensor` | A última leitura do ultrassônico foi inválida |
| `alerta_nivel` | Nível máximo pelo ultrassônico com a boia livre |
| `sem_entrada` | Relé ligado, sem vazão e nível parado (confirmado em 3 min) |
| `entrada_suspeita` | Relé ligado e sem vazão de entrada há mais de 60 s |
| `sem_fluxo` | Nenhuma água entrando nem saindo há 60 s |
| `cheia` | Boia ativada |
| `enchendo` | Relé ligado |
| `normal` | Nenhuma das condições acima |

### Sinalização local

| Situação | LEDs | Buzzer |
|---|---|---|
| Falha de sensor / alerta de nível | Todos vermelhos | Rajada de 3 beeps a cada 5 s (alerta) |
| Sem entrada (confirmado) | Roxo piscando rápido (250 ms) | 4 beeps a cada 30 s |
| Entrada suspeita | Roxo piscando lento (500 ms) | 2 beeps a cada 30 s |
| Enchendo (relé ligado) | Animação azul de baixo para cima | — |
| Nível mínimo | Todos amarelos | — |
| Normal | Verde, 1 LED a cada 2 cm | Beep curto a cada variação de 5 cm |
| Boia ativou | — | Beep duplo longo |

## MQTT

O tópico base é o valor de `MQTT_TOPICO`.

| Tópico | Conteúdo |
|---|---|
| `<base>/status` | `online` / `offline` (retido; `offline` via Last Will) |
| `<base>/telemetria` | JSON retido, a cada 2 s e a cada mudança de estado |
| `<base>/eventos` | Evento pontual a cada mudança |

**Telemetria:**

```json
{"deviceId":"reserva-001","timestamp":"2026-11-19T03:00:00Z","volumeUtilL":180.0,"autonomiaH":3.6,"estado":"normal","vazaoEntradaLh":0.0,"vazaoSaidaLh":0.0}
```

Os números têm 1 casa decimal e valem `null` quando inválidos. `autonomiaH` é o volume útil dividido pela vazão média de saída.

**Eventos** (`{"ts":...,"evento":"nome","valor":"..."}`): `boia`, `rele1`, `alerta`, `fluxo`, `fluxo_saida`, `entrada` (`NORMAL`, `ENTRADA_SUSPEITA`, `SEM_ENTRADA`) e `sem_fluxo` (`ATIVO`, `ENCERRADO`).

## Simulação no Wokwi

O projeto roda no [Wokwi](https://wokwi.com) com estas substituições:

| Real | Wokwi |
|---|---|
| Tela AMOLED RM67162 | Display ILI9341 (320×240) |
| 2× YF-S201 | 2 potenciômetros (0–30 L/min) que geram pulsos de 7,5 Hz por L/min |
| Boia | Chave deslizante |
| Barra WS2812B | 8 NeoPixels |
| Wi-Fi | Rede `Wokwi-GUEST` |

**Como usar:** crie um projeto ESP32-S3, cole os arquivos de `wokwi/`, adicione as bibliotecas pelo Library Manager e crie as abas `secrets.h` e `ca_cert.h`. Arraste o controle `distance` do HC-SR04 para mudar o nível (22 cm = vazio, 7 cm = 15 cm de água).

Link do projeto: [colar o link do Wokwi aqui]

## Solução de problemas

| Sintoma | Causa provável |
|---|---|
| Serial Monitor vazio | `USB CDC On Boot` desabilitado |
| Altura errada com o tanque vazio | `SENSOR_MOUNT_HEIGHT_CM` diferente da distância real; calibre com o tanque vazio |
| Bomba liga quando deveria desligar | Carga ligada no contato NC do relé, ou módulo ativo em HIGH (ajuste `RELAY_ACTIVE_LEVEL`) |
| Boia lida ao contrário | Ajuste `FLOAT_ACTIVE_LEVEL` |
| `sem_entrada` com água entrando | Sensor de fluxo de entrada defeituoso ou vazão abaixo de 1 L/min |
| MQTT não conecta | Confira o `rc=` no Serial, o certificado em `ca_cert.h` e a hora (NTP), necessária para validar o TLS |

## Licença

[Definir a licença do projeto]