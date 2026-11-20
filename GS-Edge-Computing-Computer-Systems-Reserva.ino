/*
 * =====================================================================
 *  Controle de água residencial - ESP32-S3 (LilyGO T-Display-S3 AMOLED)
 *  Etapa 16: alerta de "relé ligado sem vazão de entrada"
 * =====================================================================
 *
 * GEOMETRIA:
 *   O HC-SR04 fica a 22 cm do fundo, apontado para a água (de cima p/ baixo).
 *   altura da água = SENSOR_MOUNT_HEIGHT_CM - distância medida
 *
 * RELÉ 1:
 *   LIGADO    = bobina ENERGIZADA  (LED do canal ACESO)
 *   DESLIGADO = bobina SEM ENERGIA (LED do canal APAGADO)
 *   O relé INICIA DESLIGADO.
 *
 * LÓGICA:
 *   1) altura <= LEVEL_MIN_CM (1 cm)  -> relé LIGA
 *   2) BOIA ATIVADA (nível lógico 1)  -> relé DESLIGA; ultrassônico IGNORADO
 *                                        até a boia desligar. Depois da boia
 *                                        desligar o relé continua DESLIGADO
 *                                        até a altura voltar a <= LEVEL_MIN_CM.
 *   3) altura >= LEVEL_MAX_CM (15 cm) com a boia NÃO ativada (falha da boia)
 *                                     -> relé DESLIGA (backup), beeps de
 *                                        alerta e barra de LEDs TODA vermelha.
 *   4) entre os limites: o relé mantém o último estado.
 *
 * MONITOR DE ENTRADA (só avisa, NUNCA comanda o relé):
 *   Relé LIGADO e sem vazão de entrada por NOINFLOW_GRACE_MS (60 s)
 *       -> estado "sem_fluxo" (suspeita)
 *   Ao fim da janela de NOINFLOW_LEVEL_WINDOW_MS (3 min), se o nível NÃO subiu
 *   pelo menos NOINFLOW_LEVEL_RISE_CM (0,5 cm)
 *       -> estado "sem_entrada" (CONFIRMADO: a água não está entrando)
 *   Se o nível subiu, a água entra e o sensor de fluxo é que é suspeito
 *       -> continua "sem_fluxo". Normaliza quando a vazão volta ou o relé desliga.
 *   Avisos: LEDs roxos piscando, buzzer, chip na tela, MQTT e serial.
 *
 * LEDs (prioridade):
 *   vermelho (falha/alerta de nível) > roxo piscando (sem fluxo/entrada)
 *   > azul animado (enchendo) > amarelo (nível mínimo) > barra verde (nível)
 *
 * REDE: Wi-Fi + MQTT rodam em uma task própria. Sem rede, o controle da água
 *       continua funcionando normalmente (a rede só publica dados).
 *
 * TELEMETRIA (<base>/telemetria, JSON):
 *   deviceId, timestamp (UTC ISO 8601), volumeUtilL, autonomiaH, estado,
 *   vazaoEntradaLh, vazaoSaidaLh
 *
 * Arquivos na mesma pasta do sketch:
 *   secrets.h : WIFI_SSID, WIFI_SENHA, MQTT_HOST, MQTT_PORTA, MQTT_USUARIO,
 *               MQTT_SENHA, MQTT_TOPICO (tópico BASE)
 *   ca_cert.h : certificado CA (o NOME da variável deve ser o mesmo usado em
 *               MQTT_CA_CERT mais abaixo)
 *
 * Bibliotecas (Library Manager):
 *   - Adafruit NeoPixel
 *   - GFX Library for Arduino (Moon On Our Nation) - versão recente
 *   - PubSubClient (Nick O'Leary)
 * Core ESP32 da Espressif 3.x (usa ledcAttach/ledcWriteTone/attachInterruptArg)
 *
 * Arduino IDE (placa "ESP32S3 Dev Module" ou "LilyGo T-Display-S3"):
 *   - USB CDC On Boot: Enabled   (senão o Serial Monitor fica vazio!)
 *   - Flash Size: 16MB, PSRAM: OPI PSRAM, USB Mode: Hardware CDC and JTAG
 *   - Serial Monitor: 115200
 *
 * Ligação:
 *   HC-SR04: VCC->5V  GND->GND  TRIG->GPIO12
 *            ECHO->GPIO13 COM DIVISOR (1k série + 2k p/ GND) - o ECHO sai em 5V!
 *   Boia   : um fio no GPIO14 (pull-up interno), outro fio no GND
 *   Fluxo ENTRADA: YF-S201 vermelho->5V, preto->GND, amarelo (sinal)->GPIO1
 *   Fluxo SAÍDA  : YF-S201 vermelho->5V, preto->GND, amarelo (sinal)->GPIO10
 *   Relé   : IN1->GPIO15, IN2->GPIO2
 *   Buzzer : pino + -> GPIO16, pino - -> GND
 *   WS2812B: DIN->GPIO39 (resistor ~330R em série), 5V->5V, GND->GND (GND comum)
 *   Display: AMOLED onboard (QSPI: CS6 SCK47 D0=18 D1=7 D2=48 D3=5 RST17, EN38)
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "secrets.h"   // WIFI_SSID, WIFI_SENHA, MQTT_HOST, MQTT_PORTA, MQTT_USUARIO, MQTT_SENHA, MQTT_TOPICO
#include "ca_cert.h"   // certificado CA do broker

// =====================================================================
//  CONFIGURAÇÃO
// =====================================================================
#define APP_DEBUG_MODE          1      // 1 = logs DBG + tabela periódica de todos os sensores
#define SENSOR_DEBUG_PERIOD_MS  2000   // período da tabela com todos os sensores

// ---- Geometria e limites de nível (ALTURA DA ÁGUA, em cm) ----
#define SENSOR_MOUNT_HEIGHT_CM  22.76f  // distância do sensor ao FUNDO (sensor no alto, virado p/ baixo)
#define LEVEL_MIN_CM            3.0f   // altura mínima: <= este valor => relé LIGA + LEDs amarelos
#define LEVEL_MAX_CM            15.0f  // altura máxima: >= este valor => alerta + relé DESLIGA (backup)
#define ALERT_HYST_CM           0.0f   // histerese p/ encerrar o alerta (altura < MAX - histerese)

// ---- Volume do reservatório (usado na telemetria) ----
#define TANK_LITERS_PER_CM      10.0f  // litros por cm de altura da água (AJUSTAR: capacidade / altura)

// ---- Monitor de entrada (relé ligado e sem vazão) ----
#define NOINFLOW_GRACE_MS         60000    // tolerância: 60 s com o relé ligado e sem vazão antes de alertar
#define NOINFLOW_LEVEL_WINDOW_MS  180000   // janela de confirmação pelo nível: 3 min
#define NOINFLOW_LEVEL_RISE_CM    0.5f     // subida mínima do nível na janela p/ considerar que a água entra
#define NOINFLOW_BEEP_REPEAT_MS   30000    // o aviso sonoro se repete a cada 30 s
#define NOINFLOW_BEEP_ON_MS       300      // duração de cada beep do aviso
#define NOINFLOW_BEEP_OFF_MS      200      // pausa entre os beeps do aviso

// ---- HC-SR04 ----
#define HCSR04_TRIG_GPIO        12
#define HCSR04_ECHO_GPIO        13
#define HCSR04_PERIOD_MS        2000   // leitura periódica a cada 2 s
#define HCSR04_ECHO_TIMEOUT_MS  40     // tempo máximo esperando o eco
#define HCSR04_MIN_CM           2.0f   // faixa válida de DISTÂNCIA do sensor
#define HCSR04_MAX_CM           400.0f
#define HCSR04_AIR_TEMP_C       20.0f  // compensação de temperatura (fixa por enquanto)

// ---- Boia de nível (vertical) ----
#define FLOAT_GPIO              14
#define FLOAT_ACTIVE_LEVEL      HIGH   // nível lido com a boia ATIVADA (nível 1 = ativada)
#define FLOAT_SAMPLE_MS         50     // período de amostragem da boia
#define FLOAT_DEBOUNCE_MS       500    // estado precisa ficar estável por este tempo

// ---- Sensores de fluxo YF-S201 (entrada e saída) ----
#define FLOW_GPIO               1      // sensor de ENTRADA
#define FLOW_HZ_PER_LPM         7.5f   // YF-S201: frequência(Hz) = 7,5 x vazão(L/min). Calibrar se preciso
#define FLOW2_GPIO              10     // sensor de SAÍDA
#define FLOW2_HZ_PER_LPM        7.5f   // calibração do sensor de saída (YF-S201 = 7,5)
#define FLOW_SAMPLE_MS          2000   // janela de cálculo da vazão (2 s)
#define FLOW_MIN_PULSE_US       1000   // filtro: ignora bordas mais próximas que 1 ms (ruído)
#define FLOW_TASK_STACK         3072
#define FLOW_TASK_PRIO          3

// ---- Barra de LEDs ----
#define LED_GPIO                39
#define LED_COUNT               8
#define LED_BRIGHTNESS          40     // 0..255
#define LED_STEP_CM             2.0f   // 1 LED a cada 2 cm
#define LED_BAR_SHOW_HEIGHT     1      // 1 = barra mostra ALTURA da água | 0 = mostra a DISTÂNCIA medida
#define LED_UPDATE_MS           100
#define LED_FAIL_THRESHOLD      2      // falhas consecutivas até ficar tudo vermelho
#define LED_FILL_STEP_MS        250    // tempo entre um LED azul e o próximo (enchendo)
#define LED_FILL_HOLD_MS        1000   // tempo com todos acesos antes de apagar e repetir (1 s)
#define LED_NOFLOW_BLINK_MS     500    // piscada do roxo em "sem fluxo" (suspeita)
#define LED_NOENTRY_BLINK_MS    250    // piscada do roxo em "sem entrada" (confirmado): mais rápida

// ---- Display AMOLED (RM67162, QSPI) ----
#define DISPLAY_ENABLE_GPIO     38     // habilita a alimentação do painel
#define DISPLAY_CS_GPIO         6
#define DISPLAY_SCK_GPIO        47
#define DISPLAY_D0_GPIO         18
#define DISPLAY_D1_GPIO         7
#define DISPLAY_D2_GPIO         48
#define DISPLAY_D3_GPIO         5
#define DISPLAY_RST_GPIO        17
#define DISPLAY_ROTATION        1      // 1 = paisagem 536x240 (use 3 para girar 180 graus)
#define DISPLAY_BRIGHTNESS      200    // 0..255
#define DISPLAY_FRAME_MS        40     // ~25 quadros por segundo (animação da água)
#define DISPLAY_TASK_STACK      8192
#define DISPLAY_TASK_PRIO       2

// ---- Wi-Fi / MQTT ----
#define DEVICE_ID               "reserva-001"     // identificação do equipamento (deviceId + ClientID)
#define MQTT_USE_TLS            1                 // 1 = TLS com certificado CA | 0 = sem TLS
#define MQTT_CA_CERT            CA_CERT           // NOME da variável declarada no ca_cert.h
#define MQTT_TELEMETRY_SUFFIX   "/telemetria"     // "" = publicar a telemetria direto em MQTT_TOPICO
#define MQTT_TELEMETRY_MS       2000              // telemetria periódica (2 s)
#define MQTT_KEEPALIVE_S        30
#define OUTFLOW_AVG_ALPHA       0.2f              // suavização da vazão de saída p/ a autonomia (0..1)
#define AUTONOMY_MIN_FLOW_LH    1.0f              // abaixo disto (L/h) a autonomia é "null" (sem consumo)
#define WIFI_RETRY_MS           20000             // se não conectar, tenta de novo a cada 20 s
#define NTP_WAIT_MS             15000             // espera máxima pela hora (necessária p/ validar TLS)
#define TZ_OFFSET_S             (-3 * 3600)       // fuso de Brasília
#define NTP_SERVER_1            "pool.ntp.org"
#define NTP_SERVER_2            "time.google.com"
#define NET_TASK_STACK          10240
#define NET_TASK_PRIO           1

// ---- Buzzer ----
#define BUZZER_GPIO             16
#define BUZZER_ACTIVE           0      // 0 = passivo (tom por PWM) | 1 = ativo (nível HIGH)
#define BUZZER_FREQ_HZ          2700
#define BUZZER_BEEP_MS          100    // beep curto (variação de BUZZER_STEP_CM)
#define BUZZER_LONG_BEEP_MS     800    // beep longo (usado no beep duplo da boia)
#define BUZZER_DOUBLE_GAP_MS    200    // pausa entre os dois beeps longos (boia ativada)
#define BUZZER_STEP_CM          5.0f   // beep curto a cada variação de 5 cm

// ---- Alerta de nível máximo pelo ultrassônico ----
#define ALERT_BEEP_COUNT        3      // quantidade de beeps por rajada
#define ALERT_BEEP_ON_MS        150    // duração de cada beep da rajada
#define ALERT_BEEP_OFF_MS       100    // pausa entre os beeps da rajada
#define ALERT_REPEAT_MS         5000   // a rajada se repete a cada 5 s enquanto o alerta durar

// ---- Relé (módulo 2 canais) ----
#define RELAY1_GPIO             15
#define RELAY2_GPIO             2
#define RELAY2_ENABLE           1      // 1 = inicializa o canal 2 DESLIGADO (ainda sem uso na lógica)
#define RELAY_ACTIVE_LEVEL      LOW    // nível do pino que ENERGIZA a bobina (módulos comuns: LOW)

// ---- Proteção em tempo de compilação contra pinos duplicados ----
#if (FLOAT_GPIO == HCSR04_ECHO_GPIO) || (FLOAT_GPIO == HCSR04_TRIG_GPIO) || \
    (FLOAT_GPIO == BUZZER_GPIO) || (FLOAT_GPIO == RELAY1_GPIO) || \
    (FLOAT_GPIO == RELAY2_GPIO) || (FLOAT_GPIO == LED_GPIO)
  #error "FLOAT_GPIO coincide com outro pino em uso: escolha um GPIO livre"
#endif
#if RELAY2_ENABLE && (RELAY2_GPIO == BUZZER_GPIO)
  #error "RELAY2_GPIO e BUZZER_GPIO sao o mesmo pino"
#endif
#if (FLOW_GPIO == HCSR04_ECHO_GPIO) || (FLOW_GPIO == HCSR04_TRIG_GPIO) || \
    (FLOW_GPIO == BUZZER_GPIO) || (FLOW_GPIO == RELAY1_GPIO) || \
    (FLOW_GPIO == RELAY2_GPIO) || (FLOW_GPIO == LED_GPIO) || (FLOW_GPIO == FLOAT_GPIO)
  #error "FLOW_GPIO coincide com outro pino em uso: escolha um GPIO livre"
#endif
#if (FLOW2_GPIO == FLOW_GPIO) || (FLOW2_GPIO == HCSR04_ECHO_GPIO) || (FLOW2_GPIO == HCSR04_TRIG_GPIO) || \
    (FLOW2_GPIO == BUZZER_GPIO) || (FLOW2_GPIO == RELAY1_GPIO) || (FLOW2_GPIO == RELAY2_GPIO) || \
    (FLOW2_GPIO == LED_GPIO) || (FLOW2_GPIO == FLOAT_GPIO)
  #error "FLOW2_GPIO coincide com outro pino em uso: escolha um GPIO livre"
#endif
// Verificações em tempo de compilação (static_assert aceita valores decimais)
static_assert(LEVEL_MIN_CM < LEVEL_MAX_CM, "LEVEL_MIN_CM deve ser menor que LEVEL_MAX_CM");
static_assert(LEVEL_MAX_CM < SENSOR_MOUNT_HEIGHT_CM, "LEVEL_MAX_CM deve ser menor que a altura de montagem do sensor");
static_assert(NOINFLOW_GRACE_MS < NOINFLOW_LEVEL_WINDOW_MS, "a tolerancia deve ser menor que a janela de confirmacao");

// ---- Tasks FreeRTOS (pilha em bytes / prioridade) ----
#define HCSR04_TASK_STACK       4096
#define HCSR04_TASK_PRIO        5
#define CONTROL_TASK_STACK      4096
#define CONTROL_TASK_PRIO       4
#define FLOAT_TASK_STACK        3072
#define FLOAT_TASK_PRIO         4
#define LED_TASK_STACK          4096
#define LED_TASK_PRIO           3
#define DEBUG_TASK_STACK        4096
#define DEBUG_TASK_PRIO         2

// Log de debug (some do binário quando APP_DEBUG_MODE = 0)
#if APP_DEBUG_MODE
  #define DBG(fmt, ...) Serial.printf("[DBG %8lu] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#else
  #define DBG(fmt, ...)
#endif

// =====================================================================
//  SENSOR HUB: guarda a última leitura de cada sensor (thread-safe)
// =====================================================================
#define SENSOR_HUB_MAX 8

// Resultado da última tentativa de medição de um sensor
enum SensorStatus {
  SENSOR_NO_DATA = 0,   // ainda sem leitura
  SENSOR_OK,            // leitura válida
  SENSOR_TIMEOUT,       // sem resposta/eco
  SENSOR_OUT_OF_RANGE,  // fora da faixa válida
  SENSOR_ERROR
};

// Dados de um sensor no hub
struct SensorReading {
  const char  *name;
  const char  *unit;
  float        value;          // último valor VÁLIDO
  SensorStatus status;         // status da última tentativa
  int64_t      timestamp_us;   // instante da última atualização
  uint32_t     ok_count;       // total de leituras válidas
  uint32_t     err_count;      // total de falhas
};

// Campo de texto com cache do display (declarado aqui no topo porque o Arduino IDE
// gera os protótipos das funções antes das definições de struct que vêm depois)
struct TextField { char last[40]; uint16_t fg; };

// Dados de UM medidor de vazão (permite reutilizar o mesmo driver para entrada e saída)
struct FlowMeter {
  const char *tag;                // nome curto (task e logs)
  const char *hubFlow;            // nome da vazão no hub
  const char *hubVol;             // nome do volume no hub
  int         gpio;               // pino do sinal
  float       hzPerLpm;           // calibração: Hz por L/min
  int         flowId;             // id da vazão no hub
  int         volId;              // id do volume no hub
  volatile uint32_t pulses;       // pulsos contados desde a última leitura
  volatile int64_t  lastEdgeUs;   // último pulso aceito (filtro de ruído)
  volatile bool     flowing;      // true = há água passando
  portMUX_TYPE      mux;          // protege o contador entre ISR e task
};

static SemaphoreHandle_t hubMutex;                 // protege o vetor abaixo entre tasks
static SensorReading     sensors[SENSOR_HUB_MAX];
static int               sensorCount = 0;

// Converte o status em texto para o terminal
static const char *statusStr(SensorStatus s) {
  switch (s) {
    case SENSOR_NO_DATA:      return "SEM_DADOS";
    case SENSOR_OK:           return "OK";
    case SENSOR_TIMEOUT:      return "TIMEOUT";
    case SENSOR_OUT_OF_RANGE: return "FORA_FAIXA";
    default:                  return "ERRO";
  }
}

// Cria o mutex e zera o vetor de sensores
static void hubInit() {
  hubMutex = xSemaphoreCreateMutex();
  memset(sensors, 0, sizeof(sensors));
}

// Registra um sensor novo e devolve seu id (ou -1 se o hub estiver cheio)
static int hubRegister(const char *name, const char *unit) {
  int id = -1;
  xSemaphoreTake(hubMutex, portMAX_DELAY);
  if (sensorCount < SENSOR_HUB_MAX) {
    id = sensorCount++;
    sensors[id].name   = name;
    sensors[id].unit   = unit;
    sensors[id].status = SENSOR_NO_DATA;
  }
  xSemaphoreGive(hubMutex);
  if (id < 0) Serial.printf("ERRO: hub cheio, nao registrou '%s'\n", name);
  else        DBG("sensor registrado: id=%d nome=%s unidade=%s", id, name, unit);
  return id;
}

// Publica o resultado de uma medição (o valor só é guardado se status == OK)
static void hubPublish(int id, float value, SensorStatus status) {
  if (id < 0 || id >= SENSOR_HUB_MAX) return;
  xSemaphoreTake(hubMutex, portMAX_DELAY);
  SensorReading &s = sensors[id];
  s.status       = status;
  s.timestamp_us = esp_timer_get_time();
  if (status == SENSOR_OK) { s.value = value; s.ok_count++; }
  else                     { s.err_count++; }
  xSemaphoreGive(hubMutex);
}

// Copia a leitura atual de um sensor (retorna false se o id for inválido)
static bool hubGet(int id, SensorReading &out) {
  if (id < 0 || id >= SENSOR_HUB_MAX) return false;
  xSemaphoreTake(hubMutex, portMAX_DELAY);
  out = sensors[id];
  xSemaphoreGive(hubMutex);
  return true;
}

// Quantidade de sensores registrados
static int hubCount() {
  xSemaphoreTake(hubMutex, portMAX_DELAY);
  int c = sensorCount;
  xSemaphoreGive(hubMutex);
  return c;
}

// =====================================================================
//  ESTADOS COMPARTILHADOS ENTRE TASKS
// =====================================================================
// Estado do monitor de entrada (relé ligado x vazão de entrada)
#define INFLOW_STATE_OK       0   // normal (relé desligado, ou ligado com vazão)
#define INFLOW_STATE_NOFLOW   1   // relé ligado sem vazão há mais que a tolerância (suspeita)
#define INFLOW_STATE_NOENTRY  2   // CONFIRMADO: o nível também não sobe (água não entra)

static volatile bool relay1On    = false;   // true = LIGADO (bobina energizada, LED aceso)
static volatile bool floatActive = false;   // true = boia ativada (nível máximo atingido)
static volatile bool levelAlert  = false;   // true = ultrassônico indica nível máximo (falha da boia)
static volatile int  inflowState = INFLOW_STATE_OK;   // estado do monitor de entrada
static volatile bool noFlowIdle  = false;   // true = sem água entrando nem saindo há IDLE_NOFLOW_MS
static volatile bool wifiUp      = false;   // true = Wi-Fi conectado
static volatile bool mqttUp      = false;   // true = MQTT conectado
static volatile int  wifiRssi    = 0;       // intensidade do sinal Wi-Fi (dBm)
static const char * volatile relay1Reason = "INICIAL";  // motivo do último comando do relé

static int distId  = -1;   // id no hub: distância medida pelo sensor (cm)
static int levelId = -1;   // id no hub: altura da água calculada (cm)
static int flowId  = -1;   // id no hub: vazão de ENTRADA (L/min)
static int volId   = -1;   // id no hub: volume de ENTRADA (L)
static int flow2Id = -1;   // id no hub: vazão de SAÍDA (L/min)
static int vol2Id  = -1;   // id no hub: volume de SAÍDA (L)

// Texto do estado do monitor de entrada (para logs)
static const char *inflowStr(int s) {
  switch (s) {
    case INFLOW_STATE_NOFLOW:  return "SEM_FLUXO";
    case INFLOW_STATE_NOENTRY: return "SEM_ENTRADA";
    default:                   return "NORMAL";
  }
}

// =====================================================================
//  TASK DE DEBUG: imprime periodicamente os valores de TODOS os sensores
// =====================================================================
static void debugTask(void *arg) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // Espera o próximo período da tabela
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_DEBUG_PERIOD_MS));

    int n = hubCount();
    int64_t now = esp_timer_get_time();

    Serial.printf("\n=== SENSORES  t=%lld ms  heap livre=%u B ===\n",
                  (long long)(now / 1000), (unsigned)ESP.getFreeHeap());

    // Uma linha por sensor registrado no hub (distância, altura, fluxos, volumes...)
    for (int i = 0; i < n; i++) {
      SensorReading r;
      if (!hubGet(i, r)) continue;
      long long age_ms = r.timestamp_us ? (now - r.timestamp_us) / 1000 : -1;
      Serial.printf("[%d] %-12s = %8.2f %-3s | %-10s | ok=%lu err=%lu | idade=%lld ms\n",
                    i, r.name, r.value, r.unit, statusStr(r.status),
                    (unsigned long)r.ok_count, (unsigned long)r.err_count, age_ms);
    }

    // Boia, limites, alerta, relé, entrada e rede (com o nível real dos pinos p/ diagnóstico de ligação)
    Serial.printf("    boia (GPIO%d): %s | nivel lido=%d (ativa em %d)\n",
                  FLOAT_GPIO, floatActive ? "ATIVADA" : "NAO ATIVA",
                  digitalRead(FLOAT_GPIO), FLOAT_ACTIVE_LEVEL);
    Serial.printf("    limites: altura min=%.1f cm | max=%.1f cm | sensor a %.1f cm do fundo\n",
                  (double)LEVEL_MIN_CM, (double)LEVEL_MAX_CM, (double)SENSOR_MOUNT_HEIGHT_CM);
    Serial.printf("    alerta de nivel maximo (ultrassonico): %s\n", levelAlert ? "ATIVO" : "inativo");
    Serial.printf("    rele1: %s | GPIO%d=%d (bobina energiza em %d) | ultimo comando: %s\n",
                  relay1On ? "LIGADO (bobina energizada, LED aceso)" : "DESLIGADO (bobina sem energia, LED apagado)",
                  RELAY1_GPIO, digitalRead(RELAY1_GPIO), RELAY_ACTIVE_LEVEL, relay1Reason);
    Serial.printf("    entrada: %s\n", inflowStr(inflowState));
    Serial.printf("    rede: wifi=%s (rssi=%d dBm) | mqtt=%s\n",
                  wifiUp ? "OK" : "SEM", wifiRssi, mqttUp ? "OK" : "SEM");
  }
}

// =====================================================================
//  DRIVER HC-SR04 (ultrassônico)
// =====================================================================
static TaskHandle_t     hcsr04Task  = NULL;
static volatile int64_t echoRiseUs  = 0;    // instante da borda de subida do ECHO
static volatile int64_t echoPulseUs = 0;    // largura do pulso medido

// ISR chamada nas DUAS bordas do ECHO: mede a largura do pulso e acorda a task
static void IRAM_ATTR echoIsr() {
  int64_t now = esp_timer_get_time();
  if (digitalRead(HCSR04_ECHO_GPIO)) {
    echoRiseUs = now;                               // subida: marca o início
  } else {
    echoPulseUs = now - echoRiseUs;                 // descida: calcula a largura
    BaseType_t hpWoken = pdFALSE;
    vTaskNotifyGiveFromISR(hcsr04Task, &hpWoken);   // avisa a task que a medição acabou
    if (hpWoken) portYIELD_FROM_ISR();
  }
}

// Velocidade do som (m/s) corrigida pela temperatura do ar
static inline float soundSpeedMs(float tempC) {
  return 331.3f + 0.606f * tempC;
}

// Task do ultrassônico: dispara o pulso, mede o eco, converte para cm e
// publica no hub a DISTÂNCIA medida e a ALTURA da água (altura de montagem - distância)
static void hcsr04TaskFn(void *arg) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // --- 1) Dispara o pulso de TRIG (2 us em LOW, 10 us em HIGH) ---
    ulTaskNotifyTake(pdTRUE, 0);          // descarta notificação velha do ECHO
    echoPulseUs = 0;

    digitalWrite(HCSR04_TRIG_GPIO, LOW);
    delayMicroseconds(2);
    digitalWrite(HCSR04_TRIG_GPIO, HIGH);
    delayMicroseconds(10);
    digitalWrite(HCSR04_TRIG_GPIO, LOW);

    // --- 2) Aguarda o fim do eco (a ISR acorda a task) ---
    uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(HCSR04_ECHO_TIMEOUT_MS));
    if (!got) {
      // Sem eco dentro do tempo: publica falha nos dois valores
      DBG("HC-SR04: timeout aguardando eco");
      hubPublish(distId,  0, SENSOR_TIMEOUT);
      hubPublish(levelId, 0, SENSOR_TIMEOUT);
    } else {
      // --- 3) Converte tempo do eco em distância: (tempo x velocidade) / 2 ---
      int64_t pulseUs = echoPulseUs;
      float distCm = (float)pulseUs * soundSpeedMs(HCSR04_AIR_TEMP_C) / 20000.0f;

      // --- 4) Altura da água = altura de montagem - distância medida ---
      float heightCm = SENSOR_MOUNT_HEIGHT_CM - distCm;
      DBG("HC-SR04: pulso=%lld us -> dist=%.2f cm -> altura da agua=%.2f cm",
          (long long)pulseUs, distCm, heightCm);

      // --- 5) Valida a faixa do sensor e publica ---
      SensorStatus st = (distCm < HCSR04_MIN_CM || distCm > HCSR04_MAX_CM)
                        ? SENSOR_OUT_OF_RANGE : SENSOR_OK;
      hubPublish(distId,  distCm,   st);
      hubPublish(levelId, heightCm, st);
    }

    // --- 6) Espera até a próxima leitura (período fixo) ---
    vTaskDelayUntil(&last, pdMS_TO_TICKS(HCSR04_PERIOD_MS));
  }
}

// Configura os pinos, registra os dois valores no hub e cria a task + interrupção
static bool hcsr04Start() {
  distId  = hubRegister("dist_sensor", "cm");     // distância sensor -> água
  levelId = hubRegister("altura_agua", "cm");     // altura da água (montagem - distância)
  if (distId < 0 || levelId < 0) return false;

  pinMode(HCSR04_TRIG_GPIO, OUTPUT);
  digitalWrite(HCSR04_TRIG_GPIO, LOW);
  pinMode(HCSR04_ECHO_GPIO, INPUT);

  if (xTaskCreate(hcsr04TaskFn, "hcsr04", HCSR04_TASK_STACK, NULL,
                  HCSR04_TASK_PRIO, &hcsr04Task) != pdPASS) return false;

  // A ISR só é ligada depois que a task existe (ela usa o handle da task)
  attachInterrupt(digitalPinToInterrupt(HCSR04_ECHO_GPIO), echoIsr, CHANGE);

  DBG("HC-SR04 iniciado: TRIG=GPIO%d ECHO=GPIO%d periodo=%d ms, montado a %.1f cm",
      HCSR04_TRIG_GPIO, HCSR04_ECHO_GPIO, HCSR04_PERIOD_MS, (double)SENSOR_MOUNT_HEIGHT_CM);
  return true;
}

// =====================================================================
//  BOIA DE NÍVEL
// =====================================================================

// Lê o pino e devolve true se a boia está ATIVADA (considerando a polaridade configurada)
static bool floatReadRaw() {
  return digitalRead(FLOAT_GPIO) == FLOAT_ACTIVE_LEVEL;
}

// Task da boia: amostra o pino com debounce e imprime no terminal cada mudança de estado
static void floatTask(void *arg) {
  bool stable = floatActive;                         // estado inicial lido em floatStart()
  uint32_t cnt = 0;                                  // amostras consecutivas diferentes do estado
  const uint32_t need = FLOAT_DEBOUNCE_MS / FLOAT_SAMPLE_MS;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(FLOAT_SAMPLE_MS));
    bool raw = floatReadRaw();
    if (raw == stable) { cnt = 0; continue; }        // sem mudança: zera o contador
    if (++cnt >= need) {                             // mudança estável por tempo suficiente
      stable = raw;
      cnt = 0;
      floatActive = stable;                          // publica o novo estado p/ as outras tasks
      Serial.printf("[BOIA] %s\n", stable ? "ATIVADA" : "NAO ATIVA");
    }
  }
}

// Configura o pino (pull-up interno), lê o estado inicial e cria a task
static bool floatStart() {
  pinMode(FLOAT_GPIO, INPUT_PULLUP);
  delay(5);                                          // deixa o pull-up estabilizar
  floatActive = floatReadRaw();
  Serial.printf("[BOIA] estado inicial (GPIO%d): %s\n",
                FLOAT_GPIO, floatActive ? "ATIVADA" : "NAO ATIVA");

  if (xTaskCreate(floatTask, "float", FLOAT_TASK_STACK, NULL,
                  FLOAT_TASK_PRIO, NULL) != pdPASS) return false;

  DBG("Boia iniciada: GPIO%d, ativa em nivel %s, debounce=%d ms",
      FLOAT_GPIO, FLOAT_ACTIVE_LEVEL == LOW ? "LOW" : "HIGH", FLOAT_DEBOUNCE_MS);
  return true;
}

// =====================================================================
//  SENSORES DE FLUXO YF-S201 (entrada e saída, pulsos por interrupção)
// =====================================================================
// Dois medidores usando o mesmo código; cada um tem seu próprio contador
static FlowMeter flowIn  = { "flow_in",  "fluxo_ent", "vol_ent", FLOW_GPIO,  FLOW_HZ_PER_LPM,  -1, -1, 0, 0, false, portMUX_INITIALIZER_UNLOCKED };
static FlowMeter flowOut = { "flow_out", "fluxo_sai", "vol_sai", FLOW2_GPIO, FLOW2_HZ_PER_LPM, -1, -1, 0, 0, false, portMUX_INITIALIZER_UNLOCKED };

// ISR: chamada a cada borda de SUBIDA; o argumento indica de qual medidor veio o pulso
static void IRAM_ATTR flowIsr(void *arg) {
  FlowMeter *m = (FlowMeter *)arg;
  int64_t now = esp_timer_get_time();
  if (now - m->lastEdgeUs < FLOW_MIN_PULSE_US) return;   // pulso muito próximo = ruído
  m->lastEdgeUs = now;
  portENTER_CRITICAL_ISR(&m->mux);
  m->pulses++;
  portEXIT_CRITICAL_ISR(&m->mux);
}

// Task do fluxo (uma por medidor): converte pulsos em vazão (L/min), acumula o
// volume (L) e publica os dois valores no hub
static void flowTaskFn(void *arg) {
  FlowMeter *m = (FlowMeter *)arg;
  TickType_t last   = xTaskGetTickCount();
  int64_t    lastUs = esp_timer_get_time();
  double     totalL = 0;                // volume acumulado desde a partida
  bool       wasFlowing = false;        // estado anterior (detecta início/fim de fluxo)

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(FLOW_SAMPLE_MS));

    // --- 1) Lê e zera o contador de pulsos com segurança ---
    portENTER_CRITICAL(&m->mux);
    uint32_t pulses = m->pulses;
    m->pulses = 0;
    portEXIT_CRITICAL(&m->mux);

    // --- 2) Usa o tempo REAL decorrido para calcular a frequência ---
    int64_t now = esp_timer_get_time();
    float dtS = (float)(now - lastUs) / 1000000.0f;
    lastUs = now;

    // --- 3) Frequência -> vazão (L/min) e pulsos -> volume (L) ---
    float freqHz  = (float)pulses / dtS;
    float flowLpm = freqHz / m->hzPerLpm;
    totalL += (double)pulses / (double)(m->hzPerLpm * 60.0f);   // 7,5 x 60 = 450 pulsos/L

    DBG("%s: pulsos=%lu em %.2f s -> %.2f Hz -> %.2f L/min | total=%.3f L",
        m->tag, (unsigned long)pulses, dtS, freqHz, flowLpm, totalL);

    // --- 4) Publica no hub (aparece na tabela de debug) ---
    hubPublish(m->flowId, flowLpm, SENSOR_OK);
    hubPublish(m->volId,  (float)totalL, SENSOR_OK);

    // --- 5) Mensagem visível quando o fluxo começa ou para ---
    bool flowing = (pulses > 0);
    m->flowing = flowing;                           // compartilha o estado (usado pelo MQTT e pelo monitor de entrada)
    if (flowing != wasFlowing) {
      wasFlowing = flowing;
      Serial.printf("[FLUXO %s] %s\n", m->tag, flowing ? "AGUA PASSANDO" : "SEM FLUXO");
    }
  }
}

// Registra os valores no hub, configura o pino e liga a interrupção + task de UM medidor
static bool flowMeterInit(FlowMeter *m) {
  m->flowId = hubRegister(m->hubFlow, "L/m");
  m->volId  = hubRegister(m->hubVol,  "L");
  if (m->flowId < 0 || m->volId < 0) return false;

  pinMode(m->gpio, INPUT_PULLUP);

  if (xTaskCreate(flowTaskFn, m->tag, FLOW_TASK_STACK, m, FLOW_TASK_PRIO, NULL) != pdPASS) return false;

  attachInterruptArg(digitalPinToInterrupt(m->gpio), flowIsr, m, RISING);

  DBG("Fluxo '%s' iniciado: GPIO%d, %.1f Hz por L/min, janela=%d ms",
      m->tag, m->gpio, (double)m->hzPerLpm, FLOW_SAMPLE_MS);
  return true;
}

// Inicia os dois medidores e guarda os ids para o display e o MQTT
static bool flowStart() {
  bool ok = flowMeterInit(&flowIn) && flowMeterInit(&flowOut);
  flowId  = flowIn.flowId;   volId  = flowIn.volId;
  flow2Id = flowOut.flowId;  vol2Id = flowOut.volId;
  return ok;
}

// =====================================================================
//  BARRA DE LEDs WS2812B
// =====================================================================
static Adafruit_NeoPixel strip(LED_COUNT, LED_GPIO, NEO_GRB + NEO_KHZ800);

// Estados especiais da barra (valores >= 0 = quantidade de LEDs verdes)
#define LED_STATE_FAULT    (-1)  // falha do sensor: tudo vermelho
#define LED_STATE_ALERT    (-2)  // nível máximo pelo ultrassônico: tudo vermelho
#define LED_STATE_NONE     (-3)  // nada exibido ainda
#define LED_STATE_LOW      (-4)  // nível mínimo ou abaixo: tudo amarelo
#define LED_STATE_FILL     (-5)  // relé ligado (enchendo): animação azul de baixo p/ cima
#define LED_STATE_NOINFLOW (-6)  // relé ligado sem vazão de entrada: roxo piscando

// Acende 'lit' LEDs em verde (os demais ficam apagados)
static void ledShowBar(int lit) {
  strip.clear();
  for (int i = 0; i < lit && i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 255, 0));    // verde
  }
  strip.show();
}

// Acende TODOS os LEDs em vermelho (falha do sensor ou alerta de nível máximo)
static void ledShowAllRed() {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));    // vermelho
  }
  strip.show();
}

// Acende TODOS os LEDs em amarelo (nível mínimo ou abaixo)
static void ledShowAllYellow() {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(255, 160, 0));  // amarelo (o G reduzido evita o tom esverdeado)
  }
  strip.show();
}

// Acende TODOS os LEDs em roxo (relé ligado sem vazão de entrada)
static void ledShowAllPurple() {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(170, 0, 255));  // roxo
  }
  strip.show();
}

// Acende 'lit' LEDs em azul, de baixo para cima (os demais ficam apagados)
static void ledShowBlue(int lit) {
  strip.clear();
  for (int i = 0; i < lit && i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0, 80, 255));   // azul
  }
  strip.show();
}

// Task dos LEDs. Prioridade do que mostrar:
//   1) vermelho  : falha do sensor ou alerta de nível máximo
//   2) roxo piscando : relé ligado sem vazão de entrada (suspeita / confirmado)
//   3) azul      : relé LIGADO (enchendo) -> animação cíclica de baixo p/ cima
//   4) amarelo   : altura <= nível mínimo
//   5) verde     : barra de nível (1 LED a cada LED_STEP_CM)
// Animação de enchimento: acende 1 LED azul por vez, de baixo para cima; com todos
// acesos espera LED_FILL_HOLD_MS (1 s), apaga tudo e repete, até o relé desligar.
static void ledTask(void *arg) {
  const int srcId = LED_BAR_SHOW_HEIGHT ? levelId : distId;   // o que a barra representa
  int64_t  lastTs    = 0;
  int      failCount = 0;
  int      lastState = LED_STATE_NONE;
  bool     haveData  = false;
  int      fillLit   = 0;          // quantos LEDs azuis estão acesos na animação
  uint32_t fillTs    = 0;          // instante da última mudança da animação
  int      lastBlink = -1;         // última fase (acesa/apagada) do roxo piscando

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(LED_UPDATE_MS));

    SensorReading r;
    if (!hubGet(srcId, r)) continue;
    if (r.timestamp_us == 0) continue;            // ainda sem nenhuma medição

    // Processa apenas leituras novas: conta falhas consecutivas
    if (r.timestamp_us != lastTs) {
      lastTs = r.timestamp_us;
      haveData = true;
      if (r.status == SENSOR_OK) failCount = 0;
      else                       failCount++;
    }
    if (!haveData) continue;

    // Decide o que mostrar: falha > alerta > sem entrada > enchendo > nível mínimo > barra
    int state;
    int is = inflowState;                          // cópia local do estado do monitor de entrada
    if (failCount >= LED_FAIL_THRESHOLD) {
      state = LED_STATE_FAULT;
    } else if (levelAlert) {
      state = LED_STATE_ALERT;
    } else if (is != INFLOW_STATE_OK) {
      state = LED_STATE_NOINFLOW;                 // relé ligado sem vazão: roxo piscando
    } else if (relay1On) {
      state = LED_STATE_FILL;                     // relé ligado: animação de enchimento
    } else if (LED_BAR_SHOW_HEIGHT && r.status == SENSOR_OK && r.value <= LEVEL_MIN_CM) {
      state = LED_STATE_LOW;                      // altura <= mínimo: tudo amarelo
    } else {
      state = (int)(r.value / LED_STEP_CM);       // 1 LED a cada LED_STEP_CM
      if (state > LED_COUNT) state = LED_COUNT;
      if (state < 0) state = 0;                   // altura negativa = 0 LEDs
    }

    // --- Roxo piscando (sem fluxo = lento, sem entrada confirmado = rápido) ---
    if (state == LED_STATE_NOINFLOW) {
      uint32_t period = (is == INFLOW_STATE_NOENTRY) ? LED_NOENTRY_BLINK_MS : LED_NOFLOW_BLINK_MS;
      int phase = (int)((millis() / period) % 2);   // 0 = acesa, 1 = apagada
      if (lastState != LED_STATE_NOINFLOW || phase != lastBlink) {
        if (phase == 0) ledShowAllPurple();
        else            ledShowBar(0);                // apaga tudo
        lastBlink = phase;
      }
      if (lastState != LED_STATE_NOINFLOW) {
        DBG("LED: ROXO PISCANDO (%s)", inflowStr(is));
      }
      lastState = LED_STATE_NOINFLOW;
      continue;
    }

    // --- Animação de enchimento: roda a cada ciclo, sem esperar mudança de estado ---
    if (state == LED_STATE_FILL) {
      uint32_t now = millis();
      if (lastState != LED_STATE_FILL) {          // acabou de começar a encher
        fillLit = 0;
        fillTs  = now;
        ledShowBlue(0);                           // começa com tudo apagado
        DBG("LED: ENCHENDO (animacao azul iniciada)");
        lastState = LED_STATE_FILL;
      } else if (fillLit < LED_COUNT) {
        if (now - fillTs >= LED_FILL_STEP_MS) {   // acende o próximo LED (de baixo p/ cima)
          fillLit++;
          ledShowBlue(fillLit);
          fillTs = now;
        }
      } else if (now - fillTs >= LED_FILL_HOLD_MS) {   // todos acesos há 1 s: apaga e repete
        fillLit = 0;
        ledShowBlue(0);
        fillTs = now;
      }
      continue;                                   // não cai na lógica abaixo
    }

    // --- Demais estados: só atualiza a fita quando o resultado muda ---
    if (state != lastState) {
      if (state >= 0) {
        ledShowBar(state);
        DBG("LED: %d de %d acesos (verde) | valor=%.2f cm", state, LED_COUNT, r.value);
      } else if (state == LED_STATE_LOW) {
        ledShowAllYellow();
        DBG("LED: TUDO AMARELO (nivel minimo: %.2f cm <= %.1f cm)", r.value, (double)LEVEL_MIN_CM);
      } else {
        ledShowAllRed();
        DBG("LED: TUDO VERMELHO (%s)",
            state == LED_STATE_FAULT ? "falha do sensor" : "alerta de nivel maximo");
      }
      lastState = state;
    }
  }
}

// Inicializa a fita (apagada) e cria a task
static bool ledBarStart() {
  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();

  if (xTaskCreate(ledTask, "ledbar", LED_TASK_STACK, NULL,
                  LED_TASK_PRIO, NULL) != pdPASS) return false;

  DBG("Barra de LEDs iniciada: GPIO%d, %d LEDs, brilho=%d, %.1f cm/LED, mostra %s",
      LED_GPIO, LED_COUNT, LED_BRIGHTNESS, (double)LED_STEP_CM,
      LED_BAR_SHOW_HEIGHT ? "ALTURA da agua" : "DISTANCIA medida");
  return true;
}

// =====================================================================
//  DISPLAY AMOLED (RM67162, QSPI): painel gráfico de monitoramento
// =====================================================================
//  Layout (paisagem 536x240):
//   - Cabeçalho: gota + título, "chip" de estado (pisca em alerta), ícone de
//     rede (Wi-Fi/MQTT) e tempo ligado
//   - Esquerda : TANQUE animado (onda, bolhas, marcas MIN/MAX, bolinha da boia)
//   - Direita  : 4 cartões -> NIVEL | FLUXO (entrada/saída) | BOIA | RELE 1
//   O tanque usa um "canvas" (buffer pequeno) p/ animar sem piscar.
//   Textos só são redesenhados quando o valor muda.
// =====================================================================
static Arduino_DataBus *dispBus = NULL;
static Arduino_GFX     *gfx     = NULL;
static Arduino_Canvas  *tankCv  = NULL;

// ---- Paleta (RGB565) ----
#define C565(r, g, b)  ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define COL_BG         C565(6, 10, 18)       // fundo
#define COL_HEADER     C565(10, 18, 34)      // faixa do cabeçalho
#define COL_CARD       C565(16, 26, 44)      // fundo dos cartões
#define COL_EDGE       C565(36, 52, 82)      // bordas
#define COL_TRACK      C565(30, 42, 66)      // trilho das barras
#define COL_TEXT       C565(235, 242, 255)
#define COL_DIM        C565(120, 140, 175)
#define COL_BLUE       C565(70, 175, 255)
#define COL_GREEN      C565(70, 225, 140)
#define COL_AMBER      C565(255, 186, 64)
#define COL_RED        C565(255, 84, 84)
#define COL_CYAN       C565(80, 232, 232)
#define COL_PURPLE     C565(190, 100, 255)   // alerta de entrada (sem fluxo / sem entrada)
#define COL_TANKBG     C565(8, 14, 26)       // interior do tanque

// ---- Geometria do layout (todos os valores pares: o RM67162 prefere) ----
#define TANK_X     12
#define TANK_Y     36
#define TANK_W     124
#define TANK_H     192
#define CAN_W      116     // canvas = interior do tanque (inset de 4 px)
#define CAN_H      184
#define CARD_W     184
#define CARD_H     92
#define CA_X       148     // coluna esquerda dos cartões
#define CB_X       340     // coluna direita dos cartões
#define CR1_Y      36      // linha de cima
#define CR2_Y      136     // linha de baixo
#define CHIP_X     232
#define CHIP_W     160

// Estados do "chip" do cabeçalho
enum { CHIP_INIT = 0, CHIP_FAULT, CHIP_ALERT, CHIP_NOENTRY, CHIP_NOFLOW, CHIP_FLOAT, CHIP_FILLING, CHIP_NORMAL };

static TextField fUptime, fLevel, fLevelSub, fFlow, fFlow2, fFlowSub;
static TextField fFloatTxt, fFloatSub, fFloatRaw, fRelayTxt, fRelaySub, fRelayRaw;

// Escreve um texto (tamanho 'size') limpando antes a caixa clearW x clearH.
// Retorna sem desenhar se nada mudou desde a última chamada.
static void drawText(TextField &f, int16_t x, int16_t y, int16_t clearW, int16_t clearH,
                     uint8_t size, uint16_t fg, uint16_t bg, const char *txt) {
  if (f.fg == fg && strcmp(f.last, txt) == 0) return;
  strncpy(f.last, txt, sizeof(f.last) - 1);
  f.last[sizeof(f.last) - 1] = 0;
  f.fg = fg;
  gfx->fillRect(x, y, clearW, clearH, bg);
  gfx->setTextSize(size);
  gfx->setTextColor(fg);
  gfx->setCursor(x, y);
  gfx->print(txt);
}

// Texto simples, sem cache (usado no desenho estático)
static void drawLabel(int16_t x, int16_t y, uint8_t size, uint16_t fg, const char *txt) {
  gfx->setTextSize(size);
  gfx->setTextColor(fg);
  gfx->setCursor(x, y);
  gfx->print(txt);
}

// Moldura de um cartão: fundo arredondado, borda, marca colorida e título
static void drawCardFrame(int16_t x, int16_t y, const char *title, uint16_t accent) {
  gfx->fillRoundRect(x, y, CARD_W, CARD_H, 10, COL_CARD);
  gfx->drawRoundRect(x, y, CARD_W, CARD_H, 10, COL_EDGE);
  gfx->fillRoundRect(x + 10, y + 10, 4, 10, 2, accent);
  drawLabel(x + 20, y + 12, 1, COL_DIM, title);
}

// Barra horizontal de progresso com marcas de MIN e MAX
static void drawBar(int16_t x, int16_t y, int16_t w, int16_t h, float frac, uint16_t col,
                    float minFrac, float maxFrac) {
  gfx->fillRoundRect(x, y, w, h, h / 2, COL_TRACK);
  int fw = (int)(w * frac);
  if (fw < 0) fw = 0;
  if (fw > w) fw = w;
  if (fw >= h)      gfx->fillRoundRect(x, y, fw, h, h / 2, col);
  else if (fw > 0)  gfx->fillRect(x, y, fw, h, col);
  gfx->drawFastVLine(x + (int)(w * minFrac), y - 2, h + 4, COL_DIM);   // marca MIN
  gfx->drawFastVLine(x + (int)(w * maxFrac), y - 2, h + 4, COL_DIM);   // marca MAX
}

// "Chip" de estado no cabeçalho (preenchido; nos alertas pisca entre cheio e vazado)
static void drawChip(int state, bool blinkOn) {
  const char *txt; uint16_t col; bool blink = false;
  switch (state) {
    case CHIP_INIT:    txt = "INICIANDO";    col = COL_DIM;    break;
    case CHIP_FAULT:   txt = "FALHA SENSOR"; col = COL_RED;    blink = true; break;
    case CHIP_ALERT:   txt = "ALERTA NIVEL"; col = COL_RED;    blink = true; break;
    case CHIP_NOENTRY: txt = "SEM ENTRADA";  col = COL_PURPLE; blink = true; break;
    case CHIP_NOFLOW:  txt = "SEM FLUXO";    col = COL_PURPLE; blink = true; break;
    case CHIP_FLOAT:   txt = "BOIA ATIVADA"; col = COL_CYAN;   break;
    case CHIP_FILLING: txt = "ENCHENDO";     col = COL_AMBER;  break;
    default:           txt = "NORMAL";       col = COL_GREEN;  break;
  }
  bool filled = !(blink && !blinkOn);
  uint16_t fillc = filled ? col : COL_HEADER;
  uint16_t txtc  = filled ? COL_BG : col;
  gfx->fillRoundRect(CHIP_X, 4, CHIP_W, 20, 10, fillc);
  gfx->drawRoundRect(CHIP_X, 4, CHIP_W, 20, 10, col);
  int w = (int)strlen(txt) * 12;                      // fonte tamanho 2 = 12 px por letra
  gfx->setTextSize(2);
  gfx->setTextColor(txtc);
  gfx->setCursor(CHIP_X + (CHIP_W - w) / 2, 6);
  gfx->print(txt);
}

// Ícone de rede no cabeçalho: 4 barras de sinal. Cor: verde = MQTT ok,
// âmbar = só Wi-Fi, vermelho (1ª barra) = sem Wi-Fi.
static void drawNetIcon(int st, int bars) {
  uint16_t col = (st == 2) ? COL_GREEN : (st == 1 ? COL_AMBER : COL_RED);
  gfx->fillRect(396, 4, 30, 22, COL_HEADER);
  for (int i = 0; i < 4; i++) {
    int h = 5 + i * 4;
    uint16_t c = (st == 0) ? ((i == 0) ? COL_RED : COL_TRACK) : ((i < bars) ? col : COL_TRACK);
    gfx->fillRect(400 + i * 6, 24 - h, 4, h, c);
  }
}

// Desenha tudo o que NÃO muda: fundo, cabeçalho, moldura do tanque e cartões
static void displayDrawStatic() {
  gfx->fillScreen(COL_BG);

  // --- Cabeçalho: gota d'água + título ---
  gfx->fillRect(0, 0, 536, 28, COL_HEADER);
  gfx->drawFastHLine(0, 28, 536, COL_EDGE);
  gfx->fillTriangle(22, 4, 14, 17, 30, 17, COL_BLUE);
  gfx->fillCircle(22, 18, 8, COL_BLUE);
  gfx->fillCircle(19, 16, 2, C565(200, 235, 255));    // brilho da gota
  drawLabel(40, 6, 2, COL_TEXT, "RESERVA DE AGUA");

  // --- Moldura do tanque (a cor da borda é atualizada conforme o estado) ---
  gfx->fillRoundRect(TANK_X, TANK_Y, TANK_W, TANK_H, 10, COL_TANKBG);

  // --- Cartões ---
  drawCardFrame(CA_X, CR1_Y, "NIVEL DA AGUA", COL_BLUE);
  drawLabel(CA_X + 12 + 104, CR1_Y + 42, 2, COL_DIM, "cm");

  drawCardFrame(CB_X, CR1_Y, "FLUXO ENTRADA / SAIDA", COL_CYAN);

  drawCardFrame(CA_X, CR2_Y, "BOIA DE NIVEL", COL_AMBER);
  drawCardFrame(CB_X, CR2_Y, "RELE 1", COL_GREEN);
}

// Bolhas do tanque
struct Bubble { float x, y, spd; uint8_t r; };

// Task do display: ~25 fps. Anima o tanque e atualiza os cartões quando há mudança.
static void displayTask(void *arg) {
  displayDrawStatic();

  float phase = 0.0f;           // fase da onda
  float shownH = 0.0f;          // altura exibida (suavizada) em cm
  int   lastBarPx = -1;         // cache da barra de nível
  uint16_t lastBarCol = 0;
  int   lastChip = -1;          // cache do chip
  bool  lastBlink = false;
  int   lastNetSt = -1;         // cache do ícone de rede
  int   lastBars = -1;
  int   lastFloatSt = -1;       // cache dos indicadores circulares
  int   lastRelaySt = -1;
  uint16_t lastEdge = 0;        // cache da borda do tanque
  int   marqueeOff = 0;         // deslocamento da animação de fluxo
  int64_t lastLevelTs = 0;      // p/ contar falhas consecutivas (igual aos LEDs)
  int   failCount = 0;

  Bubble bub[5];
  for (int i = 0; i < 5; i++) {
    bub[i].x   = 10 + random(CAN_W - 20);
    bub[i].y   = random(CAN_H);
    bub[i].spd = 0.4f + random(0, 60) / 100.0f;
    bub[i].r   = 2 + random(0, 3);
  }

  const float pxPerCm = (float)CAN_H / SENSOR_MOUNT_HEIGHT_CM;   // escala vertical do tanque
  const int   yMax = CAN_H - (int)(LEVEL_MAX_CM * pxPerCm);      // linha do nível máximo
  const int   yMin = CAN_H - (int)(LEVEL_MIN_CM * pxPerCm);      // linha do nível mínimo

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(DISPLAY_FRAME_MS));
    uint32_t nowMs = millis();
    bool blinkOn = ((nowMs / 500) % 2) == 0;

    // ------------------------------------------------------------
    // 1) Lê os dados (do hub e dos estados compartilhados)
    // ------------------------------------------------------------
    SensorReading rl, rd, rf, rv, rf2, rv2;
    hubGet(levelId, rl);   // altura da água
    hubGet(distId,  rd);   // distância medida
    hubGet(flowId,  rf);   // vazão de entrada
    hubGet(volId,   rv);   // volume de entrada
    hubGet(flow2Id, rf2);  // vazão de saída
    hubGet(vol2Id,  rv2);  // volume de saída
    bool fa = floatActive, ro = relay1On, al = levelAlert;
    int  is = inflowState;

    // Conta falhas consecutivas do ultrassônico (mesma regra da barra de LEDs)
    if (rl.timestamp_us != 0 && rl.timestamp_us != lastLevelTs) {
      lastLevelTs = rl.timestamp_us;
      if (rl.status == SENSOR_OK) failCount = 0; else failCount++;
    }
    bool levelOk = (rl.status == SENSOR_OK);
    bool fault   = (failCount >= LED_FAIL_THRESHOLD);
    float h = rl.value;
    if (h < 0) h = 0;
    if (h > SENSOR_MOUNT_HEIGHT_CM) h = SENSOR_MOUNT_HEIGHT_CM;

    // ------------------------------------------------------------
    // 2) Estado geral -> chip do cabeçalho e cores do tanque
    // ------------------------------------------------------------
    int chip;
    if (rl.status == SENSOR_NO_DATA)       chip = CHIP_INIT;
    else if (fault)                        chip = CHIP_FAULT;
    else if (al)                           chip = CHIP_ALERT;
    else if (is == INFLOW_STATE_NOENTRY)   chip = CHIP_NOENTRY;
    else if (is == INFLOW_STATE_NOFLOW)    chip = CHIP_NOFLOW;
    else if (fa)                           chip = CHIP_FLOAT;
    else if (ro)                           chip = CHIP_FILLING;
    else                                   chip = CHIP_NORMAL;

    uint16_t waterLight, waterDark, edge;
    if (al) {            waterLight = C565(255, 130, 130); waterDark = C565(150, 30, 40);  edge = COL_RED;   }
    else if (is != INFLOW_STATE_OK) {
                         waterLight = C565(215, 160, 255); waterDark = C565(90, 40, 140);  edge = COL_PURPLE; }
    else if (fa) {       waterLight = C565(120, 242, 232); waterDark = C565(0, 110, 125);  edge = COL_CYAN;  }
    else if (levelOk && h <= LEVEL_MIN_CM) {
                         waterLight = C565(255, 214, 130); waterDark = C565(140, 85, 10);  edge = COL_AMBER; }
    else {               waterLight = C565(120, 195, 255); waterDark = C565(20, 78, 168);  edge = COL_BLUE;  }

    // Chip: redesenha só quando muda (ou a cada piscada, nos alertas)
    if (chip != lastChip || blinkOn != lastBlink) {
      drawChip(chip, blinkOn);
      lastChip = chip;
      lastBlink = blinkOn;
    }

    // Ícone de rede: 0 = sem Wi-Fi, 1 = só Wi-Fi, 2 = Wi-Fi + MQTT
    int netSt = !wifiUp ? 0 : (!mqttUp ? 1 : 2);
    int bars = 0;
    if (wifiUp) {
      int rs = wifiRssi;
      bars = (rs > -55) ? 4 : (rs > -65) ? 3 : (rs > -75) ? 2 : 1;
    }
    if (netSt != lastNetSt || bars != lastBars) {
      drawNetIcon(netSt, bars);
      lastNetSt = netSt;
      lastBars = bars;
    }

    // Tempo ligado (hh:mm:ss) no canto direito do cabeçalho
    char buf[48];
    unsigned long secs = nowMs / 1000;
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", secs / 3600, (secs / 60) % 60, secs % 60);
    drawText(fUptime, 428, 6, 100, 16, 2, COL_DIM, COL_HEADER, buf);

    // ------------------------------------------------------------
    // 3) Tanque animado (canvas): água suavizada + onda + bolhas + marcas
    // ------------------------------------------------------------
    if (edge != lastEdge) {                       // borda do tanque na cor do estado
      gfx->drawRoundRect(TANK_X, TANK_Y, TANK_W, TANK_H, 10, edge);
      gfx->drawRoundRect(TANK_X + 1, TANK_Y + 1, TANK_W - 2, TANK_H - 2, 9, edge);
      lastEdge = edge;
    }

    shownH += (h - shownH) * 0.10f;               // sobe/desce suavemente até o valor real
    phase  += 0.22f;
    int surfaceY = CAN_H - (int)(shownH * pxPerCm);
    float amp = 3.0f * ((shownH < 2.0f) ? (shownH / 2.0f) : 1.0f);   // onda some quando vazio

    tankCv->fillScreen(COL_TANKBG);
    for (int x = 0; x < CAN_W; x++) {
      int y = surfaceY + (int)(amp * sinf(phase + x * 0.09f) + 0.5f * amp * sinf(phase * 1.7f + x * 0.21f));
      if (y < 0) y = 0;
      if (y >= CAN_H || shownH < 0.15f) continue;
      tankCv->drawFastVLine(x, y, 3, waterLight);                       // crista clara
      if (y + 3 < CAN_H)  tankCv->drawFastVLine(x, y + 3, 14, (uint16_t)(((waterLight & 0xF7DE) >> 1) + ((waterDark & 0xF7DE) >> 1)));  // degradê
      if (y + 17 < CAN_H) tankCv->drawFastVLine(x, y + 17, CAN_H - y - 17, waterDark);   // corpo escuro
    }

    // Bolhas subindo
    if (shownH > 3.0f) {
      for (int i = 0; i < 5; i++) {
        bub[i].y -= bub[i].spd;
        if (bub[i].y < surfaceY + 6) {            // chegou à superfície: renasce no fundo
          bub[i].y = CAN_H - 6;
          bub[i].x = 10 + random(CAN_W - 20);
        }
        tankCv->drawCircle((int)bub[i].x, (int)bub[i].y, bub[i].r, waterLight);
      }
    }

    // Marcas de nível MIN e MAX (tracejadas) com rótulos
    char lbl[16];
    for (int x = 0; x < CAN_W; x += 8) {
      tankCv->drawFastHLine(x, yMax, 4, COL_TEXT);
      tankCv->drawFastHLine(x, yMin, 4, COL_TEXT);
    }
    tankCv->setTextSize(1);
    tankCv->setTextColor(COL_TEXT);
    snprintf(lbl, sizeof(lbl), "MAX %.0f", (double)LEVEL_MAX_CM);
    tankCv->setCursor(4, yMax - 10);
    tankCv->print(lbl);
    snprintf(lbl, sizeof(lbl), "MIN %.0f", (double)LEVEL_MIN_CM);
    tankCv->setCursor(4, yMin - 10);
    tankCv->print(lbl);

    // Bolinha da boia na linha do máximo (cheia quando ativada)
    if (fa) tankCv->fillCircle(CAN_W - 14, yMax, 7, COL_AMBER);
    else    tankCv->drawCircle(CAN_W - 14, yMax, 7, COL_DIM);

    tankCv->flush();                              // envia o quadro ao display

    // ------------------------------------------------------------
    // 4) Cartão NIVEL DA AGUA
    // ------------------------------------------------------------
    uint16_t levelCol = al ? COL_RED : (fa ? COL_CYAN : COL_TEXT);
    if (levelOk) snprintf(buf, sizeof(buf), "%.1f", h); else snprintf(buf, sizeof(buf), "--.-");
    drawText(fLevel, CA_X + 12, CR1_Y + 26, 100, 32, 4, levelCol, COL_CARD, buf);

    if (rd.status == SENSOR_OK) snprintf(buf, sizeof(buf), "dist. sensor %.1f cm", rd.value);
    else                        snprintf(buf, sizeof(buf), "sem leitura (%s)", statusStr(rd.status));
    drawText(fLevelSub, CA_X + 12, CR1_Y + 64, 164, 8, 1, COL_DIM, COL_CARD, buf);

    int barPx = levelOk ? (int)(160.0f * h / SENSOR_MOUNT_HEIGHT_CM) : 0;
    if (barPx != lastBarPx || edge != lastBarCol) {
      drawBar(CA_X + 12, CR1_Y + 78, 160, 8, levelOk ? h / SENSOR_MOUNT_HEIGHT_CM : 0.0f, edge,
              LEVEL_MIN_CM / SENSOR_MOUNT_HEIGHT_CM, LEVEL_MAX_CM / SENSOR_MOUNT_HEIGHT_CM);
      lastBarPx = barPx;
      lastBarCol = edge;
    }

    // ------------------------------------------------------------
    // 5) Cartão FLUXO: entrada (E) e saída (S), totais e "gotas" correndo
    // ------------------------------------------------------------
    float flow  = (rf.status  == SENSOR_OK) ? rf.value  : 0.0f;   // entrada
    float flow2 = (rf2.status == SENSOR_OK) ? rf2.value : 0.0f;   // saída
    float flowMax = (flow > flow2) ? flow : flow2;

    snprintf(buf, sizeof(buf), "E %.1f L/m", flow);
    drawText(fFlow,  CB_X + 12, CR1_Y + 24, 164, 16, 2, flow  > 0.05f ? COL_CYAN  : COL_DIM, COL_CARD, buf);
    snprintf(buf, sizeof(buf), "S %.1f L/m", flow2);
    drawText(fFlow2, CB_X + 12, CR1_Y + 44, 164, 16, 2, flow2 > 0.05f ? COL_AMBER : COL_DIM, COL_CARD, buf);

    snprintf(buf, sizeof(buf), "E %.2f L   S %.2f L", rv.value, rv2.value);
    drawText(fFlowSub, CB_X + 12, CR1_Y + 64, 164, 8, 1, COL_DIM, COL_CARD, buf);

    // Animação: pontos que correm, mais rápido com mais vazão (usa a maior das duas)
    gfx->fillRect(CB_X + 12, CR1_Y + 76, 164, 10, COL_CARD);
    if (flowMax > 0.05f) marqueeOff = (marqueeOff + 1 + (int)(flowMax / 3.0f)) % 16;
    for (int i = 0; i < 10; i++) {
      int px = CB_X + 12 + i * 16 + marqueeOff;
      if (px > CB_X + 12 + 158) continue;
      gfx->fillCircle(px + 2, CR1_Y + 81, (flowMax > 0.05f) ? 3 : 2, (flowMax > 0.05f) ? COL_CYAN : COL_TRACK);
    }

    // ------------------------------------------------------------
    // 6) Cartão BOIA DE NIVEL
    // ------------------------------------------------------------
    if (lastFloatSt != (int)fa) {                 // indicador circular só quando muda
      uint16_t c = fa ? COL_AMBER : COL_GREEN;
      gfx->fillCircle(CA_X + 30, CR2_Y + 36, 20, COL_CARD);
      gfx->drawCircle(CA_X + 30, CR2_Y + 36, 18, c);
      gfx->fillCircle(CA_X + 30, CR2_Y + 36, 12, c);
      lastFloatSt = (int)fa;
    }
    drawText(fFloatTxt, CA_X + 58, CR2_Y + 28, 120, 16, 2, fa ? COL_AMBER : COL_GREEN, COL_CARD,
             fa ? "ATIVADA" : "LIVRE");
    drawText(fFloatSub, CA_X + 58, CR2_Y + 50, 120, 8, 1, COL_DIM, COL_CARD,
             fa ? "nivel maximo atingido" : "abaixo do maximo");
    snprintf(buf, sizeof(buf), "GPIO%d nivel=%d", FLOAT_GPIO, digitalRead(FLOAT_GPIO));
    drawText(fFloatRaw, CA_X + 12, CR2_Y + 76, 164, 8, 1, COL_DIM, COL_CARD, buf);

    // ------------------------------------------------------------
    // 7) Cartão RELE 1
    // ------------------------------------------------------------
    if (lastRelaySt != (int)ro) {
      uint16_t c = ro ? COL_GREEN : COL_DIM;
      gfx->fillCircle(CB_X + 30, CR2_Y + 36, 20, COL_CARD);
      gfx->drawCircle(CB_X + 30, CR2_Y + 36, 18, c);
      if (ro) gfx->fillCircle(CB_X + 30, CR2_Y + 36, 12, c);
      else    gfx->drawCircle(CB_X + 30, CR2_Y + 36, 12, c);
      lastRelaySt = (int)ro;
    }
    drawText(fRelayTxt, CB_X + 58, CR2_Y + 28, 120, 16, 2, ro ? COL_GREEN : COL_DIM, COL_CARD,
             ro ? "LIGADO" : "DESLIGADO");
    snprintf(buf, sizeof(buf), "motivo: %s", relay1Reason);
    drawText(fRelaySub, CB_X + 12, CR2_Y + 62, 164, 8, 1, COL_DIM, COL_CARD, buf);
    snprintf(buf, sizeof(buf), "GPIO%d nivel=%d", RELAY1_GPIO, digitalRead(RELAY1_GPIO));
    drawText(fRelayRaw, CB_X + 12, CR2_Y + 76, 164, 8, 1, COL_DIM, COL_CARD, buf);
  }
}

// Liga o painel, inicializa o display e o canvas do tanque e cria a task gráfica
static bool displayStart() {
  pinMode(DISPLAY_ENABLE_GPIO, OUTPUT);
  digitalWrite(DISPLAY_ENABLE_GPIO, HIGH);          // alimenta o painel AMOLED
  delay(20);

  dispBus = new Arduino_ESP32QSPI(DISPLAY_CS_GPIO, DISPLAY_SCK_GPIO, DISPLAY_D0_GPIO,
                                  DISPLAY_D1_GPIO, DISPLAY_D2_GPIO, DISPLAY_D3_GPIO);
  gfx = new Arduino_RM67162(dispBus, DISPLAY_RST_GPIO, 0 /* rotação inicial */);

  if (!gfx->begin()) {
    Serial.println("ERRO: display AMOLED nao respondeu");
    return false;
  }
  gfx->setRotation(DISPLAY_ROTATION);

  // Brilho do painel (comando 0x51 do RM67162)
  dispBus->beginWrite();
  dispBus->writeC8D8(0x51, DISPLAY_BRIGHTNESS);
  dispBus->endWrite();

  // Canvas do tanque: buffer pequeno (~43 KB) na RAM; não depende de PSRAM
  tankCv = new Arduino_Canvas(CAN_W, CAN_H, gfx, TANK_X + 4, TANK_Y + 4);
  if (!tankCv->begin(GFX_SKIP_OUTPUT_BEGIN)) {
    Serial.println("ERRO: sem memoria para o canvas do tanque");
    return false;
  }

  if (xTaskCreate(displayTask, "display", DISPLAY_TASK_STACK, NULL,
                  DISPLAY_TASK_PRIO, NULL) != pdPASS) return false;

  DBG("Display iniciado: RM67162 QSPI, rotacao=%d, brilho=%d", DISPLAY_ROTATION, DISPLAY_BRIGHTNESS);
  return true;
}

// =====================================================================
//  BUZZER
// =====================================================================

// Emite um beep de 'ms' milissegundos (bloqueia só a task que chamou)
static void buzzerBeep(uint32_t ms) {
#if BUZZER_ACTIVE
  digitalWrite(BUZZER_GPIO, HIGH);
  vTaskDelay(pdMS_TO_TICKS(ms));
  digitalWrite(BUZZER_GPIO, LOW);
#else
  ledcWriteTone(BUZZER_GPIO, BUZZER_FREQ_HZ);     // liga o tom
  vTaskDelay(pdMS_TO_TICKS(ms));
  ledcWriteTone(BUZZER_GPIO, 0);                  // desliga o tom
#endif
}

// Beep DUPLO longo: dois beeps longos separados por uma pausa (boia ativada)
static void buzzerDoubleLong() {
  buzzerBeep(BUZZER_LONG_BEEP_MS);
  vTaskDelay(pdMS_TO_TICKS(BUZZER_DOUBLE_GAP_MS));
  buzzerBeep(BUZZER_LONG_BEEP_MS);
}

// Rajada de beeps de ALERTA (nível máximo pelo ultrassônico)
static void buzzerAlertBurst() {
  for (int i = 0; i < ALERT_BEEP_COUNT; i++) {
    buzzerBeep(ALERT_BEEP_ON_MS);
    if (i < ALERT_BEEP_COUNT - 1) vTaskDelay(pdMS_TO_TICKS(ALERT_BEEP_OFF_MS));
  }
}

// Aviso de entrada: 'n' beeps médios (2 = sem fluxo/suspeita, 4 = sem entrada/confirmado)
static void buzzerNoInflow(int n) {
  for (int i = 0; i < n; i++) {
    buzzerBeep(NOINFLOW_BEEP_ON_MS);
    if (i < n - 1) vTaskDelay(pdMS_TO_TICKS(NOINFLOW_BEEP_OFF_MS));
  }
}

// Configura o pino do buzzer (PWM se passivo, saída digital se ativo)
static bool buzzerInit() {
#if BUZZER_ACTIVE
  pinMode(BUZZER_GPIO, OUTPUT);
  digitalWrite(BUZZER_GPIO, LOW);
  return true;
#else
  if (!ledcAttach(BUZZER_GPIO, BUZZER_FREQ_HZ, 8)) return false;
  ledcWriteTone(BUZZER_GPIO, 0);
  return true;
#endif
}

// =====================================================================
//  RELÉ (LIGADO = bobina energizada | DESLIGADO = bobina sem energia)
// =====================================================================

// Liga (true) ou desliga (false) o canal:
//   true  -> pino no nível que ENERGIZA a bobina (LED do canal aceso)
//   false -> pino no nível oposto, bobina SEM energia (LED apagado)
static void relaySet(int gpio, bool on) {
  digitalWrite(gpio, on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
}

// Configura os pinos dos relés e garante o estado inicial DESLIGADO
static void relayInit() {
  pinMode(RELAY1_GPIO, OUTPUT);
  relaySet(RELAY1_GPIO, false);            // canal 1 inicia DESLIGADO (bobina sem energia)
  relay1On = false;
#if RELAY2_ENABLE
  pinMode(RELAY2_GPIO, OUTPUT);
  relaySet(RELAY2_GPIO, false);            // canal 2 inicia DESLIGADO (ainda sem uso na lógica)
#endif
  DBG("Rele iniciado: canal1=GPIO%d (desligado)%s", RELAY1_GPIO,
      RELAY2_ENABLE ? ", canal2 habilitado" : ", canal2 desabilitado");
}

// =====================================================================
//  TASK DE CONTROLE: relé 1 + alerta + monitor de entrada + beeps
// =====================================================================
// Roda a cada 100 ms. Variável de decisão: wantOn = "o relé deve estar LIGADO".
//
//   BOIA ativou                -> wantOn = false, alerta cancelado, ultrassônico ignorado
//   BOIA desligou              -> nada muda: relé continua DESLIGADO até altura <= mínimo
//   altura >= LEVEL_MAX (boia inativa) -> wantOn = false + ALERTA (backup da boia)
//   altura <= LEVEL_MIN (boia inativa) -> wantOn = true  (liga o relé)
//   entre os limites           -> mantém o último estado
//
//   Relé real = wantOn E boia inativa
//
// Monitor de entrada (só avisa, não mexe no relé):
//   relé LIGADO e sem vazão de entrada por NOINFLOW_GRACE_MS -> INFLOW_STATE_NOFLOW
//   ao fim da janela NOINFLOW_LEVEL_WINDOW_MS, nível subiu < NOINFLOW_LEVEL_RISE_CM
//                                                           -> INFLOW_STATE_NOENTRY
//   vazão voltou ou relé desligou -> INFLOW_STATE_OK
//
// Sons (um por ciclo, nesta prioridade):
//   1) boia acabou de ativar            -> beep DUPLO longo
//   2) alerta de nível ativo            -> rajada de beeps a cada ALERT_REPEAT_MS
//   3) sem fluxo / sem entrada          -> 2 ou 4 beeps a cada NOINFLOW_BEEP_REPEAT_MS
//   4) variação de BUZZER_STEP_CM (ultrassônico) -> beep curto
static void controlTask(void *arg) {
  int64_t  lastTs          = 0;       // timestamp da última leitura já processada
  bool     haveRef         = false;   // já existe altura de referência para o beep de variação?
  float    refCm           = 0;       // altura de referência do último beep
  bool     wantOn          = false;   // relé deve estar ligado? (começa DESLIGADO)
  bool     prevFloat       = false;   // estado anterior da boia (detecta bordas)
  uint32_t lastAlertBeepMs = 0;       // instante da última rajada de alerta
  const char *reason       = "INICIAL";

  // --- variáveis do monitor de entrada ---
  uint32_t noFlowSince     = 0;       // início de "relé ligado sem vazão" (0 = fora dessa condição)
  uint32_t winStartMs      = 0;       // início da janela atual de confirmação pelo nível
  float    winRefCm        = 0;       // altura de referência no início da janela
  bool     winRefValid     = false;   // a referência já foi capturada?
  uint32_t lastInflowBeepMs = 0;      // instante do último aviso sonoro de entrada

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));

    bool beepDouble = false, beepShort = false;
    bool fa = floatActive;            // cópia local do estado da boia neste ciclo

    // -----------------------------------------------------------------
    // 1) BOIA: detecta mudança de estado
    // -----------------------------------------------------------------
    if (fa != prevFloat) {
      prevFloat = fa;
      if (fa) {
        // Boia ATIVOU: nível máximo atingido. Desliga o relé, cancela o alerta
        // do ultrassônico e passa a ignorar o ultrassônico até a boia desligar.
        wantOn = false;
        reason = "BOIA";
        if (levelAlert) levelAlert = false;
        beepDouble = true;
        Serial.println("[CONTROLE] BOIA ATIVADA: nivel maximo. Rele1 DESLIGA e o ultrassonico e IGNORADO");
      } else {
        // Boia DESLIGOU: a água baixou. O relé permanece DESLIGADO e só volta a
        // ligar quando o ultrassônico indicar altura <= mínimo.
        Serial.printf("[CONTROLE] BOIA DESLIGOU: rele1 segue DESLIGADO ate a altura ser <= %.1f cm\n",
                      (double)LEVEL_MIN_CM);
      }
    }

    // -----------------------------------------------------------------
    // 2) ULTRASSÔNICO: processa somente leituras NOVAS (altura da água)
    // -----------------------------------------------------------------
    SensorReading r;
    if (hubGet(levelId, r) && r.timestamp_us != 0 && r.timestamp_us != lastTs) {
      lastTs = r.timestamp_us;

      if (r.status != SENSOR_OK) {
        // Leitura inválida (timeout / fora da faixa): mantém tudo como está
        DBG("Controle: leitura invalida (%s), estado mantido (rele %s, alerta %s)",
            statusStr(r.status), wantOn ? "LIGADO" : "DESLIGADO", levelAlert ? "ATIVO" : "inativo");
      } else {
        float h = r.value;            // altura da água em cm

        if (fa) {
          // Boia ativa: ações do ultrassônico DESCONSIDERADAS. Só acompanha a
          // referência em silêncio para não gerar beep ao voltar a valer.
          refCm = h;
          haveRef = true;
          DBG("Controle: altura %.2f cm IGNORADA (boia ativada)", h);
          if (h <= LEVEL_MIN_CM) {
            Serial.printf("[AVISO] Boia ativada mas a altura medida e %.2f cm (<= %.1f): boia presa ou sensor errado?\n",
                          h, (double)LEVEL_MIN_CM);
          }
        } else {
          // ---- Boia INATIVA: o ultrassônico comanda ----

          if (h >= LEVEL_MAX_CM) {
            // Nível máximo pelo ultrassônico SEM a boia ativada = falha da boia.
            if (!levelAlert) {
              levelAlert = true;
              lastAlertBeepMs = 0;    // força a primeira rajada imediatamente
              Serial.printf("[ALERTA] Altura %.2f cm >= %.1f cm com a boia NAO ativada: possivel falha da boia!\n",
                            h, (double)LEVEL_MAX_CM);
            }
            if (wantOn) {             // backup: desliga o relé
              wantOn = false;
              reason = "ULTRASSONICO_MAX";
            }
          } else {
            // Abaixo do máximo: encerra o alerta (com histerese opcional)
            if (levelAlert && h < (LEVEL_MAX_CM - ALERT_HYST_CM)) {
              levelAlert = false;
              Serial.printf("[ALERTA] Encerrado: altura %.2f cm abaixo do maximo\n", h);
            }
            // Altura mínima atingida: liga o relé
            if (h <= LEVEL_MIN_CM && !wantOn) {
              wantOn = true;
              reason = "ULTRASSONICO_MIN";
              Serial.printf("[CONTROLE] Altura %.2f cm <= %.1f cm: rele1 LIGA\n",
                            h, (double)LEVEL_MIN_CM);
            }
          }

          // Beep curto a cada BUZZER_STEP_CM de variação em relação à última referência
          if (!haveRef) {
            refCm = h;
            haveRef = true;
            DBG("Buzzer: referencia inicial = %.2f cm", refCm);
          } else if (fabsf(h - refCm) >= BUZZER_STEP_CM) {
            beepShort = true;
            refCm = h;                // nova referência
          }
        }
      }
    }

    // -----------------------------------------------------------------
    // 3) RELÉ 1: ligado somente se wantOn e a boia NÃO estiver ativa
    // -----------------------------------------------------------------
    bool desiredOn = wantOn && !fa;
    if (desiredOn != relay1On) {                     // só mexe no pino se o estado mudou
      relaySet(RELAY1_GPIO, desiredOn);
      relay1Reason = reason;                         // motivo primeiro, p/ o MQTT ler consistente
      relay1On = desiredOn;
      Serial.printf("[RELE1] %s (motivo: %s)\n",
                    desiredOn ? "LIGADO - bobina energizada, LED aceso"
                              : "DESLIGADO - bobina sem energia, LED apagado",
                    reason);
    }

    // -----------------------------------------------------------------
    // 4) MONITOR DE ENTRADA: relé ligado e sem vazão de entrada (só avisa)
    // -----------------------------------------------------------------
    uint32_t nowMs = millis();
    bool noFlowCond = relay1On && !flowIn.flowing;   // "enchendo" mas sem água passando no sensor

    if (!noFlowCond) {
      // Condição encerrada (vazão voltou ou relé desligou): normaliza tudo
      if (inflowState != INFLOW_STATE_OK) {
        Serial.printf("[ENTRADA] Normalizada (%s): %s\n",
                      relay1On ? "vazao de entrada voltou" : "rele desligado",
                      inflowStr(INFLOW_STATE_OK));
        inflowState = INFLOW_STATE_OK;
      }
      noFlowSince = 0;
      winRefValid = false;
    } else {
      // Início da condição: marca o tempo e abre a primeira janela de confirmação
      if (noFlowSince == 0) {
        noFlowSince = nowMs;
        winStartMs  = nowMs;
        winRefValid = false;
        DBG("Entrada: rele ligado sem vazao, tolerancia de %d s iniciada", NOINFLOW_GRACE_MS / 1000);
      }

      // Referência do nível para a janela (primeira leitura válida da janela)
      SensorReading lv;
      hubGet(levelId, lv);
      if (!winRefValid && lv.status == SENSOR_OK) {
        winRefCm = lv.value;
        winRefValid = true;
      }

      // (a) Tolerância esgotada: alerta de "sem fluxo" (suspeita)
      if (inflowState == INFLOW_STATE_OK && (nowMs - noFlowSince) >= NOINFLOW_GRACE_MS) {
        inflowState = INFLOW_STATE_NOFLOW;
        lastInflowBeepMs = 0;                        // força o primeiro aviso sonoro agora
        Serial.printf("[ENTRADA] ALERTA: rele ligado ha %lu s sem vazao de entrada. Confirmando pelo nivel...\n",
                      (unsigned long)((nowMs - noFlowSince) / 1000));
      }

      // (b) Fim da janela de confirmação: o nível subiu o suficiente?
      if ((nowMs - winStartMs) >= NOINFLOW_LEVEL_WINDOW_MS) {
        if (inflowState != INFLOW_STATE_OK && winRefValid && lv.status == SENSOR_OK) {
          float rise = lv.value - winRefCm;
          if (rise < NOINFLOW_LEVEL_RISE_CM) {
            if (inflowState != INFLOW_STATE_NOENTRY) {
              inflowState = INFLOW_STATE_NOENTRY;
              lastInflowBeepMs = 0;
              Serial.printf("[ENTRADA] CONFIRMADO: nivel subiu so %.2f cm em %d min (< %.1f cm). Agua NAO esta entrando!\n",
                            rise, NOINFLOW_LEVEL_WINDOW_MS / 60000, (double)NOINFLOW_LEVEL_RISE_CM);
            }
          } else {
            if (inflowState == INFLOW_STATE_NOENTRY) inflowState = INFLOW_STATE_NOFLOW;
            Serial.printf("[ENTRADA] Nivel subiu %.2f cm em %d min: a agua entra, sensor de fluxo de entrada suspeito\n",
                          rise, NOINFLOW_LEVEL_WINDOW_MS / 60000);
          }
        }
        winStartMs  = nowMs;                         // abre nova janela
        winRefValid = false;                         // a próxima leitura válida vira a referência
      }
    }

    // -----------------------------------------------------------------
    // 5) SONS (o relé já foi atualizado antes de tocar)
    // -----------------------------------------------------------------
    int is = inflowState;
    if (beepDouble) {
      DBG("Buzzer: BEEP DUPLO LONGO (boia ativada)");
      buzzerDoubleLong();
    } else if (levelAlert && (lastAlertBeepMs == 0 || (nowMs - lastAlertBeepMs) >= ALERT_REPEAT_MS)) {
      DBG("Buzzer: RAJADA DE ALERTA (nivel maximo pelo ultrassonico)");
      buzzerAlertBurst();
      lastAlertBeepMs = millis();
    } else if (is != INFLOW_STATE_OK && (lastInflowBeepMs == 0 || (nowMs - lastInflowBeepMs) >= NOINFLOW_BEEP_REPEAT_MS)) {
      DBG("Buzzer: AVISO DE ENTRADA (%s)", inflowStr(is));
      buzzerNoInflow(is == INFLOW_STATE_NOENTRY ? 4 : 2);
      lastInflowBeepMs = millis();
    } else if (beepShort && !levelAlert) {
      DBG("Buzzer: BEEP curto (variacao >= %.1f cm)", (double)BUZZER_STEP_CM);
      buzzerBeep(BUZZER_BEEP_MS);
    }
  }
}

// Inicializa relé e buzzer e cria a task de controle
static bool controlStart() {
  relayInit();
  if (!buzzerInit()) return false;

  if (xTaskCreate(controlTask, "control", CONTROL_TASK_STACK, NULL,
                  CONTROL_TASK_PRIO, NULL) != pdPASS) return false;

  DBG("Controle iniciado: altura min=%.1f cm / max=%.1f cm, buzzer GPIO%d (%s)",
      (double)LEVEL_MIN_CM, (double)LEVEL_MAX_CM, BUZZER_GPIO, BUZZER_ACTIVE ? "ativo" : "passivo");
  return true;
}

// =====================================================================
//  REDE: Wi-Fi + MQTT (TLS) em task própria
// =====================================================================
//  Tópicos (base = MQTT_TOPICO):
//    <base>/status      "online" / "offline"  (retido; "offline" via Last Will)
//    <base>/telemetria  JSON do contrato (periódico e a cada mudança, retido)
//    <base>/eventos     JSON apenas nas mudanças (boia, rele1, alerta, entrada, fluxos)
//
//  Esta task SÓ LÊ os dados (hub e estados). Nunca comanda o relé:
//  se a rede cair, o controle da água segue normalmente.
// =====================================================================
#if MQTT_USE_TLS
static WiFiClientSecure netClient;     // cliente TLS (valida o servidor com o certificado CA)
#else
static WiFiClient netClient;           // cliente sem criptografia
#endif
static PubSubClient mqtt(netClient);

static char topicStatus[112];
static char topicTele[112];
static char topicEvents[112];
static char mqttClientId[32];

// Monta os tópicos a partir de MQTT_TOPICO e define o ClientID (= deviceId)
static void netBuildTopics() {
  String base = MQTT_TOPICO;
  while (base.endsWith("/")) base.remove(base.length() - 1);   // remove "/" no final, se houver
  snprintf(topicStatus, sizeof(topicStatus), "%s/status", base.c_str());
  snprintf(topicTele,   sizeof(topicTele),   "%s%s", base.c_str(), MQTT_TELEMETRY_SUFFIX);
  snprintf(topicEvents, sizeof(topicEvents), "%s/eventos", base.c_str());
  snprintf(mqttClientId, sizeof(mqttClientId), "%s", DEVICE_ID);   // ClientID = deviceId
}

// Hora UNIX atual, ou 0 se o relógio ainda não foi sincronizado por NTP
static uint32_t netEpoch() {
  time_t t = time(nullptr);
  return (t > 1700000000) ? (uint32_t)t : 0;
}

// Formata um número para JSON: valor com 1 casa, ou "null" se inválido
static void fmtNum(char *out, size_t n, bool valid, float v) {
  if (valid) snprintf(out, n, "%.1f", v);
  else       snprintf(out, n, "null");
}

// Estado da caixa em texto minúsculo (formato combinado com a integração)
static const char *tankState(const SensorReading &rl) {
  if (rl.status == SENSOR_NO_DATA)          return "iniciando";
  if (rl.status != SENSOR_OK)               return "falha_sensor";
  if (levelAlert)                           return "alerta_nivel";      // ultrassônico no máximo e boia inativa
  if (inflowState == INFLOW_STATE_NOENTRY)  return "sem_entrada";       // relé ligado, sem vazão e nível parado
  if (inflowState == INFLOW_STATE_NOFLOW)   return "entrada_suspeita";  // relé ligado sem vazão (confirmando)
  if (noFlowIdle)                           return "sem_fluxo";         // nada entrando nem saindo há 60 s
  if (floatActive)                          return "cheia";             // boia ativada
  if (relay1On)                             return "enchendo";          // relé ligado
  return "normal";
}

// Publica a telemetria (JSON, retida) em <base>/telemetria
static bool netPublishTelemetry() {
  SensorReading rl, rf, rf2;
  hubGet(levelId, rl);   // altura da água
  hubGet(flowId,  rf);   // vazão de entrada (L/min)
  hubGet(flow2Id, rf2);  // vazão de saída (L/min)

  // --- Vazões em L/h (sensores medem L/min, então x60) ---
  bool inOk  = (rf.status  == SENSOR_OK);
  bool outOk = (rf2.status == SENSOR_OK);
  float inLh  = rf.value  * 60.0f;
  float outLh = rf2.value * 60.0f;

  // --- Média móvel da vazão de saída (só atualiza quando chega leitura nova) ---
  static float   outAvgLh  = 0;
  static bool    avgInit   = false;
  static int64_t lastOutTs = 0;
  if (outOk && rf2.timestamp_us != lastOutTs) {
    lastOutTs = rf2.timestamp_us;
    outAvgLh = avgInit ? (outAvgLh + OUTFLOW_AVG_ALPHA * (outLh - outAvgLh)) : outLh;
    avgInit = true;
  }

  // --- Volume útil (L): litros acima da altura mínima ---
  bool  volOk = (rl.status == SENSOR_OK);
  float volL  = (rl.value - LEVEL_MIN_CM) * TANK_LITERS_PER_CM;
  if (volL < 0) volL = 0;

  // --- Autonomia (h) = volume útil / vazão média de saída ---
  bool  autOk = volOk && outOk && avgInit && (outAvgLh >= AUTONOMY_MIN_FLOW_LH);
  float autH  = autOk ? (volL / outAvgLh) : 0;

  // --- Timestamp em UTC (2026-11-19T02:30:00Z), ou null se a hora ainda não sincronizou ---
  char ts[40] = "null";
  time_t t = time(nullptr);
  if (t > 1700000000) {
    struct tm tmUtc;
    gmtime_r(&t, &tmUtc);
    char iso[24];
    strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &tmUtc);
    snprintf(ts, sizeof(ts), "\"%s\"", iso);
  }

  char sVol[16], sAut[16], sIn[16], sOut[16];
  fmtNum(sVol, sizeof(sVol), volOk, volL);
  fmtNum(sAut, sizeof(sAut), autOk, autH);
  fmtNum(sIn,  sizeof(sIn),  inOk,  inLh);
  fmtNum(sOut, sizeof(sOut), outOk, outLh);

  char json[320];
  snprintf(json, sizeof(json),
           "{\"deviceId\":\"%s\",\"timestamp\":%s,\"volumeUtilL\":%s,\"autonomiaH\":%s,"
           "\"estado\":\"%s\",\"vazaoEntradaLh\":%s,\"vazaoSaidaLh\":%s}",
           DEVICE_ID, ts, sVol, sAut, tankState(rl), sIn, sOut);

  bool ok = mqtt.publish(topicTele, json, true);     // retida: quem assina depois já recebe o último valor
  DBG("MQTT telemetria %s: %s", ok ? "enviada" : "FALHOU", json);
  return ok;
}

// Publica um evento pontual (não retido) em <base>/eventos
static void netPublishEvent(const char *nome, const char *valor) {
  char json[160];
  snprintf(json, sizeof(json), "{\"ts\":%lu,\"evento\":\"%s\",\"valor\":\"%s\"}",
           (unsigned long)netEpoch(), nome, valor);
  mqtt.publish(topicEvents, json, false);
  Serial.printf("[MQTT] evento: %s = %s\n", nome, valor);
}

// Task de rede: conecta o Wi-Fi, sincroniza a hora, conecta o MQTT (com espera
// crescente entre tentativas), publica telemetria periódica e eventos de mudança.
static void netTask(void *arg) {
  uint32_t backoff     = 2000;          // espera entre tentativas de MQTT (dobra até 60 s)
  uint32_t lastTry     = 0;             // instante da última tentativa de MQTT
  uint32_t lastTele    = 0;             // instante da última telemetria
  uint32_t lastWifiTry = millis();      // instante da última tentativa de Wi-Fi
  uint32_t ntpStart    = 0;
  bool wasWifi = false, timeOk = false;
  bool pFloat = floatActive, pRelay = relay1On, pAlert = levelAlert;
  bool pFlow = flowIn.flowing, pFlow2 = flowOut.flowing;
  int  pInflow = inflowState;

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_SENHA);
  Serial.printf("[WIFI] conectando a '%s'...\n", WIFI_SSID);

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));

    // ---------------- Wi-Fi ----------------
    bool w = (WiFi.status() == WL_CONNECTED);
    if (w != wasWifi) {                                   // mudou o estado do Wi-Fi
      wasWifi = w;
      wifiUp = w;
      if (w) {
        Serial.printf("[WIFI] conectado: IP=%s RSSI=%d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
        configTime(TZ_OFFSET_S, 0, NTP_SERVER_1, NTP_SERVER_2);   // pede a hora (necessária p/ validar o TLS)
        ntpStart = millis();
        timeOk = false;
        backoff = 2000;
        lastTry = 0;
      } else {
        Serial.println("[WIFI] desconectado");
        mqttUp = false;
        if (mqtt.connected()) mqtt.disconnect();
        lastWifiTry = millis();
      }
    }
    if (!w) {
      if (millis() - lastWifiTry > WIFI_RETRY_MS) {       // não conectou: força nova tentativa
        lastWifiTry = millis();
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_SENHA);
        Serial.println("[WIFI] tentando de novo...");
      }
      continue;
    }
    wifiRssi = WiFi.RSSI();

    // ---------------- Hora (NTP) ----------------
    if (!timeOk) {
      if (netEpoch() > 0) {
        timeOk = true;
        Serial.println("[NTP] hora sincronizada");
      } else if (millis() - ntpStart > NTP_WAIT_MS) {
        timeOk = true;
        Serial.println("[NTP] sem sincronizacao: tentando MQTT mesmo assim");
      } else {
        continue;                                         // ainda esperando a hora
      }
    }

    // ---------------- MQTT: conexão ----------------
    if (!mqtt.connected()) {
      if (mqttUp) {
        mqttUp = false;
        Serial.println("[MQTT] desconectado");
      }
      if (lastTry == 0 || millis() - lastTry >= backoff) {
        lastTry = millis();
        Serial.printf("[MQTT] conectando a %s:%u ...\n", MQTT_HOST, (unsigned)MQTT_PORTA);
        // Last Will: o broker publica "offline" (retido) se este equipamento cair
        if (mqtt.connect(mqttClientId, MQTT_USUARIO, MQTT_SENHA, topicStatus, 1, true, "offline")) {
          mqttUp = true;
          backoff = 2000;
          mqtt.publish(topicStatus, "online", true);
          pFloat = floatActive; pRelay = relay1On; pAlert = levelAlert;
          pFlow = flowIn.flowing; pFlow2 = flowOut.flowing; pInflow = inflowState;
          netPublishTelemetry();                          // envia o estado atual logo ao conectar
          lastTele = millis();
          Serial.printf("[MQTT] conectado | topicos: %s | %s | %s\n", topicStatus, topicTele, topicEvents);
        } else {
          Serial.printf("[MQTT] falha rc=%d | nova tentativa em %lu s\n",
                        mqtt.state(), (unsigned long)(backoff / 1000));
          backoff = (backoff * 2 > 60000) ? 60000 : backoff * 2;
        }
      }
      continue;
    }

    // ---------------- MQTT: conectado ----------------
    mqtt.loop();                                          // mantém a conexão (keep-alive)

    // Eventos: publica quando algum estado importante muda
    bool changed = false;
    bool f = floatActive, r = relay1On, a = levelAlert;
    bool fl = flowIn.flowing, fl2 = flowOut.flowing;
    int  inf = inflowState;
    if (f   != pFloat)  { pFloat  = f;   netPublishEvent("boia",        f   ? "ATIVADA"  : "NAO_ATIVA");  changed = true; }
    if (r   != pRelay)  { pRelay  = r;   netPublishEvent("rele1",       r   ? "LIGADO"   : "DESLIGADO");  changed = true; }
    if (a   != pAlert)  { pAlert  = a;   netPublishEvent("alerta",      a   ? "ATIVO"    : "ENCERRADO");  changed = true; }
    if (fl  != pFlow)   { pFlow   = fl;  netPublishEvent("fluxo",       fl  ? "PASSANDO" : "PARADO");     changed = true; }
    if (fl2 != pFlow2)  { pFlow2  = fl2; netPublishEvent("fluxo_saida", fl2 ? "PASSANDO" : "PARADO");     changed = true; }
    if (inf != pInflow) { pInflow = inf; netPublishEvent("entrada",     inflowStr(inf));                  changed = true; }

    // Telemetria: periódica, e também logo após qualquer mudança
    if (changed || (millis() - lastTele >= MQTT_TELEMETRY_MS)) {
      netPublishTelemetry();
      lastTele = millis();
    }
  }
}

// Prepara tópicos, certificado e cliente MQTT e cria a task de rede
static bool netStart() {
  netBuildTopics();
#if MQTT_USE_TLS
  netClient.setCACert(MQTT_CA_CERT);                      // valida o servidor com o certificado do ca_cert.h
#endif
  netClient.setTimeout(10);                               // 10 s de timeout nas operações de rede
  mqtt.setServer(MQTT_HOST, MQTT_PORTA);
  mqtt.setBufferSize(768);                                // folga p/ o JSON da telemetria (padrão = 256)
  mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
  mqtt.setSocketTimeout(10);

  if (xTaskCreate(netTask, "net", NET_TASK_STACK, NULL, NET_TASK_PRIO, NULL) != pdPASS) return false;

  DBG("Rede iniciada: clientId=%s, TLS=%s", mqttClientId, MQTT_USE_TLS ? "sim" : "nao");
  return true;
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);   // tempo para o monitor serial USB conectar

#if APP_DEBUG_MODE
  Serial.println("*** MODO DEBUG ATIVADO ***");
#endif
  Serial.println("Controle de agua residencial - firmware iniciando");
  Serial.printf("Heap livre: %u B | CPU: %u MHz\n", (unsigned)ESP.getFreeHeap(), (unsigned)getCpuFrequencyMhz());

  hubInit();   // cria o hub de sensores

  // Ordem: os sensores registram os ids no hub ANTES das tasks que os usam
  if (!hcsr04Start())  Serial.println("ERRO: falha ao iniciar HC-SR04");
  if (!floatStart())   Serial.println("ERRO: falha ao iniciar boia");
  if (!flowStart())    Serial.println("ERRO: falha ao iniciar sensores de fluxo");
  if (!controlStart()) Serial.println("ERRO: falha ao iniciar controle (rele/buzzer)");
  if (!ledBarStart())  Serial.println("ERRO: falha ao iniciar barra de LEDs");
  if (!displayStart()) Serial.println("ERRO: falha ao iniciar display");
  if (!netStart())     Serial.println("ERRO: falha ao iniciar rede (Wi-Fi/MQTT)");   // por último: o controle já está rodando

#if APP_DEBUG_MODE
  xTaskCreate(debugTask, "dbg_print", DEBUG_TASK_STACK, NULL, DEBUG_TASK_PRIO, NULL);
#endif
}

void loop() {
  vTaskDelay(portMAX_DELAY);   // tudo roda nas tasks FreeRTOS
}