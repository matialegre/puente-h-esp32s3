/*
 * Puente H completo para motor DC con STM32F103 "Blue Pill".
 * 4 PWM complementarios con dead-time por hardware (TIM1) + control por RTT
 * (terminal sobre el ST-Link, sin cables extra).
 *
 * Etapa de potencia: P-MOS arriba (activo en BAJO) + N-MOS abajo (activo en alto).
 *   -> canal alto con polaridad invertida (CCxP=1), canal bajo normal (CCxNP=0).
 *
 * Pines (TIM1, sin remap):
 *   PA8  = CH1   = gate high-side rama A (HA, P-MOS, activo bajo)
 *   PB13 = CH1N  = gate low-side  rama A (LA, N-MOS, activo alto)
 *   PA9  = CH2   = gate high-side rama B (HB)
 *   PB14 = CH2N  = gate low-side  rama B (LB)
 *   Motor entre OUT_A (rama A) y OUT_B (rama B).
 *
 * Esquema sign-magnitude:
 *   adelante: rama A hace PWM, rama B a GND (low-side fijo)
 *   atras:    rama B hace PWM, rama A a GND
 *   freno:    ambas a GND (motor en corto)   coast: MOE=0 (todo apagado)
 *
 * PWM: 72 MHz / (ARR+1) = 20 kHz edge-aligned.
 *
 * Comandos (terminal RTT, una linea + Enter):
 *   F500  adelante 50.0%        R250  atras 25.0%
 *   S     stop (coast)          B     freno
 *   D700  dead-time 700 ns      E / X enable / disable salidas
 *   ?     estado
 */

#include <Arduino.h>

// ---------------- parametros PWM ----------------
static const uint32_t F_TIM   = 72000000UL;   // reloj de TIM1 (APB2)
static const uint32_t PWM_HZ  = 20000UL;
static const uint16_t ARR     = (F_TIM / PWM_HZ) - 1;   // 3599 -> 20 kHz
static const uint32_t T_DTS_PS = 13888UL;      // 1/72MHz en picosegundos (~13.89 ns)

// ---------------- estado del motor ----------------
enum Dir  { FWD, REV };
enum Mode { UNIPOLAR, BIPOLAR };   // unipolar: 1 rama conmuta | bipolar: 2 en contrafase
static Dir      g_dir   = FWD;
static Mode     g_mode  = UNIPOLAR;
static uint16_t g_speed = 0;       // 0..1000 (por mil)
static uint16_t g_dtg   = 36;      // ~500 ns de dead-time
static bool     g_enabled = false;

// =================================================================
//  RTT minimo (compatible con OpenOCD "rtt setup ... \"SEGGER RTT\"")
// =================================================================
#define RTT_UP_SIZE   512
#define RTT_DOWN_SIZE 64

static char rtt_up_buf[RTT_UP_SIZE];
static char rtt_down_buf[RTT_DOWN_SIZE];

typedef struct {
  const char *sName;
  char       *pBuffer;
  unsigned    SizeOfBuffer;
  volatile unsigned WrOff;   // up: lo escribe el target ; down: lo escribe el host
  volatile unsigned RdOff;   // up: lo escribe el host   ; down: lo escribe el target
  unsigned    Flags;
} RTT_BUF;

typedef struct {
  char    acID[16];
  int     MaxNumUpBuffers;
  int     MaxNumDownBuffers;
  RTT_BUF aUp[1];
  RTT_BUF aDown[1];
} RTT_CB;

// El control block vive en RAM (.data). OpenOCD lo encuentra buscando "SEGGER RTT".
static RTT_CB _rtt __attribute__((used)) = {
  "SEGGER RTT", 1, 1,
  { { "Terminal", rtt_up_buf,   RTT_UP_SIZE,   0, 0, 0 } },   // up   (MCU -> PC)
  { { "Terminal", rtt_down_buf, RTT_DOWN_SIZE, 0, 0, 0 } },   // down (PC -> MCU)
};

static void rtt_write(const char *s) {
  while (*s) {
    unsigned wr  = _rtt.aUp[0].WrOff;
    unsigned nxt = wr + 1; if (nxt >= RTT_UP_SIZE) nxt = 0;
    if (nxt == _rtt.aUp[0].RdOff) break;       // lleno: descarta (no bloquea)
    rtt_up_buf[wr] = *s++;
    _rtt.aUp[0].WrOff = nxt;
  }
}

static int rtt_getc(void) {
  unsigned rd = _rtt.aDown[0].RdOff;
  if (rd == _rtt.aDown[0].WrOff) return -1;     // vacio
  char c = rtt_down_buf[rd];
  unsigned nxt = rd + 1; if (nxt >= RTT_DOWN_SIZE) nxt = 0;
  _rtt.aDown[0].RdOff = nxt;
  return (int)(uint8_t)c;
}

// =================================================================
//  TIM1 a nivel de registros
// =================================================================
static void gpio_af_pp(GPIO_TypeDef *port, int pin) {
  // salida alterna push-pull 50 MHz: CNF=10, MODE=11  -> 0b1011 = 0xB
  volatile uint32_t *cr = (pin < 8) ? &port->CRL : &port->CRH;
  int sh = (pin & 7) * 4;
  *cr = (*cr & ~(0xFu << sh)) | (0xBu << sh);
}

static uint16_t deadtime_to_dtg(uint32_t ns) {
  // rango 0xxxxxxx del DTG: DT = DTG[6:0] * tDTS  (hasta ~1764 ns)
  uint32_t dtg = (ns * 1000UL) / T_DTS_PS;       // ns -> pasos de tDTS
  if (dtg > 127) dtg = 127;
  return (uint16_t)dtg;
}

static void tim1_apply_outputs(void) {
  const uint32_t full = ARR + 1;
  uint32_t sp = (uint32_t)g_speed * full / 1000;        // duty en cuentas (0..full)
  if (sp > ARR) sp = ARR;

  if (g_mode == UNIPOLAR) {
    // sign-magnitude: una rama hace PWM, la otra queda a GND (CCR=0)
    if (g_dir == FWD) { TIM1->CCR1 = sp; TIM1->CCR2 = 0;  }   // rama A PWM, B a GND
    else              { TIM1->CCR1 = 0;  TIM1->CCR2 = sp; }   // rama B PWM, A a GND
  } else {
    // bipolar (locked antiphase): las dos ramas en contrafase.
    // Vmotor = (CCR1-CCR2)/full * Vbus ; en reposo (s=0) ambas al 50%.
    uint32_t mid  = full / 2;
    uint32_t half = sp / 2;
    uint32_t hi = mid + half;                            // (1+s)/2
    uint32_t lo = mid - half;                            // (1-s)/2
    if (hi > ARR) hi = ARR;
    if (g_dir == FWD) { TIM1->CCR1 = hi; TIM1->CCR2 = lo; }
    else              { TIM1->CCR1 = lo; TIM1->CCR2 = hi; }
  }
}

static void tim1_set_deadtime(uint16_t dtg) {
  uint32_t bdtr = TIM1->BDTR & ~0xFFu;           // preserva MOE y demas
  bdtr |= (dtg & 0x7F);                          // DTG[6:0], bit7=0 -> rango lineal
  TIM1->BDTR = bdtr;
}

static void tim1_set_enabled(bool en) {
  if (en) TIM1->BDTR |=  TIM_BDTR_MOE;           // habilita las salidas
  else    TIM1->BDTR &= ~TIM_BDTR_MOE;           // coast: todo en estado idle (off)
  g_enabled = en;
}

static void tim1_init(void) {
  // relojes
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                  RCC_APB2ENR_AFIOEN | RCC_APB2ENR_TIM1EN;

  // pines de salida en AF push-pull
  gpio_af_pp(GPIOA, 8);    // CH1  (HA)
  gpio_af_pp(GPIOA, 9);    // CH2  (HB)
  gpio_af_pp(GPIOB, 13);   // CH1N (LA)
  gpio_af_pp(GPIOB, 14);   // CH2N (LB)

  // base de tiempo: edge-aligned, ARPE para CCR/ARR con preload
  TIM1->PSC = 0;
  TIM1->ARR = ARR;
  TIM1->CR1 = TIM_CR1_ARPE;

  // CH1 y CH2 en PWM modo 1, con preload de CCR
  TIM1->CCMR1 =
      (6u << 4) | TIM_CCMR1_OC1PE |          // OC1M=110 (PWM1), OC1PE=1
      (6u << 12) | TIM_CCMR1_OC2PE;          // OC2M=110 (PWM1), OC2PE=1

  // habilita salidas complementarias y fija polaridad:
  //   high-side P-MOS = activo en BAJO -> CCxP=1
  //   low-side  N-MOS = activo en alto -> CCxNP=0
  TIM1->CCER =
      TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC1P |
      TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC2P;

  // estados idle (cuando MOE=0): todo apagado.
  //   P-MOS off = gate alto -> OISx=1 ;  N-MOS off = gate bajo -> OISxN=0
  TIM1->CR2 = TIM_CR2_OIS1 | TIM_CR2_OIS2;

  // dead-time + (MOE arranca en 0 = salidas desactivadas)
  TIM1->BDTR = 0;
  tim1_set_deadtime(g_dtg);

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;

  TIM1->EGR = TIM_EGR_UG;                     // carga los registros con preload
  TIM1->CR1 |= TIM_CR1_CEN;                   // arranca el contador
}

// =================================================================
//  parser de comandos
// =================================================================
static void print_status(void) {
  char b[96];
  snprintf(b, sizeof(b),
           "[estado] %s | %s | %s vel=%u.%u%% | dead=%uns (dtg=%u) | PWM=%luHz\r\n",
           g_enabled ? "ON " : "OFF",
           g_mode == UNIPOLAR ? "UNI" : "BIP",
           g_dir == FWD ? "FWD" : "REV",
           g_speed / 10, g_speed % 10,
           (unsigned)((uint32_t)g_dtg * T_DTS_PS / 1000UL), g_dtg,
           (unsigned long)PWM_HZ);
  rtt_write(b);
}

static void handle_line(char *s) {
  // saltea espacios al inicio
  while (*s == ' ' || *s == '\t') s++;
  char c = *s++;
  long n = atol(s);                           // numero opcional tras el comando

  switch (c) {
    case 'F': case 'f':
      g_dir = FWD; g_speed = (n < 0) ? 0 : (n > 1000 ? 1000 : n);
      tim1_apply_outputs(); if (!g_enabled) tim1_set_enabled(true);
      rtt_write("ok adelante\r\n"); break;

    case 'R': case 'r':
      g_dir = REV; g_speed = (n < 0) ? 0 : (n > 1000 ? 1000 : n);
      tim1_apply_outputs(); if (!g_enabled) tim1_set_enabled(true);
      rtt_write("ok atras\r\n"); break;

    case 'S': case 's':
      g_speed = 0; tim1_apply_outputs(); tim1_set_enabled(false);
      rtt_write("ok stop (coast)\r\n"); break;

    case 'B': case 'b':
      g_speed = 0; TIM1->CCR1 = 0; TIM1->CCR2 = 0; tim1_set_enabled(true);
      rtt_write("ok freno\r\n"); break;

    case 'E': case 'e':
      tim1_set_enabled(true);  rtt_write("ok enable\r\n"); break;

    case 'X': case 'x':
      tim1_set_enabled(false); rtt_write("ok disable\r\n"); break;

    case 'D': case 'd':
      g_dtg = deadtime_to_dtg((n < 0) ? 0 : n);
      tim1_set_deadtime(g_dtg);
      rtt_write("ok dead-time\r\n"); break;

    case 'M': case 'm':                         // M0 = unipolar, M1 = bipolar
      g_mode = (n != 0) ? BIPOLAR : UNIPOLAR;
      tim1_apply_outputs();
      rtt_write(g_mode == BIPOLAR ? "ok bipolar\r\n" : "ok unipolar\r\n"); break;

    case '?':
      print_status(); break;

    case 0: break;                            // linea vacia
    default: rtt_write("?? comando desconocido\r\n"); break;
  }
}

static void poll_rtt_commands(void) {
  static char line[64];
  static uint8_t len = 0;
  int ci;
  while ((ci = rtt_getc()) >= 0) {
    char c = (char)ci;
    if (c == '\r' || c == '\n') {
      line[len] = 0;
      if (len) handle_line(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

// =================================================================
void setup() {
  tim1_init();
  rtt_write("\r\n== Puente H STM32F103 listo ==\r\n");
  rtt_write("comandos: F<n> R<n> S B D<ns> M<0|1> E X ?  (n=0..1000=0..100.0%)\r\n");
  rtt_write("  M0=unipolar  M1=bipolar\r\n");
  print_status();
}

void loop() {
  poll_rtt_commands();
}
