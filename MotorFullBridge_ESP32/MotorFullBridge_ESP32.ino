/* ═══════════════════════════════════════════════════════════════════
   PUENTE H  ·  ESP32  ·  MCPWM con dead-time por hardware
   Reemplazo del firmware de la Blue Pill (STM32 TIM1 complementario).

   ─── POR QUE MCPWM Y NO LEDC ────────────────────────────────────────
   El LEDC (el de analogWrite) NO tiene tiempo muerto. En un puente H eso
   significa que en cada conmutacion hay un instante con las dos llaves de
   la misma rama conduciendo: cortocircuito de la fuente a traves de los
   MOSFET. El MCPWM inserta el dead-time en el propio hardware, sin
   depender de que el codigo llegue a tiempo. Es la unica opcion correcta.

   ─── LA POLARIDAD, QUE ES LO QUE PUEDE QUEMAR ALGO ──────────────────
   En este circuito cada gate lo maneja un 2N2222 que INVIERTE:

     pin en ALTO  -> BJT conduce -> gate a masa
                     MOSFET P (arriba, source a +12V): Vgs = -12V -> PRENDE
                     MOSFET N (abajo,  source a masa): Vgs =  0V  -> APAGA

     pin en BAJO  -> BJT cortado -> gate a +12V por la 2k2
                     MOSFET P: Vgs = 0    -> APAGA
                     MOSFET N: Vgs = +12V -> PRENDE

   Consecuencia: las dos patas de una misma rama van al MISMO nivel en
   regimen, no a niveles opuestos.

       HA=1 LA=1  ->  P prendido, N apagado  ->  salida a +12V
       HA=0 LA=0  ->  P apagado,  N prendido ->  salida a masa
       HA=0 LA=1  ->  los dos apagados       ->  DEAD-TIME / libre
       HA=1 LA=0  ->  LOS DOS PRENDIDOS      ->  CORTOCIRCUITO

   El ultimo estado no puede ocurrir NUNCA. Por eso el dead-time se
   configura para que la transicion pase por (0,1) y jamas por (1,0).

   ─── COMO SE LOGRA ESO CON EL MCPWM ─────────────────────────────────
   Los dos generadores de una rama arrancan de la MISMA senal S:
     genA: se le retrasa el flanco de SUBIDA   (posedge_delay)
     genB: se le retrasa el flanco de BAJADA   (negedge_delay)

     S sube    ->  A=0, B=1  ->  (0,1) los dos apagados   <- dead-time
     +DT       ->  A=1, B=1  ->  P prendido
     S baja    ->  A=0, B=1  ->  (0,1) los dos apagados   <- dead-time
     +DT       ->  A=0, B=0  ->  N prendido

   Nunca aparece (1,0). El hueco cae siempre del lado seguro.

   ─── ANTES DE CONECTAR EL MOTOR ─────────────────────────────────────
   Enchufa el analizador logico a los 4 pines y verifica:
     1. Los 4 conmutan a 20 kHz.
     2. HA y LA de la misma rama nunca estan en (1,0).
     3. Entre que uno cae y el otro sube hay un hueco medible (el DT).
   Recien despues alimenta el puente, y con 5-6 V y sin motor.
   ═══════════════════════════════════════════════════════════════════ */

#include <Arduino.h>
#include "driver/mcpwm_prelude.h"
#include <WiFi.h>
#include <WebServer.h>

// ── Control remoto ──────────────────────────────────────────────────
// El ESP32 levanta su PROPIA red. No depende del WiFi del taller ni de
// que haya router: te conectas con el celular y entras a 192.168.4.1.
// Un puente de motor no deberia depender de una red ajena para parar.
// La red lleva clave WPA2: quien entre a 192.168.4.1 puede mover el motor.
// La clave por defecto es PUBLICA (esta en este archivo y en el README):
// cambiala antes de usar la placa. Dos formas:
//   1. editar AP_CLAVE aca abajo (entre 8 y 63 caracteres), o
//   2. pasarla al compilar, sin tocar el archivo:
//      arduino-cli compile ... --build-property "compiler.cpp.extra_flags=-DAP_CLAVE=\"otraClave123\""
// Si la clave no tiene entre 8 y 63 caracteres el firmware NO levanta la
// red (lo avisa por serie) y el puente queda manejable solo por USB.
#ifndef AP_SSID
#define AP_SSID   "PuenteH"
#endif
#define AP_CLAVE_DEFAULT "cambiame-1234"
#ifndef AP_CLAVE
#define AP_CLAVE  AP_CLAVE_DEFAULT
#endif
static WebServer g_web(80);

// Ademas de su red, se cuelga del WiFi de casa (si esta) para que la PC
// llegue a la misma pagina sin perder internet. Las credenciales viven en
// wifi_casa.h, que es local y no se comparte. Sin ese archivo, solo AP.
#define USAR_WIFI_CASA 0            // 0 = solo la red propia (uso en el laboratorio)
#if USAR_WIFI_CASA && __has_include("wifi_casa.h")
#include "wifi_casa.h"
#endif

// Hombre muerto: si el navegador deja de dar senales de vida (se cerro la
// pestana, se fue el WiFi, se apago el celular) y el motor esta girando,
// se corta solo. Sin esto, perder la conexion deja el motor a fondo y sin
// forma de pararlo salvo desenchufar.
#define VIGILIA_MS 4000
static uint32_t g_ultimoContacto = 0;

// ── Pines ───────────────────────────────────────────────────────────
// Placa: ESP32-S3-WROOM-1 N16R8. Los GPIO 26-32 del S3 estan cableados a
// la flash y a la PSRAM del modulo y NO salen al header — por eso no hay
// 25/26/32/33 en la serigrafia. Se eligen 4 GPIO libres y CONSECUTIVOS en
// la fila de abajo, para enchufar el analizador de un tiron. Se evitan los
// de arranque (0, 3, 45, 46), el LED RGB (48) y el USB nativo (19, 20).
#define PIN_HA   4     // rama A · gate del MOSFET P (arriba)
#define PIN_LA   5     // rama A · gate del MOSFET N (abajo)
#define PIN_HB   6     // rama B · gate del MOSFET P (arriba)
#define PIN_LB   7     // rama B · gate del MOSFET N (abajo)

// ── LA BANDERA QUE QUEMA PUENTES ────────────────────────────────────
// El ejemplo oficial de ESP-IDF para dead-time complementario pone
// `flags.invert_output = true` en el segundo generador. Eso da salidas
// OPUESTAS, que es lo correcto para un puente con drivers normales.
//
// En ESTA placa el 2N2222 ya invierte. Salidas opuestas en el pin
// significan las dos llaves de la rama conduciendo: cortocircuito franco
// en todos los modos utiles. Por eso aca NO se usa esa bandera y las dos
// senales de una rama quedan al MISMO nivel en regimen.
//
// Si algun dia cambias los drivers por unos que no inviertan (un IR2104,
// por ejemplo), este es el unico lugar que hay que tocar.
#define DRIVER_INVIERTE 1

// ── Tiempos ─────────────────────────────────────────────────────────
// El timer cuenta 0 -> pico -> 0 (subida/bajada): ES la portadora
// triangular. Con 40 MHz y 20 kHz el pico es 1000 -> 1000 pasos de duty
// (~10 bits) y el dead-time se ajusta de a 25 ns.
#define RELOJ_HZ        40000000UL   // 40 MHz -> 1 tick = 25 ns
// Valores por defecto = los que ANDAN en esta placa (medido 4-oct): con los
// 2N2222 + 2k2 el gate del P tarda ~12 us en apagarse, y con dead-time de
// 0,5 o 2 us hay conduccion cruzada (salta la fuente). Con 5 kHz y 5 us anda.
// 20 kHz / 2 us (lo del TP) recien con un driver de gate mas rapido.
#define FRECUENCIA_DEF  5000
#define FRECUENCIA_MIN  1000
#define FRECUENCIA_MAX  50000
#define DEADTIME_NS_DEF 5000
#define DEADTIME_NS_MAX 5000
#define ZONA_MUERTA_PCT 4.0f         // |Vm| < 4 % -> salida 0 (estado 5h): por debajo
                                     // el pulso queda mas corto que el dead-time

// ── Estado ──────────────────────────────────────────────────────────
enum Modo { LIBRE, FRENO, ADELANTE, ATRAS };
// Modulaciones (notacion del TP: SW1=P rama A, SW2=N rama A, SW3=P rama B,
// SW4=N rama B):
//   SIGNO    una rama conmuta y la otra queda abajo        9 <-> 5
//   BIPOLAR  rama B = complemento de rama A                 9 <-> 6
//   UNIPOLAR +vctrl y -vctrl contra la misma triangular     A-9-5-9
enum Mod  { SIGNO, BIPOLAR, UNIPOLAR };
static Modo     g_modo     = LIBRE;
static Mod      g_mod      = UNIPOLAR;
static float    g_velocidad = 0.0f;   // 0..100 %
static uint32_t g_deadNs   = DEADTIME_NS_DEF;
static uint32_t g_frecHz   = FRECUENCIA_DEF;
static uint32_t g_periodo  = RELOJ_HZ / FRECUENCIA_DEF;   // ticks por periodo

// UN solo timer para las dos ramas: si cada rama tuviera el suyo, la fase
// entre ramas quedaria librada a cuando arranco cada uno, y la secuencia
// de estados del puente (9, 6, A, 5) dejaria de ser la de la teoria.
static mcpwm_timer_handle_t g_timer;

// Una rama del puente: operador + comparador + dos generadores.
struct Rama {
  mcpwm_oper_handle_t      oper;
  mcpwm_cmpr_handle_t      comp;
  mcpwm_gen_handle_t       genAlto;   // va al gate del P
  mcpwm_gen_handle_t       genBajo;   // va al gate del N
  int                      inv;       // acciones cargadas: -1 ninguna, 0 normal, 1 invertida
};
static Rama g_ramaA, g_ramaB;

// ── Construccion de una rama ────────────────────────────────────────
static void timerInit() {
  mcpwm_timer_config_t tc = {};
  tc.group_id      = 0;
  tc.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tc.resolution_hz = RELOJ_HZ;
  tc.count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN;   // triangular
  tc.period_ticks  = g_periodo;                       // pico = periodo/2
  ESP_ERROR_CHECK(mcpwm_new_timer(&tc, &g_timer));
}

static void timerArrancar() {
  ESP_ERROR_CHECK(mcpwm_timer_enable(g_timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(g_timer, MCPWM_TIMER_START_NO_STOP));
}

static void ramaInit(Rama& r, int pinAlto, int pinBajo) {
  r.inv = -1;
  mcpwm_operator_config_t oc = {};
  oc.group_id = 0;
  ESP_ERROR_CHECK(mcpwm_new_operator(&oc, &r.oper));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(r.oper, g_timer));

  mcpwm_comparator_config_t cc = {};
  cc.flags.update_cmp_on_tez = true;      // el duty cambia en el valle
  ESP_ERROR_CHECK(mcpwm_new_comparator(r.oper, &cc, &r.comp));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(r.comp, 0));

  mcpwm_generator_config_t gc = {};
  // Habilita el camino de entrada del GPIO para poder LEER lo que el propio
  // MCPWM esta sacando. Es lo que permite el autochequeo del comando 'p'.
  gc.flags.io_loop_back = true;
  gc.gen_gpio_num = pinAlto;
  ESP_ERROR_CHECK(mcpwm_new_generator(r.oper, &gc, &r.genAlto));
  gc.gen_gpio_num = pinBajo;
  ESP_ERROR_CHECK(mcpwm_new_generator(r.oper, &gc, &r.genBajo));

}

// Acciones de los generadores contra la triangular. Los DOS generadores de
// una rama reciben exactamente las mismas ordenes; lo que los diferencia
// despues es el dead-time.
//   normal:    S = 1 mientras contador < comparador  (pulso centrado en el valle)
//              -> rama arriba (P prendido) una fraccion cmp/pico del periodo
//   invertida: S = 1 mientras contador > comparador  (el complemento exacto)
// Es la comparacion "vctrl > vtri" de la teoria, hecha por hardware.
static void ramaAcciones(Rama& r, int invertida) {
  if (r.inv == invertida) return;
  mcpwm_generator_action_t subiendo = invertida ? MCPWM_GEN_ACTION_HIGH : MCPWM_GEN_ACTION_LOW;
  mcpwm_generator_action_t bajando  = invertida ? MCPWM_GEN_ACTION_LOW  : MCPWM_GEN_ACTION_HIGH;
  for (mcpwm_gen_handle_t g : { r.genAlto, r.genBajo }) {
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(g,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, r.comp, subiendo)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(g,
      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, r.comp, bajando)));
  }
  r.inv = invertida;
}

// El hueco va del lado seguro: retrasamos la SUBIDA del que prende el P y
// la BAJADA del que prende el N. Asi la transicion pasa por (0,1) —los dos
// MOSFET cortados— y nunca por (1,0).
static void ramaDeadTime(Rama& r, uint32_t ticks) {
  mcpwm_dead_time_config_t dtAlto = {};
  dtAlto.posedge_delay_ticks = ticks;
#if !DRIVER_INVIERTE
  dtAlto.flags.invert_output = true;   // solo con drivers que NO inviertan
#endif
  ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(r.genAlto, r.genAlto, &dtAlto));

  mcpwm_dead_time_config_t dtBajo = {};
  dtBajo.negedge_delay_ticks = ticks;
  ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(r.genBajo, r.genBajo, &dtBajo));
}

// ns -> ticks del timer (1 tick = 1e9/RELOJ_HZ ns), redondeado. Devuelve
// los ticks aplicados; g_deadNs queda con el valor REAL, no el pedido.
static uint32_t fijarDeadTime(uint32_t ns) {
  if (ns > DEADTIME_NS_MAX) ns = DEADTIME_NS_MAX;
  const uint32_t nsTick = 1000000000UL / RELOJ_HZ;
  uint32_t ticks = (ns + nsTick / 2) / nsTick;
  if (ticks == 0 && ns > 0) ticks = 1;
  g_deadNs = ticks * nsTick;
  ramaDeadTime(g_ramaA, ticks);
  ramaDeadTime(g_ramaB, ticks);
  return ticks;
}

// Fuerza una rama a un estado fijo, sin PWM.
//   nivelP / nivelN son los niveles del PIN, no del MOSFET. Acordate de
//   que el BJT invierte: pin 1 = P prendido / N apagado.
static void ramaForzar(Rama& r, int nivelP, int nivelN) {
  ESP_ERROR_CHECK(mcpwm_generator_set_force_level(r.genAlto, nivelP, true));
  ESP_ERROR_CHECK(mcpwm_generator_set_force_level(r.genBajo, nivelN, true));
}

static void ramaLiberar(Rama& r) {   // vuelve a mandar el PWM
  ESP_ERROR_CHECK(mcpwm_generator_set_force_level(r.genAlto, -1, true));
  ESP_ERROR_CHECK(mcpwm_generator_set_force_level(r.genBajo, -1, true));
}

// Pone una rama a conmutar con una fraccion 'arriba' (0..1) del periodo con
// el P prendido. 'invertida' = pulso alrededor del pico en vez del valle.
// En 0 y en 1 no hay pulso que generar: se fuerza el nivel, sin PWM.
static void ramaPWM(Rama& r, float arriba, int invertida) {
  if (arriba <= 0.0f) { ramaForzar(r, 0, 0); return; }   // N prendido fijo
  if (arriba >= 1.0f) { ramaForzar(r, 1, 1); return; }   // P prendido fijo
  if (r.inv != invertida) {
    ramaForzar(r, 0, 1);              // cortada mientras cambian las acciones
    ramaAcciones(r, invertida);
  }
  uint32_t pico = g_periodo / 2;
  float f = invertida ? 1.0f - arriba : arriba;
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(r.comp, (uint32_t)(pico * f + 0.5f)));
  ramaLiberar(r);
}

static void fijarFrecuencia(uint32_t hz) {
  if (hz < FRECUENCIA_MIN) hz = FRECUENCIA_MIN;
  if (hz > FRECUENCIA_MAX) hz = FRECUENCIA_MAX;
  g_frecHz  = hz;
  g_periodo = RELOJ_HZ / hz;
  ESP_ERROR_CHECK(mcpwm_timer_set_period(g_timer, g_periodo));
  // El duty vive en CUENTAS, no en porcentaje: si no se reescribe, cambiar
  // la frecuencia le cambia el ancho de pulso al motor sin que nadie lo
  // haya pedido.
  aplicar();
}

// ── Aplicar el estado pedido ────────────────────────────────────────
static void aplicar() {
  switch (g_modo) {
    case LIBRE:   // los cuatro MOSFET cortados: el motor gira por inercia
      ramaForzar(g_ramaA, 0, 1);
      ramaForzar(g_ramaB, 0, 1);
      break;

    case FRENO:   // los dos N prendidos: el motor queda en corto, frena duro
      ramaForzar(g_ramaA, 0, 0);
      ramaForzar(g_ramaB, 0, 0);
      break;

    case ADELANTE:
    case ATRAS: {
      // s en -1..+1: el signo es el sentido, el modulo la tension media
      // pedida (Vm = s * Vcc en las tres modulaciones).
      float s = g_velocidad / 100.0f * (g_modo == ADELANTE ? 1.0f : -1.0f);

      // Zona muerta: cerca de Vm = 0 no se conmuta. Las dos ramas abajo
      // (estado 5h, freewheeling inferior): salida 0 V exacta y sin
      // pulsos angostos que el dead-time se comeria a medias.
      if (fabsf(s) * 100.0f < ZONA_MUERTA_PCT) {
        ramaForzar(g_ramaA, 0, 0);
        ramaForzar(g_ramaB, 0, 0);
        break;
      }

      switch (g_mod) {
        // SIGNO-MAGNITUD: conmuta una sola rama, la otra queda abajo.
        // El motor ve 0 o +Vcc: estados 9h y 5h (o 6h y 5h al reves).
        case SIGNO:
          if (s > 0) { ramaPWM(g_ramaA,  s, 0); ramaForzar(g_ramaB, 0, 0); }
          else       { ramaPWM(g_ramaB, -s, 0); ramaForzar(g_ramaA, 0, 0); }
          break;

        // BIPOLAR: una sola senal de control. Rama A arriba una fraccion
        // dA = (1+s)/2 y rama B exactamente al reves (mismo comparador,
        // acciones invertidas): SW1/SW4 y SW2/SW3 como pares rigidos.
        // Estados 9h <-> 6h, el motor ve +Vcc o -Vcc. Vm = Vcc(2dA-1) = s.Vcc
        case BIPOLAR: {
          float dA = (1.0f + s) / 2.0f;
          ramaPWM(g_ramaA, dA, 0);
          ramaPWM(g_ramaB, 1.0f - dA, 1);
          break;
        }

        // UNIPOLAR: +vctrl contra la triangular maneja la rama A y -vctrl
        // la rama B. Las dos conmutan, cada una por su lado:
        // secuencia A-9-5-9 (s > 0) o A-6-5-6 (s < 0). La carga ve pulsos
        // a 2.fsw con la misma portadora. Vm = Vcc(dA - dB) = s.Vcc
        case UNIPOLAR:
          ramaPWM(g_ramaA, (1.0f + s) / 2.0f, 0);
          ramaPWM(g_ramaB, (1.0f - s) / 2.0f, 0);
          break;
      }
      break;
    }
  }
}

static const char* nombreMod() {
  return g_mod == SIGNO ? "SIGNO" : (g_mod == BIPOLAR ? "BIPOLAR" : "UNIPOLAR");
}

static const char* nombreModo() {
  switch (g_modo) {
    case LIBRE:    return "LIBRE (coast)";
    case FRENO:    return "FRENO";
    case ADELANTE: return "ADELANTE";
    default:       return "ATRAS";
  }
}

static String json();   // definida mas abajo, junto al servidor web

static void estado() {
  Serial.printf("\n  modo %s | %s | velocidad %.1f %% | dead-time %u ns | %u Hz\n",
                nombreModo(), nombreMod(),
                g_velocidad, g_deadNs, g_frecHz);
  Serial.printf("  pines  HA=%d  LA=%d  HB=%d  LB=%d\n",
                PIN_HA, PIN_LA, PIN_HB, PIN_LB);
  Serial.printf("  red propia %s en %s | WiFi casa: %s\n\n", AP_SSID,
                WiFi.softAPIP().toString().c_str(),
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "no conectado");
}

// Autochequeo: lee los 4 pines y avisa si una rama quedo en (1,0), que es
// el estado que pone en corto la fuente a traves de los dos MOSFET. En los
// modos quietos (LIBRE, FRENO, duty 0 o 100) la lectura es exacta. Con PWM
// corriendo es una foto en un instante cualquiera, y se aclara.
static void chequearPines() {
  int ha = digitalRead(PIN_HA), la = digitalRead(PIN_LA);
  int hb = digitalRead(PIN_HB), lb = digitalRead(PIN_LB);
  bool quieto = (g_modo == LIBRE || g_modo == FRENO ||
                 g_velocidad <= 0.01f || g_velocidad >= 99.99f);

  Serial.printf("\n  pines leidos:  A=(%d,%d)   B=(%d,%d)\n", ha, la, hb, lb);

  bool malA = (ha == 1 && la == 0), malB = (hb == 1 && lb == 0);
  if (malA || malB) {
    Serial.printf("  *** PELIGRO: rama %s en (1,0) = los dos MOSFET conduciendo.\n",
                  malA && malB ? "A y B" : (malA ? "A" : "B"));
    Serial.println("  *** NO alimentes el puente. Es cortocircuito de la fuente.");
  } else if (quieto) {
    Serial.println("  OK: ninguna rama en (1,0). Estado quieto, la lectura es exacta.");
  } else {
    Serial.println("  OK en esta muestra, pero hay PWM corriendo: es una foto suelta.");
    Serial.println("  Para el hueco del dead-time hace falta el analizador logico.");
  }
}

static void ayuda() {
  Serial.println(F(
    "\n  ── PUENTE H · ESP32 ─────────────────────────────────────────\n"
    "   v <0-100>   velocidad en %\n"
    "   a           adelante\n"
    "   r           atras (reversa)\n"
    "   f           freno (los dos N prendidos, motor en corto)\n"
    "   s           libre / coast (los cuatro cortados)\n"
    "   b / u / g   modulacion bipolar (9-6) / unipolar (A-9-5-9) / signo-magnitud\n"
    "   h <Hz>      frecuencia de conmutacion (portadora triangular)\n"
    "   d <ns>      dead-time en nanosegundos (paso de 25)\n"
    "   ?           estado\n"
    "  ─────────────────────────────────────────────────────────────\n"
    "   Antes de conectar el motor: analizador logico en los 4 pines.\n"
    "   HA y LA NUNCA pueden estar en (1,0) a la vez. Si lo ves, PARA.\n"));
}

// ── Consola ─────────────────────────────────────────────────────────
static void procesar(String s) {
  s.trim();
  if (!s.length()) return;
  char c = s[0];
  String arg = s.substring(1); arg.trim();

  switch (c) {
    case 'v': {
      float v = arg.toFloat();
      if (v < 0) v = 0; if (v > 100) v = 100;
      g_velocidad = v; aplicar();
      Serial.printf("  velocidad -> %.1f %%\n", g_velocidad);
      break;
    }
    case 'a': g_modo = ADELANTE; aplicar(); Serial.println("  ADELANTE"); break;
    case 'r': g_modo = ATRAS;    aplicar(); Serial.println("  ATRAS");    break;
    case 'f': g_modo = FRENO;    aplicar(); Serial.println("  FRENO");    break;
    case 's': g_modo = LIBRE;    aplicar(); Serial.println("  LIBRE (coast)"); break;
    case 'd': {
      uint32_t ticks = fijarDeadTime((uint32_t)arg.toInt());
      Serial.printf("  dead-time -> %u ns (%u ticks)\n", g_deadNs, ticks);
      break;
    }
    case 'h': {
      fijarFrecuencia((uint32_t)arg.toInt());
      Serial.printf("  frecuencia -> %u Hz (periodo %u cuentas)\n", g_frecHz, g_periodo);
      break;
    }
    case 'b': g_mod = BIPOLAR;  aplicar(); Serial.println("  modulacion BIPOLAR (9h <-> 6h)");  break;
    case 'u': g_mod = UNIPOLAR; aplicar(); Serial.println("  modulacion UNIPOLAR (A-9-5-9)");  break;
    case 'g': g_mod = SIGNO;    aplicar(); Serial.println("  modulacion SIGNO-MAGNITUD (9h <-> 5h)"); break;
    // Mismo JSON que devuelve la pagina web. Una sola fuente para el
    // estado: si se agrega un campo, aparece en los dos lados a la vez.
    // Es lo que le permite al puente serie de la PC hablar con esto sin
    // parsear texto pensado para leer con los ojos.
    case 'j': Serial.println(json()); break;
    case 'p': chequearPines(); break;
    case '?': estado(); break;
    default:  ayuda();  break;
  }
}

// ── Pagina de control ───────────────────────────────────────────────
// Va embebida en el firmware: sin internet, sin CDN, sin instalar nada.
static const char PAGINA[] PROGMEM = R"HTML(<!doctype html><html lang="es"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Puente H</title><style>
*{box-sizing:border-box}
body{margin:0 auto;padding:14px;background:#0b0e13;color:#c9d1d9;
     font:15px/1.45 ui-monospace,Consolas,monospace;max-width:520px}
h1{font-size:13px;letter-spacing:.18em;color:#8b949e;margin:0 0 12px;display:flex;justify-content:space-between}
#con{font-size:11px;letter-spacing:.05em}
.ok{color:#3fb950}.mal{color:#f85149}
.p{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px;margin-bottom:10px}
.l{font-size:10px;letter-spacing:.14em;color:#8b949e;text-transform:uppercase;margin-bottom:8px}
.g{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.g3{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:8px}
button{font:inherit;font-size:14px;padding:13px 6px;border-radius:6px;cursor:pointer;
       background:#21262d;color:#c9d1d9;border:1px solid #30363d}
button:active{transform:translateY(1px)}
button.on{background:#1f6feb;border-color:#58a6ff;color:#fff}
#paro{grid-column:1/-1;background:#8b1a1a;border-color:#ff7b72;color:#fff;font-weight:700;letter-spacing:.12em;padding:16px}
input[type=range]{width:100%;accent-color:#1f6feb;height:34px}
.v{font-size:36px;color:#e6edf3;text-align:center}
.s{font-size:12px;color:#8b949e;text-align:center}
.f{display:flex;justify-content:space-between;font-size:12px;color:#8b949e;padding:2px 0}
.f b{color:#c9d1d9;font-weight:400}
</style></head><body>
<h1><span>PUENTE H · ESP32-S3</span><span id="con" class="mal">sin conexion</span></h1>

<div class="p"><div class="l">Velocidad</div>
  <div class="v"><span id="vv">0</span> %</div>
  <div class="s" id="vm">Vm = 0 % de Vcc</div>
  <input type="range" id="vs" min="0" max="100" step="1" value="0">
  <div class="g4">
    <button id="m10" onclick="paso(-10)">-10</button><button id="m1" onclick="paso(-1)">-1</button>
    <button id="p1" onclick="paso(1)">+1</button><button id="p10" onclick="paso(10)">+10</button>
  </div>
</div>

<div class="p"><div class="l">Sentido</div><div class="g">
  <button id="ba" onclick="cmd('m=a')">ADELANTE</button>
  <button id="br" onclick="cmd('m=r')">ATRAS</button>
  <button id="bf" onclick="cmd('m=f')">FRENO</button>
  <button id="bs" onclick="cmd('m=s')">LIBRE</button>
  <button id="paro" onclick="paro()">PARO</button>
</div></div>

<div class="p"><div class="l">Modulacion</div><div class="g3">
  <button id="mb" onclick="cmd('m=b')">BIPOLAR<br><small>9-6</small></button>
  <button id="mu" onclick="cmd('m=u')">UNIPOLAR<br><small>A-9-5-9</small></button>
  <button id="mg" onclick="cmd('m=g')">SIGNO<br><small>9-5</small></button>
</div></div>

<div class="p"><div class="l">Estado</div>
  <div class="f"><span>modo</span><b id="em">-</b></div>
  <div class="f"><span>modulacion</span><b id="eo">-</b></div>
  <div class="f"><span>velocidad</span><b id="ev">-</b></div>
  <div class="f"><span>dead-time</span><b id="ed">-</b></div>
  <div class="f"><span>frecuencia</span><b id="ef">-</b></div>
  <div class="f"><span>zona muerta</span><b id="ez">-</b></div>
  <div class="f"><span>IP en casa</span><b id="ei">-</b></div>
</div>

<script>
var vel=0,ocupado=0,ultimo=0;
function $(i){return document.getElementById(i)}
function cmd(q){return fetch('/cmd?'+q).then(function(r){return r.json()}).then(pintar).catch(function(){})}
function paro(){$('vs').value=0;cmd('m=s&v=0')}
function paso(d){var n=Math.max(0,Math.min(100,vel+d));$('vs').value=n;cmd('v='+n)}
// El slider manda mientras se arrastra, de a una peticion por vez (en el
// celular si no se encolan); al soltar manda el valor final.
$('vs').oninput=function(){$('vv').textContent=this.value;if(ocupado)return;ocupado=1;
  cmd('v='+this.value).then(function(){ocupado=0})};
$('vs').onchange=function(){cmd('v='+this.value)};
function pintar(e){
  if(!e)return; ultimo=Date.now(); $('con').textContent='conectado';$('con').className='ok';
  vel=Math.round(e.vel);
  if(document.activeElement!==$('vs')){$('vs').value=vel;$('vv').textContent=vel}
  var s=(e.modo=='ADELANTE'?1:e.modo=='ATRAS'?-1:0)*e.vel;
  $('vm').textContent='Vm = '+(Math.abs(s)<e.zm?0:s).toFixed(0)+' % de Vcc';
  $('em').textContent=e.modo;$('eo').textContent=e.mod;$('ev').textContent=e.vel.toFixed(1)+' %';
  $('ed').textContent=(e.dt/1000).toFixed(2)+' us';$('ef').textContent=(e.hz/1000)+' kHz';
  $('ez').textContent='< '+e.zm+' %';$('ei').textContent=e.ip;
  var m={ADELANTE:'ba',ATRAS:'br',FRENO:'bf',LIBRE:'bs'};
  for(var k in m)$(m[k]).className=(e.modo==k?'on':'');
  var o={BIPOLAR:'mb',UNIPOLAR:'mu',SIGNO:'mg'};
  for(var k in o)$(o[k]).className=(e.mod==k?'on':'');
}
// Este latido es tambien el hombre muerto: si se corta 4 s, el ESP32 para solo.
setInterval(function(){fetch('/estado').then(function(r){return r.json()}).then(pintar).catch(function(){});
  if(Date.now()-ultimo>2500){$('con').textContent='sin conexion';$('con').className='mal'}},1000);
fetch('/estado').then(function(r){return r.json()}).then(pintar).catch(function(){});
</script></body></html>)HTML";

// ── Servidor ────────────────────────────────────────────────────────
// Cuando el mando lo tiene el navegador se arma el hombre muerto. Si
// estas trabajando solo por el puerto serie no se arma: si no, el motor
// se te cortaria a los 4 segundos sin ninguna razon visible.
static bool g_mandaWeb = false;

static String json() {
  char b[300];
  const char* m = (g_modo == LIBRE) ? "LIBRE"
                : (g_modo == FRENO) ? "FRENO"
                : (g_modo == ADELANTE) ? "ADELANTE" : "ATRAS";
  String ip = WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("-");
  snprintf(b, sizeof(b),
    "{\"modo\":\"%s\",\"mod\":\"%s\",\"bip\":%d,\"vel\":%.1f,\"dt\":%u,\"hz\":%d,"
    "\"zm\":%.0f,\"ip\":\"%s\",\"pines\":\"%d %d %d %d\"}",
    m, nombreMod(), g_mod == BIPOLAR ? 1 : 0, g_velocidad, g_deadNs, g_frecHz,
    ZONA_MUERTA_PCT, ip.c_str(), PIN_HA, PIN_LA, PIN_HB, PIN_LB);
  return String(b);
}

// El panel embebido se sirve desde el propio ESP32 y no necesita esto,
// pero motor_ui.html se abre desde la PC (otro origen). Sin la cabecera
// el navegador bloquea la respuesta sin decir nada: la pagina queda
// "sin conexion" aunque el ESP32 conteste perfecto.
static void cors() { g_web.sendHeader("Access-Control-Allow-Origin", "*"); }

static void webEstado() {
  cors();
  g_ultimoContacto = millis();
  g_web.send(200, "application/json", json());
}

static void webCmd() {
  cors();
  g_ultimoContacto = millis();
  g_mandaWeb = true;

  if (g_web.hasArg("v")) {
    float v = g_web.arg("v").toFloat();
    if (v < 0) v = 0; if (v > 100) v = 100;
    g_velocidad = v;
  }
  if (g_web.hasArg("d")) fijarDeadTime((uint32_t)g_web.arg("d").toInt());
  if (g_web.hasArg("f")) fijarFrecuencia((uint32_t)g_web.arg("f").toInt());
  if (g_web.hasArg("m")) {
    char m = g_web.arg("m")[0];
    if      (m == 'a') g_modo = ADELANTE;
    else if (m == 'r') g_modo = ATRAS;
    else if (m == 'f') g_modo = FRENO;
    else if (m == 's') g_modo = LIBRE;
    else if (m == 'u') g_mod = UNIPOLAR;
    else if (m == 'b') g_mod = BIPOLAR;
    else if (m == 'g') g_mod = SIGNO;
  }
  aplicar();
  g_web.send(200, "application/json", json());
}

static void webInit() {
#if USAR_WIFI_CASA && defined(CASA_SSID)
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(CASA_SSID, CASA_CLAVE);   // no bloquea: si no conecta, el AP anda igual
#else
  WiFi.mode(WIFI_AP);
#endif
  const size_t largo = strlen(AP_CLAVE);
  const bool claveOk = largo >= 8 && largo <= 63;
  if (claveOk) WiFi.softAP(AP_SSID, AP_CLAVE);
  g_web.on("/",       []{ g_web.send_P(200, "text/html", PAGINA); });
  g_web.on("/estado", webEstado);
  g_web.on("/cmd",    webCmd);
  g_web.begin();

  if (!claveOk) {
    Serial.println("\n  ERROR: AP_CLAVE debe tener entre 8 y 63 caracteres."
                   " Red NO levantada; control solo por serie.\n");
    return;
  }
  Serial.printf("\n  RED  %s   (WPA2)\n", AP_SSID);
  if (strcmp(AP_CLAVE, AP_CLAVE_DEFAULT) == 0)
    Serial.println("  OJO: clave por defecto (publica). Cambiala: ver AP_CLAVE.");
  Serial.print  ("  ABRI http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("  en el celular\n");
}

// ── Arranque ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  // Primero lo seguro, despues lo lindo: dejamos los pines en el estado
  // "todos los MOSFET cortados" ANTES de que el MCPWM tome control. Si no,
  // durante la configuracion los pines quedan en alta impedancia, el BJT
  // se corta, los gates suben a 12 V por las 2k2 y los DOS MOSFET N
  // conducen: el motor queda frenado. No es peligroso, pero no es lo que
  // uno espera al enchufar.
  pinMode(PIN_HA, OUTPUT); digitalWrite(PIN_HA, LOW);
  pinMode(PIN_HB, OUTPUT); digitalWrite(PIN_HB, LOW);
  pinMode(PIN_LA, OUTPUT); digitalWrite(PIN_LA, HIGH);
  pinMode(PIN_LB, OUTPUT); digitalWrite(PIN_LB, HIGH);

  timerInit();
  ramaInit(g_ramaA, PIN_HA, PIN_LA);
  ramaInit(g_ramaB, PIN_HB, PIN_LB);
  fijarDeadTime(g_deadNs);

  g_modo = LIBRE; g_velocidad = 0;
  aplicar();          // forzado a LIBRE ANTES de que el timer arranque
  timerArrancar();

  webInit();
  ayuda();
  estado();
}

void loop() {
  g_web.handleClient();

  static String linea;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      g_mandaWeb = false;      // el mando vuelve al puerto serie
      procesar(linea);
      linea = "";
    }
    else if (linea.length() < 40) linea += c;
  }

  // Hombre muerto. Solo actua si el navegador tenia el mando y el motor
  // estaba girando: perder la conexion con el motor a fondo es la unica
  // situacion en la que uno no puede hacer nada mas que desenchufar.
  if (g_mandaWeb && g_modo != LIBRE && g_modo != FRENO &&
      millis() - g_ultimoContacto > VIGILIA_MS) {
    g_modo = LIBRE;
    g_velocidad = 0;
    aplicar();
    g_mandaWeb = false;
    Serial.println("  se perdio el navegador -> LIBRE por seguridad");
  }
}
