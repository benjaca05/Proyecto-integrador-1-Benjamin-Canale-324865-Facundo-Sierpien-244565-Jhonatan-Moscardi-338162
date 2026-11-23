/*
 * ROBOT LANZADOR - Proyecto entrega
 * ==================================
 *
 * Que hace el programa (una sola vez, al encender o reiniciar la placa):
 *   1. Gira sobre su eje buscando la canasta con el sensor ultrasonico.
 *   2. Se acerca o se aleja hasta quedar a 25 cm de la canasta.
 *      Si la pierde en el camino, la busca a la derecha y despues a la izquierda.
 *   3. Se vuelve a centrar para corregir la punteria (al avanzar se desvia un poco).
 *   4. Activa el motor lanzador durante 1 segundo.
 *
 * Hardware:
 *   - Arduino Uno.
 *   - Shield de motores con dos L293D (puentes H) y un registro de
 *     desplazamiento 74HC595 que maneja las entradas de direccion.
 *   - Dos motores DC de traccion: M1 y M4.
 *   - Sensor ultrasonico HC-SR04 (mide distancia por tiempo de vuelo del sonido).
 *   - Motor lanzador manejado por un driver externo, con la senal en A5.
 *
 * Idea clave de la deteccion:
 *   Salvo la canasta, no hay nada a menos de 90 cm del robot. Entonces, si el
 *   sensor mide algo a menos de 100 cm, eso es la canasta (su poste central).
 */

#include <Arduino.h>

// ============================================================================
// 1. PINES
// ============================================================================

// Senal PWM de velocidad de cada motor (van a los pines "enable" de los L293D).
const uint8_t PIN_PWM_M1 = 11;
const uint8_t PIN_PWM_M4 = 5;

// Registro de desplazamiento 74HC595: convierte un dato serie (bit a bit)
// en 8 salidas en paralelo, que son las entradas de direccion de los L293D.
const uint8_t PIN_REGISTRO_LATCH = 12;  // Pasa el dato cargado a las salidas.
const uint8_t PIN_REGISTRO_CLOCK = 4;   // Cada flanco carga un bit.
const uint8_t PIN_REGISTRO_DATO = 8;    // El bit que se esta cargando.
const uint8_t PIN_REGISTRO_ENABLE = 7;  // Activo en LOW: habilita las salidas.

// Encoders de las ruedas (conectados, todavia no se usan en el programa).
const uint8_t PIN_ENCODER_M1 = A4;
const uint8_t PIN_ENCODER_M4 = A2;

// Sensor ultrasonico HC-SR04.
const uint8_t PIN_ULTRASONIDO_TRIG = A0;  // Salida: pulso que dispara la medicion.
const uint8_t PIN_ULTRASONIDO_ECHO = A1;  // Entrada: pulso cuyo ancho es el tiempo de vuelo.

// Lanzador: A5 es una senal logica (0 V / 5 V) hacia el driver del motor.
// No alimenta al motor; la potencia la entrega el driver.
const uint8_t PIN_LANZADOR = A5;

// ============================================================================
// 2. DIRECCION DE LOS MOTORES
// ============================================================================
//
// Cada motor tiene un puente H con dos entradas (A y B):
//   A = 1, B = 0 -> gira para un lado
//   A = 0, B = 1 -> gira para el otro lado
// Esas entradas salen del 74HC595. Bits de cada motor en el byte enviado:
//   bit 0 = M4 A    bit 2 = M1 A
//   bit 6 = M4 B    bit 3 = M1 B
//
//                           76543210  (numero de bit)
const uint8_t PATRON_AVANCE = 0b00001001;      // M1 B + M4 A
const uint8_t PATRON_RETROCESO = 0b01000100;   // M1 A + M4 B (las dos invertidas)
const uint8_t PATRON_GIRO = 0b00000101;        // M1 A + M4 A (M1 invertido respecto a avanzar)
const uint8_t PATRON_GIRO_INVERSO = 0b01001000; // M1 B + M4 B

// ============================================================================
// 3. PARAMETROS (los valores que se ajustan probando)
// ============================================================================

// Velocidad: ciclo de trabajo del PWM, de 0 (0 %) a 255 (100 %).
const int VELOCIDAD_AVANCE = 255;
const int VELOCIDAD_GIRO = 255;

// Sensor ultrasonico.
const float VELOCIDAD_SONIDO_CM_POR_US = 0.0343f; // 343 m/s = 0,0343 cm/us
const unsigned long TIEMPO_MAX_ECO_US = 25000UL;  // Si tarda mas, no hubo eco (~4,3 m).
const uint8_t LECTURAS_POR_MEDICION = 7;
const uint8_t LECTURAS_CERCANAS_MINIMAS = 3;       // De las 7, cuantas deben ver la canasta.
const unsigned long PAUSA_ENTRE_LECTURAS_MS = 40UL; // Deja que se apague el eco anterior.
const float DISTANCIA_MAXIMA_CANASTA_CM = 100.0f;  // Mas cerca que esto = canasta.

// Busqueda girando.
const unsigned long TIEMPO_PULSO_GIRO_MS = 80UL;   // Cuanto gira en cada paso.
const unsigned long PAUSA_DESPUES_DE_MOVER_MS = 200UL; // Espera a que el robot se quiete.
const uint8_t PASOS_MAXIMOS_BUSQUEDA = 120;
const uint8_t PULSOS_EXTRA_CENTRADO = 2;
const uint8_t BUSQUEDAS_MAXIMAS = 3;               // Si pierde la canasta al acercarse.
const uint8_t PASOS_BUSQUEDA_LATERAL = 10;         // Cuanto mira hacia cada costado.

// Sentido de giro. Si al probar el robot gira al reves de lo que dice el
// monitor, cambiar GIRO_DERECHA a true.
const bool GIRO_DERECHA = false;
const bool GIRO_IZQUIERDA = !GIRO_DERECHA;

// Acercamiento a la canasta.
const float DISTANCIA_OBJETIVO_CM = 25.0f;
const float TOLERANCIA_CM = 5.0f;                  // Acepta entre 20 y 30 cm.
const float ERROR_GRANDE_CM = 15.0f;               // Desde aca usa el pulso largo.
const unsigned long PULSO_AVANCE_CORTO_MS = 100UL; // Cerca del objetivo.
const unsigned long PULSO_AVANCE_LARGO_MS = 200UL; // Lejos del objetivo.
const unsigned long PULSO_ANTES_DE_TIRAR_MS = PULSO_AVANCE_CORTO_MS; // Ultimo avance antes de disparar.
const uint8_t PULSOS_ANTES_DE_TIRAR = 3;         // Cuantos pulsos avanza antes de disparar.
const unsigned long TIEMPO_MAX_ACERCAMIENTO_MS = 15000UL;

// Lanzador.
const unsigned long TIEMPO_LANZADOR_MS = 1000UL;

// ============================================================================
// 4. MOTORES
// ============================================================================

// Envia el byte de direccion al 74HC595.
// shiftOut() saca los 8 bits por DATO, con un pulso de CLOCK por bit.
// Mientras tanto LATCH esta en LOW, asi las salidas no cambian a medias;
// al ponerlo en HIGH, los 8 bits aparecen juntos en las salidas.
void escribirDireccion(uint8_t patron)
{
  digitalWrite(PIN_REGISTRO_LATCH, LOW);
  shiftOut(PIN_REGISTRO_DATO, PIN_REGISTRO_CLOCK, MSBFIRST, patron);
  digitalWrite(PIN_REGISTRO_LATCH, HIGH);
}

// analogWrite() genera PWM: la tension media en el motor es aproximadamente
// (pwm / 255) x tension de la bateria, y de eso depende su velocidad.
void aplicarVelocidad(int pwm)
{
  analogWrite(PIN_PWM_M1, pwm);
  analogWrite(PIN_PWM_M4, pwm);
}

void parar()
{
  aplicarVelocidad(0);
}

void avanzar()
{
  escribirDireccion(PATRON_AVANCE);
  aplicarVelocidad(VELOCIDAD_AVANCE);
}

void retroceder()
{
  escribirDireccion(PATRON_RETROCESO);
  aplicarVelocidad(VELOCIDAD_AVANCE);
}

// Un "paso" de giro: mueve los motores un instante, frena y espera a que el
// robot quede quieto. Medir con el robot quieto da lecturas mas estables.
void girarUnPaso(bool inverso)
{
  escribirDireccion(inverso ? PATRON_GIRO_INVERSO : PATRON_GIRO);
  aplicarVelocidad(VELOCIDAD_GIRO);
  delay(TIEMPO_PULSO_GIRO_MS);
  parar();
  delay(PAUSA_DESPUES_DE_MOVER_MS);
}

// ============================================================================
// 5. SENSOR ULTRASONICO
// ============================================================================

// Una medicion del HC-SR04:
//   1. Un pulso de 10 us en TRIG hace que el sensor emita sonido (40 kHz).
//   2. El sensor pone ECHO en HIGH hasta que vuelve el rebote.
//   3. El ancho de ese pulso es el tiempo de ida y vuelta del sonido:
//        distancia = tiempo x velocidad del sonido / 2   (se divide por 2 por la ida y la vuelta)
// Devuelve false si no volvio eco (no hay nada al alcance).
bool medirDistancia(float &distanciaCm)
{
  digitalWrite(PIN_ULTRASONIDO_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIDO_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIDO_TRIG, LOW);

  // pulseIn mide cuantos microsegundos esta ECHO en HIGH (0 si se paso del limite).
  const unsigned long tiempoUs = pulseIn(PIN_ULTRASONIDO_ECHO, HIGH, TIEMPO_MAX_ECO_US);
  if (tiempoUs == 0)
  {
    return false;
  }

  distanciaCm = tiempoUs * VELOCIDAD_SONIDO_CM_POR_US / 2.0f;
  return true;
}

// Decide si la canasta esta enfrente.
// El poste es angosto: a veces solo algunas de las 7 lecturas lo ven y las
// demas rebotan en una pared lejana. Por eso:
//   - se guardan solo las lecturas cercanas (menos de 100 cm),
//   - si hay al menos 3, la canasta esta enfrente,
//   - la distancia es la MEDIANA de esas lecturas (el valor del medio una vez
//     ordenadas). A diferencia del promedio, una lectura erronea no la mueve.
bool verCanasta(float &distanciaCm)
{
  float cercanas[LECTURAS_POR_MEDICION];
  uint8_t cantidad = 0;

  for (uint8_t i = 0; i < LECTURAS_POR_MEDICION; ++i)
  {
    float lecturaCm = 0.0f;
    if (medirDistancia(lecturaCm) && lecturaCm <= DISTANCIA_MAXIMA_CANASTA_CM)
    {
      // Se inserta la lectura en orden, de menor a mayor (como ordenar cartas
      // en la mano): se corren hacia la derecha las que son mas grandes.
      uint8_t posicion = cantidad;
      while (posicion > 0 && cercanas[posicion - 1] > lecturaCm)
      {
        cercanas[posicion] = cercanas[posicion - 1];
        --posicion;
      }
      cercanas[posicion] = lecturaCm;
      ++cantidad;
    }
    delay(PAUSA_ENTRE_LECTURAS_MS);
  }

  if (cantidad < LECTURAS_CERCANAS_MINIMAS)
  {
    return false;
  }
  distanciaCm = cercanas[cantidad / 2]; // El del medio = mediana.
  return true;
}

void imprimirLectura(bool veCanasta, float distanciaCm)
{
  if (veCanasta)
  {
    Serial.print(distanciaCm);
    Serial.print(" cm");
  }
  else
  {
    Serial.print("nada cerca");
  }
}

// ============================================================================
// 6. BUSQUEDA DE LA CANASTA
// ============================================================================
//
// El robot gira de a pasos y en cada paso se fija si ve la canasta:
//
//   paso:     1      2      3      4      5      6
//   ve:      no     SI     SI     SI     no
//                   ^ empieza       ^ termina
//
// La canasta se vio en 3 pasos (2, 3 y 4); su centro es el paso 3. El robot
// esta en el paso 5, asi que vuelve 2 pasos girando al reves.
// En general vuelve (pasos vistos + 1) / 2 pasos.
// Se cuentan pasos y no tiempo porque el robot solo gira durante el pulso.

// Vuelve hasta el centro de la canasta y confirma que la ve.
bool centrarEnCanasta(uint16_t pasosViendoCanasta, bool busquedaInversa)
{
  const bool volverInverso = !busquedaInversa; // Vuelve al reves de como busco.
  const uint16_t pasosParaVolver = (pasosViendoCanasta + 1) / 2;

  Serial.print("Volviendo ");
  Serial.print(pasosParaVolver);
  Serial.println(" pasos para centrar.");
  for (uint16_t i = 0; i < pasosParaVolver; ++i)
  {
    girarUnPaso(volverInverso);
  }

  // Al cambiar de sentido, el primer paso puede girar menos por el juego de
  // los engranajes. Si no ve la canasta, prueba hasta 2 pasos mas.
  for (uint8_t extra = 0; extra <= PULSOS_EXTRA_CENTRADO; ++extra)
  {
    if (extra > 0)
    {
      Serial.println("No se ve la canasta al frente; un paso mas de vuelta.");
      girarUnPaso(volverInverso);
    }

    float distanciaCm = 0.0f;
    if (verCanasta(distanciaCm))
    {
      Serial.print("Canasta al frente a ");
      Serial.print(distanciaCm);
      Serial.println(" cm.");
      return true;
    }
  }

  Serial.println("Despues de centrar no se ve la canasta al frente.");
  return false;
}

// Gira hasta encontrar la canasta completa (de un borde al otro) y se centra.
// inverso elige el sentido (GIRO_DERECHA o GIRO_IZQUIERDA).
// pasosMaximos limita cuanto gira buscando el primer borde; una vez que la
// encontro, siempre termina de recorrerla.
bool buscarCanasta(bool inverso, uint8_t pasosMaximos)
{
  Serial.print("Busqueda girando a la ");
  Serial.println(inverso == GIRO_DERECHA ? "derecha." : "izquierda.");

  float distanciaCm = 0.0f;
  bool veiaCanasta = verCanasta(distanciaCm);
  Serial.print("Paso 0: ");
  imprimirLectura(veiaCanasta, distanciaCm);
  Serial.println();

  // Si arranca viendo la canasta no sabe donde esta su primer borde.
  // Entonces gira al reves hasta dejar de verla: asi el primer borde le
  // queda adelante y lo puede contar.
  if (veiaCanasta)
  {
    Serial.println("Arranca viendo la canasta: gira al reves hasta pasar el borde.");
    uint8_t pasosAtras = 0;
    while (veiaCanasta)
    {
      if (pasosAtras == PASOS_MAXIMOS_BUSQUEDA)
      {
        Serial.println("Nunca dejo de ver la canasta; se cancela la busqueda.");
        return false;
      }
      girarUnPaso(!inverso);
      ++pasosAtras;
      veiaCanasta = verCanasta(distanciaCm);
    }
    Serial.print("Borde pasado despues de ");
    Serial.print(pasosAtras);
    Serial.println(" pasos. Ahora si se cuenta.");
  }

  // Aca el robot NO ve la canasta: la proxima vez que la vea es el primer borde.
  uint16_t pasoInicio = 0;

  for (uint16_t paso = 1;; ++paso)
  {
    if (!veiaCanasta && paso > pasosMaximos)
    {
      Serial.println("Se llego al limite de pasos sin encontrar la canasta.");
      return false;
    }
    if (veiaCanasta && paso - pasoInicio > PASOS_MAXIMOS_BUSQUEDA)
    {
      Serial.println("La canasta nunca termina; se cancela la busqueda.");
      return false;
    }

    girarUnPaso(inverso);
    const bool veCanasta = verCanasta(distanciaCm);

    Serial.print("Paso ");
    Serial.print(paso);
    Serial.print(": ");
    imprimirLectura(veCanasta, distanciaCm);

    if (veCanasta && !veiaCanasta)
    {
      // Antes no la veia y ahora si: primer borde.
      pasoInicio = paso;
      Serial.print("  <- empieza la canasta");
    }
    else if (!veCanasta && veiaCanasta)
    {
      // Antes la veia y ahora no: segundo borde.
      const uint16_t pasosViendoCanasta = paso - pasoInicio;
      Serial.print("  <- termina la canasta, la vio en ");
      Serial.print(pasosViendoCanasta);
      Serial.println(" pasos");
      return centrarEnCanasta(pasosViendoCanasta, inverso);
    }

    Serial.println();
    veiaCanasta = veCanasta;
  }
}

// Busca la canasta cuando deberia estar cerca del frente (se perdio al
// acercarse, o hay que recentrar) sin dar una vuelta entera:
//   1. Mira a la derecha.
//   2. Si no esta, gira a la izquierda el doble (vuelve al frente y sigue).
//   3. Solo si tampoco esta, hace una busqueda completa.
// Si arranca viendola, buscarCanasta() ya se encarga de pasar el borde.
bool buscarCanastaCerca()
{
  Serial.println("Buscando la canasta cerca del frente: primero a la derecha.");
  if (buscarCanasta(GIRO_DERECHA, PASOS_BUSQUEDA_LATERAL))
  {
    return true;
  }

  Serial.println("No estaba a la derecha: se busca a la izquierda.");
  if (buscarCanasta(GIRO_IZQUIERDA, 2 * PASOS_BUSQUEDA_LATERAL))
  {
    return true;
  }

  Serial.println("No estaba cerca del frente: busqueda completa.");
  return buscarCanasta(GIRO_IZQUIERDA, PASOS_MAXIMOS_BUSQUEDA);
}

// ============================================================================
// 7. ACERCAMIENTO A 25 cm
// ============================================================================
//
// Control simple "medir -> mover -> medir":
//   error = distancia medida - 25 cm
//   error > +5 cm -> esta lejos: avanza
//   error < -5 cm -> esta cerca: retrocede
//   si no           -> listo
// Si el error es grande, mueve un pulso largo (llega rapido); si es chico,
// un pulso corto (no se pasa de largo).

// Recuerdan como fue el PRIMER movimiento del acercamiento. Si arranco muy
// cerca y tuvo que retroceder, despues no se avanzan los pulsos antes de tirar.
bool yaSeMovioAlAcercarse = false;
bool empezoRetrocediendo = false;

bool acercarseALaCanasta()
{
  Serial.println("Ajustando distancia a la canasta.");
  const unsigned long inicioMs = millis();

  while (millis() - inicioMs < TIEMPO_MAX_ACERCAMIENTO_MS)
  {
    float distanciaCm = 0.0f;
    if (!verCanasta(distanciaCm))
    {
      Serial.println("Se perdio la canasta mientras se ajustaba la distancia.");
      return false;
    }

    const float errorCm = distanciaCm - DISTANCIA_OBJETIVO_CM;

    Serial.print("Distancia: ");
    Serial.print(distanciaCm);
    if (errorCm > TOLERANCIA_CM)
    {
      Serial.println(" cm -> avanza");
      avanzar();
      yaSeMovioAlAcercarse = true;
    }
    else if (errorCm < -TOLERANCIA_CM)
    {
      Serial.println(" cm -> retrocede");
      retroceder();
      if (!yaSeMovioAlAcercarse)
      {
        empezoRetrocediendo = true; // Su primer movimiento fue hacia atras.
      }
      yaSeMovioAlAcercarse = true;
    }
    else
    {
      Serial.println(" cm -> distancia correcta");
      return true;
    }

    delay(fabs(errorCm) > ERROR_GRANDE_CM ? PULSO_AVANCE_LARGO_MS
                                          : PULSO_AVANCE_CORTO_MS);
    parar();
    delay(PAUSA_DESPUES_DE_MOVER_MS);
  }

  Serial.println("Se acabo el tiempo para ajustar la distancia.");
  return false;
}

// ============================================================================
// 8. LANZADOR
// ============================================================================

// Pone A5 en HIGH (5 V) durante 1 segundo: el driver enciende el motor lanzador.
void activarLanzador()
{
  digitalWrite(PIN_LANZADOR, HIGH);
  delay(TIEMPO_LANZADOR_MS);
  digitalWrite(PIN_LANZADOR, LOW);
}

// ============================================================================
// 9. PROGRAMA PRINCIPAL
// ============================================================================

void setup()
{
  Serial.begin(9600); // Para ver los mensajes en el Monitor Serie.

  pinMode(PIN_PWM_M1, OUTPUT);
  pinMode(PIN_PWM_M4, OUTPUT);
  pinMode(PIN_REGISTRO_LATCH, OUTPUT);
  pinMode(PIN_REGISTRO_CLOCK, OUTPUT);
  pinMode(PIN_REGISTRO_DATO, OUTPUT);
  pinMode(PIN_REGISTRO_ENABLE, OUTPUT);

  pinMode(PIN_ENCODER_M1, INPUT_PULLUP);
  pinMode(PIN_ENCODER_M4, INPUT_PULLUP);

  pinMode(PIN_ULTRASONIDO_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIDO_ECHO, INPUT);
  digitalWrite(PIN_ULTRASONIDO_TRIG, LOW);

  pinMode(PIN_LANZADOR, OUTPUT);
  digitalWrite(PIN_LANZADOR, LOW);

  digitalWrite(PIN_REGISTRO_ENABLE, LOW); // Habilita las salidas del 74HC595.
  parar();
  escribirDireccion(PATRON_AVANCE);
}

// loop() se repite siempre, pero la prueba se hace una sola vez:
// la variable "static" conserva su valor entre repeticiones.
void loop()
{
  static bool pruebaHecha = false;
  if (pruebaHecha)
  {
    return;
  }
  pruebaHecha = true;

  Serial.println("Prueba: buscar, ajustar a 25 cm, volver a centrar y disparar.");

  // Paso 1: buscar la canasta (puede estar en cualquier lado: vuelta completa).
  if (!buscarCanasta(GIRO_DERECHA, PASOS_MAXIMOS_BUSQUEDA))
  {
    Serial.println("No se encontro la canasta; no se activo el lanzador.");
    return;
  }

  // Paso 2: quedar a 25 cm. Si pierde la canasta, seguro quedo a un costado:
  // la busca a la derecha y despues a la izquierda, sin dar la vuelta entera.
  uint8_t busquedas = 1;
  while (!acercarseALaCanasta())
  {
    if (busquedas == BUSQUEDAS_MAXIMAS)
    {
      Serial.println("Se perdio la canasta demasiadas veces; no se activo el lanzador.");
      return;
    }
    ++busquedas;
    if (!buscarCanastaCerca())
    {
      Serial.println("No se encontro la canasta; no se activo el lanzador.");
      return;
    }
  }

  // Paso 3: al avanzar el robot se desvia un poco, asi que se vuelve a centrar.
  Serial.println("Recalibrando.");
  if (!buscarCanastaCerca())
  {
    Serial.println("No se encontro la canasta al recalibrar; no se activo el lanzador.");
    return;
  }

  // Paso 4: unos pulsos cortos hacia adelante y disparar.
  // Si al principio tuvo que retroceder (arranco muy cerca), no avanza.
  if (empezoRetrocediendo)
  {
    Serial.println("Canasta centrada a 25 cm; arranco retrocediendo, asi que no avanza.");
  }
  else
  {
    Serial.print("Canasta centrada a 25 cm; avanza ");
    Serial.print(PULSOS_ANTES_DE_TIRAR);
    Serial.println(" pulsos.");
    for (uint8_t i = 0; i < PULSOS_ANTES_DE_TIRAR; ++i)
    {
      avanzar();
      delay(PULSO_ANTES_DE_TIRAR_MS);
      parar();
      delay(PAUSA_DESPUES_DE_MOVER_MS); // Que quede quieto antes de seguir o tirar.
    }
  }

  Serial.println("Disparando por 1 segundo.");
  activarLanzador();
  Serial.println("Disparo finalizado.");
}
