#include <Arduino.h>
#include <ControlRuedas.h>
#include <Preferences.h>
#include <LittleFS.h>
#include "sensores.h"

// ETAPA 1: calibración del giro sobre el propio eje (tirones).
// ETAPA 2: escaneo ultrasónico (tecla 'e') para registrar cómo ve el sensor al aro.
// ETAPA 4: búsqueda del aro por su patrón bajada-mantenimiento-subida (tecla 'b').
// La versión anterior (búsqueda del aro) está en respaldo_main_busqueda_continua.txt.
//
// Comandos por el monitor serial. Cada tecla actúa al instante (sin Enter):
//   1 2 3 4  elige la rueda a ajustar (1=FL 2=FR 3=RL 4=RR)
//   5        elige las cuatro ruedas juntas
//   6        elige el porcentaje de tirones
//   +  /  -  sube o baja el PWM de la rueda elegida (de a PASO_PWM) o, con 6, el % de tirones
//   d  /  i  gira continuamente a la derecha / izquierda
//   o        activa / desactiva los tirones (PWM dentro de PWM)
//   u  /  j  sube o baja el % del período con las ruedas encendidas
//   b        busca el aro: espera 8 s, gira a la derecha, lo detecta, pasa de largo
//            y vuelve a la izquierda hasta apuntar a su centro
//   a        busca el aro, se acerca a 26 cm, gira a la izquierda hasta perderlo y
//            lo vuelve a buscar a la derecha para quedar centrado
//   w  /  z  avanza / retrocede de forma continua (sin tirones); 's' frena
//   m  /  n  sube / baja el PWM de la rueda elegida (1-4, o 5 = todas) del juego marcado con *
//            (avance o retroceso: el del último w / z apretado)
//   y        imprime los PWM de avance
//   c  /  r  un pulso de avance / retroceso (PULSO_AVANCE_MS): mide la distancia antes y después
//   q        prueba de simetría: TIRONES_PRUEBA tirones a la derecha, pausa y otie partida)
//   e        escaneo: espera 7 s, gira NUM_VUELTAS midiendo, frena e imprime el CSV
//   x        prueba del sensor: promedio de las lecturas de cada medio segundo (x otra vez, para)
//   v        imprime el registro guardado de todo lo que salió por el monitor (queda en flash:
//            se puede desconectar el cable, correr la rutina y verlo después)
//   l        borra el registro
//   V        imprime el CSV del último escaneo (queda guardado en flash:
//            se puede desenchufar, volver a enchufar y apretar v)
//   s        detiene e imprime cuántos tirones dio y cuánto tardó desde que empezó a girar
//   p        imprime los PWM actuales
//   g  /  h  servo (GPIO 13) gira en un sentido / en el otro; f lo frena
//   [  /  ]  ajusta el punto de frenado del servo (±5 us) hasta que quede quieto
//   W / G    TEMPORAL: espera 5 s y avanza / gira sobre sí mismo 3 s a toda velocidad
//   L        lanzar la pelota (motor 8 s, servo suelta, a los 2 s frena el motor y el servo vuelve);
//            también se hace solo al terminar 'a' (centrado y a 26 cm). 's' interrumpe
//   ,  /  .  baja / sube la potencia del motor (de a 5 %); <  /  >  baja / sube el tiempo de giro del servo al lanzar
//   k  /  t  motor del mosfet (GPIO 14) prendido / apagado (lógica invertida: señal en alto = frenado)

// Todo lo que el programa "diría" por el monitor pasa por Salida: se envía al puerto serial y se
// guarda en un buffer en RAM. El buffer se vuelca al archivo REGISTRO (flash) cuando el robot está
// quieto, nunca en pleno movimiento (escribir en flash frena el programa unos milisegundos).
// Tecla v: imprime el registro guardado (sirve aunque se haya desconectado el cable).
class RegistroSalida : public Print {
public:
    static constexpr size_t TAMANO = 16384;
    bool registrar = true;

    size_t pendiente() const { return largo_; }
    size_t perdidos() const { return perdidos_; }
    const uint8_t* datos() const { return buffer_; }
    void vaciar() {
        largo_ = 0;
        perdidos_ = 0;
    }

    size_t write(uint8_t byte) override { return write(&byte, 1); }
    size_t write(const uint8_t* datos, size_t cantidad) override {
        Serial.write(datos, cantidad);
        if (registrar) {
            const size_t libre = TAMANO - largo_;
            const size_t copiar = cantidad < libre ? cantidad : libre;
            memcpy(buffer_ + largo_, datos, copiar);
            largo_ += copiar;
            perdidos_ += cantidad - copiar;
        }
        return cantidad;
    }

private:
    uint8_t buffer_[TAMANO];
    size_t largo_ = 0;
    size_t perdidos_ = 0;
};

RegistroSalida Salida;
constexpr const char* ARCHIVO_REGISTRO = "/registro.txt";
constexpr size_t MAX_BYTES_REGISTRO = 300000;  // al superarlo se borra y se empieza de nuevo
bool registroListo = false;
unsigned long ultimoGuardadoRegistro = 0;

ControlRuedas ruedas;

// ---- Servo y motor con mosfet (control manual por teclado) ----
constexpr int PIN_SERVO = 13;
constexpr int PIN_MOSFET = 14;

// Canales LEDC (núcleo Arduino-ESP32 2.x). analogWrite de las ruedas usa los canales 15 a 12,
// así que el servo (8) y el motor (2) no chocan con ellas.
constexpr int CANAL_SERVO = 8;
constexpr int SERVO_FRECUENCIA_HZ = 50;
constexpr int SERVO_RESOLUCION_BITS = 16;

constexpr int CANAL_MOTOR = 2;
constexpr int MOTOR_FRECUENCIA_HZ = 1000;
constexpr int MOTOR_RESOLUCION_BITS = 8;
constexpr int MOTOR_DUTY_MAX = 255;
int motorPotenciaPct = 60;  // potencia del motor (0-100 %), se ajusta con , y .
constexpr int MOTOR_PASO_PCT = 5;
bool motorPrendido = false;

// Servo de rotación continua: el ancho del pulso define velocidad y sentido.
// 1500 us = frenado (puede variar unos ±30 us según el servo).
int pulsoFrenoUs = 1500;
constexpr int SERVO_DIFERENCIA_US = 50;  // subirlo gira más rápido, bajarlo más lento

void servoPulso(int pulsoUs) {
    const uint32_t duty = (uint32_t)(((uint64_t)pulsoUs * ((1UL << SERVO_RESOLUCION_BITS) - 1)) /
                                     (1000000UL / SERVO_FRECUENCIA_HZ));
    ledcWrite(CANAL_SERVO, duty);
}

void servoGira(int sentidoServo) {
    const int pulso = pulsoFrenoUs + sentidoServo * SERVO_DIFERENCIA_US;
    servoPulso(pulso);
    Serial.printf("Servo: gira (%d us)\n", pulso);
}

void servoFrena() {
    servoPulso(pulsoFrenoUs);
    Serial.printf("Servo: frenado (%d us)\n", pulsoFrenoUs);
}

// Lógica invertida (NOT): señal en alto = motor frenado, señal en 0 = motor activo.
void motorAplicar() {
    const int duty = MOTOR_DUTY_MAX * motorPotenciaPct / 100;
    ledcWrite(CANAL_MOTOR, motorPrendido ? MOTOR_DUTY_MAX - duty : MOTOR_DUTY_MAX);
}

void motorOn() {
    motorPrendido = true;
    motorAplicar();
    Serial.printf("Motor: PRENDIDO (%d%%)\n", motorPotenciaPct);
}

void motorOff() {
    motorPrendido = false;
    motorAplicar();
    Serial.println("Motor: apagado");
}

// ---- Lanzamiento (tecla 'L', y al final de la tecla 'a') ----
// 1) prende el motor y espera 8 s a que llegue a su velocidad terminal; 2) el servo gira (sentido de 'g')
// el tiempo equivalente a ~80 grados y suelta la pelota; 3) a los LANZAR_ESPERA_SERVO_MS de empezar
// a girar el servo, apaga el motor y el servo vuelve a su posición inicial (mismo tiempo, sentido contrario).
// El servo es de rotación continua: los grados se logran por tiempo. Calibrar LANZAR_SERVO_MS
// con las teclas < y > hasta que gire unos 80 grados.
constexpr unsigned long LANZAR_ACELERAR_MS = 8000;
constexpr unsigned long LANZAR_ESPERA_SERVO_MS = 2000;
unsigned long lanzarServoMs = 350;
constexpr unsigned long LANZAR_SERVO_PASO_MS = 25;

// Espera ms milisegundos. Devuelve false si se apretó 's' (aborta).
bool esperarLanzamiento(unsigned long ms) {
    const unsigned long inicio = millis();
    while (millis() - inicio < ms) {
        if (Serial.available() > 0 && Serial.read() == 's') {
            return false;
        }
    }
    return true;
}

// Ajuste automático de la potencia según la distancia final al aro (solo en el lanzamiento
// automático): más lejos del objetivo = más potencia, más cerca = menos.
constexpr float LANZAR_PCT_POR_CM = 2.0f;  // puntos de potencia por cada cm de diferencia
constexpr float LANZAR_AJUSTE_MAX_PCT = 5.0f;  // tope del ajuste (el sensor varía ~±5 cm dentro del aro)
constexpr unsigned long ARRANQUE_AUTOMATICO_MS = 7000;  // espera al encender antes de arrancar solo
constexpr unsigned long LANZAR_ESPERA_PREVIA_MS = 4000;  // pausa antes del tiro automático

// distanciaCm < 0 = sin ajuste (tecla 'L').
void lanzar(float distanciaCm = -1, float objetivoCm = 0) {
    const int potenciaBase = motorPotenciaPct;
    if (distanciaCm >= 0) {
        const float ajuste = (distanciaCm - objetivoCm) * LANZAR_PCT_POR_CM;
        const float ajusteLimitado = constrain(ajuste, -LANZAR_AJUSTE_MAX_PCT, LANZAR_AJUSTE_MAX_PCT);
        motorPotenciaPct = constrain((int)lroundf(potenciaBase + ajusteLimitado), 0, 100);
        Salida.printf("Distancia %.1f cm (objetivo %.1f): potencia %d %% -> %d %%\n",
                      distanciaCm, objetivoCm, potenciaBase, motorPotenciaPct);
    }
    Salida.printf("Lanzamiento: motor al %d %%, servo %lu ms.\n", motorPotenciaPct, lanzarServoMs);
    motorOn();
    Salida.printf("Acelerando el motor %lu s...\n", LANZAR_ACELERAR_MS / 1000);

    unsigned long servoGiradoMs = 0;
    bool completo = esperarLanzamiento(LANZAR_ACELERAR_MS);

    if (completo) {
        Salida.println("Servo: suelta la pelota.");
        const unsigned long inicioServo = millis();
        servoGira(+1);
        completo = esperarLanzamiento(lanzarServoMs);
        servoGiradoMs = completo ? lanzarServoMs : millis() - inicioServo;
        servoFrena();

        if (completo) {
            const unsigned long yaPaso = millis() - inicioServo;
            completo = yaPaso >= LANZAR_ESPERA_SERVO_MS || esperarLanzamiento(LANZAR_ESPERA_SERVO_MS - yaPaso);
        }
    }

    motorOff();
    if (servoGiradoMs > 0) {
        Salida.println("Servo: vuelve a su posicion inicial.");
        servoGira(-1);
        delay(servoGiradoMs);
        servoFrena();
    }
    motorPotenciaPct = potenciaBase;  // el ajuste por distancia no se acumula
    Salida.println(completo ? "Lanzamiento terminado." : "Lanzamiento interrumpido.");
}

// PWM de cada rueda (0-255). Cambiar acá o en vivo con las teclas.
// Giro con solo dos ruedas opuestas: FR y RL activas, FL y RR en 0 (libres).
constexpr int PWM_DELANTERA_IZQUIERDA = 250;
constexpr int PWM_DELANTERA_DERECHA = 255;
constexpr int PWM_TRASERA_IZQUIERDA = 250;
constexpr int PWM_TRASERA_DERECHA = 255;
constexpr int PASO_PWM = 5;

// Prueba de simetría (tecla 'q').
constexpr unsigned long PRUEBA_ESPERA_MS = 5000;
constexpr unsigned long PRUEBA_PAUSA_MS = 2000;
constexpr int TIRONES_PRUEBA = 30;

// Tirones (tecla 'o'): "PWM dentro de PWM". Cada PERIODO_TIRON_MS las ruedas reciben
// su PWM durante tironPorcentaje % del período y quedan en 0 el resto.
constexpr unsigned long PERIODO_TIRON_MS = 600;
// Período vigente (la búsqueda lo alarga para dejar más pausa entre tirones; ver buscarAro).
unsigned long periodoTironMs = PERIODO_TIRON_MS;
constexpr float PASO_TIRON = 2.5f;
constexpr float TIRON_PORCENTAJE_INICIAL = 10.0f;

// Escaneo (tecla 'e'). El ángulo se estima suponiendo velocidad angular constante:
// ángulo = 360 * tiempo / PERIODO_VUELTA_MS. El período sale de las vueltas
// cronometradas con estos PWM y estos tirones (30511, 27613 y 26050 ms; 51, 47 y 44 tirones).
constexpr unsigned long ESCANEO_ESPERA_MS = 12000;
constexpr unsigned long PERIODO_VUELTA_MS = 28000;
constexpr int NUM_VUELTAS = 2;
constexpr unsigned long INTERVALO_MEDICION_MS = 65;
constexpr int MAX_MEDICIONES = 900;
static_assert(NUM_VUELTAS * PERIODO_VUELTA_MS < 65535, "el tiempo se guarda en 16 bits");
static_assert(MAX_MEDICIONES * 4 < 4000, "el bloque guardado en flash debe ser menor a 4000 bytes");

// Búsqueda del aro (tecla 'b'). Se toma una medida por tirón: mediana de
// DISPAROS_POR_MEDIDA disparos en la pausa (se usa la mediana, no el promedio), ASENTAR_MS después de que termina el tirón.
// Distancias en cm. Como hay tirones de dos tamaños, los anchos se miden en grados
// acumulados y no en cantidad de tirones.
// Período de los tirones durante la búsqueda: más largo que PERIODO_TIRON_MS, así el robot
// gira más lento y queda más pausa para medir. El pulso (ms) no cambia, solo la pausa.
constexpr unsigned long PERIODO_BUSQUEDA_MS = 800;
constexpr unsigned long ESPERA_BUSQUEDA_MS = 7000;
constexpr int DISPAROS_POR_MEDIDA = 5;
constexpr unsigned long ASENTAR_MS = 150;
constexpr float CAIDA_CM = 25.0f;           // bajada (o subida) que cuenta como cambio
constexpr float CAIDA_INICIO_CM = 10.0f;    // bajada mínima para sospechar del aro (ya debe estar a < 1 m)
constexpr float TOLERANCIA_ARO_CM = 15.0f;  // variación aceptada hacia más lejos dentro del aro
constexpr float TOLERANCIA_ARO_CERCA_CM = 40.0f;  // variación aceptada hacia más cerca (partes del aro más próximas)
constexpr float DISTANCIA_MAX_ARO_CM = 100.0f;  // el aro puede verse hasta ~95 cm (se midió a 89-95 cm); 100 cm o más no es el aro. Las paredes de 83-100 cm las descarta el filtro de ancho
constexpr int MEDIDAS_PARA_CONFIRMAR_ARO = 4;
constexpr int MEDIDAS_PARA_CONFIRMAR_FIN = 3;
constexpr int MEDIDAS_PARA_CONFIRMAR_SUBIDA = 2;  // medidas altas seguidas tras un nivel estable cercano
constexpr int MEDIDAS_PARA_PERDER_ARO = 3;       // medidas seguidas con salto que indican que se dejó de ver el aro
constexpr int FALLOS_SEGUIDOS_MAX = 2;      // medidas atípicas seguidas que cambian el estado
constexpr float MAX_GIRO_BUSQUEDA_GRADOS = 720.0f;  // 2 vueltas

// Tirón grueso: el porcentaje vigente al apretar 'b' (10 %: 47 tirones por vuelta).
// Tirón fino: se usa desde que se sospecha del aro (antes 7,5 %, 80,5 tirones por vuelta; subido a 10 % porque el robot pesa más).
constexpr float TIRON_PORCENTAJE_FINO = 10.0f;
constexpr float ANGULO_TIRON_GRUESO = 7.6f;
constexpr float ANGULO_TIRON_FINO = 7.6f;  // el pulso fino (60 ms en la búsqueda) es igual al grueso: mismo ángulo. Medir con el peso actual
// Prueba 5 con tirón fino: con 30 tirones por lado, la izquierda se pasó 7 y 9 tirones
// (~27 %). Con tirón grueso fue ~12 %. Poner 1.0 para desactivar la corrección.
constexpr float RELACION_GIRO_IZQUIERDA_FINO = 1.12f;  // con pulso de 60 ms (con 45 ms era 1,27)

// Ancho geométrico del aro, solo informativo (el sensor lo ve más ancho por su haz).
constexpr float ANCHO_ARO_CM = 20.0f;
// Ancho máximo aceptado para el aro. Lo que se ve más ancho se descarta (una pared, por ejemplo).
// El aro se midió entre ~97 y ~133 grados; se imprime el ancho observado en cada detección.
constexpr float ANCHO_MAX_ARO_GRADOS = 160.0f;
// Más cerca el aro ocupa más grados (a 23 cm se midieron ~160): hasta ANCHO_CERCA_CM se acepta más ancho.
// Las paredes que se confundieron con el aro estaban a 83-100 cm.
constexpr float ANCHO_MAX_ARO_CERCA_GRADOS = 240.0f;
constexpr float ANCHO_CERCA_CM = 60.0f;
float anchoMaxAro(float distanciaCm) {
    return distanciaCm < ANCHO_CERCA_CM ? ANCHO_MAX_ARO_CERCA_GRADOS : ANCHO_MAX_ARO_GRADOS;
}

int pwm[4] = {
    PWM_DELANTERA_IZQUIERDA,
    PWM_DELANTERA_DERECHA,
    PWM_TRASERA_IZQUIERDA,
    PWM_TRASERA_DERECHA};

// PWM de cada rueda al avanzar y retroceder (teclas w / z, ajuste con m / n).
// Aparte de los de giro porque cada rueda necesita su propia compensación para ir recto.
int pwmAvance[4] = {110, 170, 110, 150};  // ruedas derechas (FR, RR) +10: se desviaba hacia un lado
// PWM al retroceder: al ir hacia atrás el robot se desviaba, así que RL (trasera izquierda, 130)
// gira más rápido que al avanzar. Ajustar con w / z + m / n hasta que vaya recto.
int pwmRetroceso[4] = {110, 145, 140, 125};
// Cuál de los dos juegos ajustan m / n: el del último w (2) o z (-2) apretado.
int sentidoAjuste = 2;

// 0 = detenido, 1 = girando a la derecha, -1 = girando a la izquierda,
// 2 = avanzando, -2 = retrocediendo.
int sentido = 0;

// Se pone en true cuando girarTirones se interrumpe con 's'.
bool giroInterrumpido = false;

// Vuelca el buffer al archivo de flash. Llamar solo con el robot quieto.
void guardarRegistro() {
    ultimoGuardadoRegistro = millis();
    if (!registroListo || (Salida.pendiente() == 0 && Salida.perdidos() == 0)) {
        return;
    }

    File archivo = LittleFS.open(ARCHIVO_REGISTRO, FILE_APPEND);
    if (archivo && archivo.size() + Salida.pendiente() > MAX_BYTES_REGISTRO) {
        archivo.close();
        LittleFS.remove(ARCHIVO_REGISTRO);
        archivo = LittleFS.open(ARCHIVO_REGISTRO, FILE_WRITE);
        if (archivo) {
            archivo.println("[registro lleno: se borro lo anterior]");
        }
    }
    if (!archivo) {
        return;  // no se pudo abrir: se conserva el buffer para reintentar
    }

    archivo.write(Salida.datos(), Salida.pendiente());
    if (Salida.perdidos() > 0) {
        archivo.print("\n[faltan ");
        archivo.print(Salida.perdidos());
        archivo.println(" bytes: el buffer se lleno mientras el robot se movia]");
    }
    archivo.close();
    Salida.vaciar();
}

void guardarRegistroSiQuieto() {
    if (sentido == 0) {
        guardarRegistro();
    }
}

void imprimirRegistro() {
    guardarRegistroSiQuieto();
    Serial.println("# INICIO_REGISTRO");

    File archivo = registroListo ? LittleFS.open(ARCHIVO_REGISTRO, FILE_READ) : File();
    if (!archivo) {
        Serial.println("# (vacio)");
    } else {
        uint8_t bloque[256];
        while (archivo.available()) {
            const size_t leidos = archivo.read(bloque, sizeof(bloque));
            Serial.write(bloque, leidos);
        }
        archivo.close();
    }

    Serial.println("\n# FIN_REGISTRO");
}

void borrarRegistro() {
    if (registroListo) {
        LittleFS.remove(ARCHIVO_REGISTRO);
    }
    Salida.vaciar();
    Serial.println("Registro borrado.");
}

bool tironesActivos = true;
float tironPorcentaje = TIRON_PORCENTAJE_INICIAL;
bool ruedasEncendidas = true;

// Conteo de tirones desde el inicio del giro (el primero cuenta al arrancar).
unsigned long inicioTirones = 0;
unsigned long inicioGiroMs = 0;
int tironesContados = 0;

// Mediciones del escaneo, compactas para guardarlas en flash (4 bytes cada una):
// tiempo desde el inicio del giro en ms y distancia en décimas de cm.
constexpr uint16_t SIN_ECO = 0xFFFF;
struct Medicion {
    uint16_t tiempoMs;
    uint16_t distanciaDecimasCm;
};
Medicion mediciones[MAX_MEDICIONES];
int cantidadMediciones = 0;

// Configuración con la que se hizo el último escaneo (se guarda junto a los datos).
int pwmEscaneo[4];
float tironEscaneo = 0;

Preferences memoria;

void guardarEscaneo() {
    memoria.begin("escaneo", false);
    memoria.putUShort("n", cantidadMediciones);
    memoria.putBytes("med", mediciones, cantidadMediciones * sizeof(Medicion));
    memoria.putBytes("pwm", pwmEscaneo, sizeof(pwmEscaneo));
    memoria.putFloat("tiron2", tironEscaneo);
    memoria.end();
}

void cargarEscaneo() {
    memoria.begin("escaneo", false);
    cantidadMediciones = min<int>(memoria.getUShort("n", 0), MAX_MEDICIONES);
    memoria.getBytes("med", mediciones, cantidadMediciones * sizeof(Medicion));
    memoria.getBytes("pwm", pwmEscaneo, sizeof(pwmEscaneo));
    tironEscaneo = memoria.getFloat("tiron2", 0);
    memoria.end();
}

// PWM que se suma a todas las ruedas al avanzar (solo durante el impulso para destrabarse).
int impulsoAvanceExtra = 0;

void aplicarGiro() {
    ruedasEncendidas = true;
    if (sentido == 2) {
        ruedas.establecerVelocidades(pwmAvance[0] + impulsoAvanceExtra, pwmAvance[1] + impulsoAvanceExtra,
                                     pwmAvance[2] + impulsoAvanceExtra, pwmAvance[3] + impulsoAvanceExtra);
    } else if (sentido == -2) {
        ruedas.establecerVelocidades(pwmRetroceso[0], pwmRetroceso[1], pwmRetroceso[2], pwmRetroceso[3]);
    } else {
        ruedas.establecerVelocidades(pwm[0], pwm[1], pwm[2], pwm[3]);
    }

    if (sentido == 1) {
        ruedas.girarDerecha();
    } else if (sentido == -1) {
        ruedas.girarIzquierda();
    } else if (sentido == 2) {
        ruedas.avanzar();
    } else if (sentido == -2) {
        ruedas.retroceder();
    } else {
        ruedas.detener();
    }
}

// Alterna las ruedas entre su PWM y 0 según el período y el porcentaje de tirones.
void actualizarTirones() {
    if (!tironesActivos || sentido == 0 || sentido == 2 || sentido == -2) {
        return;
    }

    const bool encendido =
        ((millis() - inicioTirones) % periodoTironMs) < periodoTironMs * tironPorcentaje / 100.0f;

    if (encendido != ruedasEncendidas) {
        if (encendido) {
            tironesContados++;
            aplicarGiro();
        } else {
            ruedasEncendidas = false;
            ruedas.detener();
        }
    }
}

// Empieza a girar en el sentido indicado (1 = derecha, -1 = izquierda).
void iniciarGiro(int nuevoSentido) {
    sentido = nuevoSentido;
    inicioTirones = millis();
    inicioGiroMs = inicioTirones;
    tironesContados = tironesActivos ? 1 : 0;
    aplicarGiro();
}

// Gira exactamente "cantidad" tirones (1 = derecha, -1 = izquierda) y frena.
// Con 's' se interrumpe.
void girarTirones(int direccion, int cantidad) {
    giroInterrumpido = false;
    iniciarGiro(direccion);

    while (tironesContados < cantidad || ruedasEncendidas) {
        actualizarTirones();

        if (Serial.available() > 0 && Serial.read() == 's') {
            giroInterrumpido = true;
            break;
        }
    }

    sentido = 0;
    aplicarGiro();
}

// Prueba de simetría: ida y vuelta con la misma cantidad de tirones.
void pruebaSimetria() {
    if (!tironesActivos) {
        Salida.println("Activar los tirones con 'o' antes de la prueba.");
        return;
    }

    Salida.print("Prueba: ");
    Salida.print(TIRONES_PRUEBA);
    Salida.println(" tirones a la derecha y otros tantos a la izquierda. Empieza en 5 s.");
    delay(PRUEBA_ESPERA_MS);
    girarTirones(1, TIRONES_PRUEBA);
    Salida.println("Derecha lista.");
    delay(PRUEBA_PAUSA_MS);
    girarTirones(-1, TIRONES_PRUEBA);
    Salida.println("Prueba terminada.");
}

// ---- Búsqueda del aro ----

enum class EstadoBusqueda { BUSCANDO, POSIBLE_ARO, EN_ARO };

EstadoBusqueda estadoBusqueda = EstadoBusqueda::BUSCANDO;
float historial[3];            // últimas 3 medidas (para estimar el fondo con la mediana)
int cantidadHistorial = 0;
float nivelAroCm = 0;          // distancia media a la que se ve el aro
int medidasBajas = 0;          // medidas parecidas al nivel del aro
int fallosSeguidos = 0;        // medidas atípicas seguidas
float anguloActual = 0;        // grados girados desde que empezó la búsqueda
float anguloInicioAro = 0;     // ángulo de la primera medida baja
float anguloFinAro = 0;        // ángulo de la primera medida alta tras el aro
int medidasAltas = 0;
int fallosAltos = 0;
bool tironFino = false;
int subidasSeguidas = 0;     // medidas seguidas que subieron respecto de un nivel cercano estable
float nivelPrevioCm = 0;     // ese nivel cercano
float anguloSubida = 0;      // ángulo donde empezó la subida
bool empezoDentro = false;  // la búsqueda empezó ya apuntando a algo a menos de DISTANCIA_MAX_ARO_CM
float porcentajeGrueso = 0;    // porcentaje de tirón con el que empezó la búsqueda
char mensajeBusqueda[160];     // evento ocurrido con la última medida ("" si no hubo)

float medianaDeTres(float a, float b, float c) {
    return max(min(a, b), min(max(a, b), c));
}

const char* nombreEstado() {
    if (estadoBusqueda == EstadoBusqueda::POSIBLE_ARO) {
        return "posible aro";
    }
    if (estadoBusqueda == EstadoBusqueda::EN_ARO) {
        return "detectando el aro";
    }
    return "buscando";
}

// Cambia entre tirón grueso y fino. Vale desde el próximo tirón.
void usarTironFino(bool fino) {
    tironFino = fino;
    // El porcentaje fino está dado para PERIODO_TIRON_MS: se reescala para mantener el mismo pulso.
    tironPorcentaje = fino ? TIRON_PORCENTAJE_FINO * PERIODO_TIRON_MS / periodoTironMs : porcentajeGrueso;
}

float anguloPorTiron() {
    return tironFino ? ANGULO_TIRON_FINO : ANGULO_TIRON_GRUESO;
}

// Grados que ocupa un aro recto de ANCHO_ARO_CM visto a esa distancia.
float anchoGeometricoGrados(float distanciaCm) {
    return 2.0f * atanf(ANCHO_ARO_CM / (2.0f * distanciaCm)) * 180.0f / PI;
}

void reiniciarBusqueda() {
    estadoBusqueda = EstadoBusqueda::BUSCANDO;
    cantidadHistorial = 0;
    medidasBajas = 0;
    fallosSeguidos = 0;
    medidasAltas = 0;
    fallosAltos = 0;
    anguloActual = 0;
    tironFino = false;
    empezoDentro = false;
    subidasSeguidas = 0;
    mensajeBusqueda[0] = '\0';
}

void agregarAlHistorial(float distancia) {
    if (cantidadHistorial < 3) {
        historial[cantidadHistorial++] = distancia;
    } else {
        historial[0] = historial[1];
        historial[1] = historial[2];
        historial[2] = distancia;
    }
}

// Procesa la medida tomada cuando el robot lleva "angulo" grados girados. Cada cambio
// de estado se confirma con las medidas siguientes, así una medida suelta no engaña.
// Devuelve true cuando se confirmó que el aro ya pasó.
bool procesarMedida(float distancia, float angulo) {
    bool aroTermino = false;
    mensajeBusqueda[0] = '\0';

    if (estadoBusqueda == EstadoBusqueda::BUSCANDO) {
        // Primera medida de la búsqueda y ya hay algo a menos de 100 cm: el robot empezó apuntando a
        // algo. Se lo evalúa como posible aro; su borde izquierdo es desconocido (queda detrás).
        // Al terminar por la derecha se confirma girando a la izquierda (ver confirmarAroPorIzquierda).
        if (cantidadHistorial == 0 && distancia < DISTANCIA_MAX_ARO_CM) {
            estadoBusqueda = EstadoBusqueda::POSIBLE_ARO;
            nivelAroCm = distancia;
            medidasBajas = 1;
            fallosSeguidos = 0;
            anguloInicioAro = 0;
            empezoDentro = true;
            usarTironFino(true);
            snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                     "Empieza apuntando a algo a %.1f cm: se evalua si es el aro.", distancia);
        }

        // Hay un posible aro si la medida está a menos de 100 cm y: la anterior estaba a 100 cm o más
        // (se entra al aro desde lejos, aunque sea en escalera), o bajó CAIDA_INICIO_CM
        // respecto a la mediana de las últimas 3 medidas.
        if (cantidadHistorial >= 1 && distancia < DISTANCIA_MAX_ARO_CM) {
            const float anterior = historial[cantidadHistorial - 1];
            const bool vieneDeLejos = anterior >= DISTANCIA_MAX_ARO_CM;
            const float fondo = cantidadHistorial == 3
                ? medianaDeTres(historial[0], historial[1], historial[2])
                : anterior;
            const bool baja = cantidadHistorial == 3 && distancia < fondo - CAIDA_INICIO_CM;

            if (vieneDeLejos || baja) {
                estadoBusqueda = EstadoBusqueda::POSIBLE_ARO;
                nivelAroCm = distancia;
                medidasBajas = 1;
                fallosSeguidos = 0;
                // El borde real está entre la medida anterior y esta: se toma el punto medio
                // (todavía con el tirón con el que se llegó hasta acá, el grueso).
                anguloInicioAro = angulo - anguloPorTiron() / 2.0f;
                usarTironFino(true);
                snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                         "Posible aro: baja de %.1f a %.1f cm. Tirones finos hasta confirmar.",
                         vieneDeLejos ? anterior : fondo, distancia);
            }
        }

        // Subida de distancia tras un tramo cercano y estable: puede ser el borde derecho de un aro
        // que ya se estaba apuntando al empezar (no hubo bajada que lo delatara). Se confirma
        // girando a la izquierda (confirmarAroPorIzquierda), lo mismo que al empezar apuntando.
        if (estadoBusqueda == EstadoBusqueda::BUSCANDO) {
            const int u = cantidadHistorial - 1;
            const auto sube = [](float d, float nivel) {
                return d >= DISTANCIA_MAX_ARO_CM || d > nivel + CAIDA_CM;
            };

            if (subidasSeguidas == 0) {
                if (cantidadHistorial >= 2 && historial[u] < DISTANCIA_MAX_ARO_CM
                    && historial[u - 1] < DISTANCIA_MAX_ARO_CM
                    && fabsf(historial[u] - historial[u - 1]) <= TOLERANCIA_ARO_CM) {
                    nivelPrevioCm = (historial[u] + historial[u - 1]) / 2.0f;
                    if (sube(distancia, nivelPrevioCm)) {
                        subidasSeguidas = 1;
                        anguloSubida = angulo - anguloPorTiron() / 2.0f;
                    }
                }
            } else if (sube(distancia, nivelPrevioCm)) {
                if (++subidasSeguidas >= MEDIDAS_PARA_CONFIRMAR_SUBIDA) {
                    empezoDentro = true;
                    nivelAroCm = nivelPrevioCm;
                    anguloFinAro = anguloSubida;
                    subidasSeguidas = 0;
                    usarTironFino(true);
                    aroTermino = true;
                    snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                             "Subida de %.1f a %.1f cm: puede ser el borde de un aro que ya se apuntaba. Se confirma a la izquierda.",
                             nivelPrevioCm, distancia);
                }
            } else {
                subidasSeguidas = 0;
            }
        }
    } else if (estadoBusqueda == EstadoBusqueda::POSIBLE_ARO) {
        if (distancia < DISTANCIA_MAX_ARO_CM
            && distancia <= nivelAroCm + TOLERANCIA_ARO_CM
            && distancia >= nivelAroCm - TOLERANCIA_ARO_CERCA_CM) {
            nivelAroCm = (nivelAroCm * medidasBajas + distancia) / (medidasBajas + 1);
            medidasBajas++;
            fallosSeguidos = 0;

            if (medidasBajas >= MEDIDAS_PARA_CONFIRMAR_ARO) {
                estadoBusqueda = EstadoBusqueda::EN_ARO;
                medidasAltas = 0;
                fallosAltos = 0;
                snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                         ">>> DETECTANDO EL ARO (a %.1f cm; geometricamente ocuparia %.1f grados)",
                         nivelAroCm, anchoGeometricoGrados(nivelAroCm));
            }
        } else if (++fallosSeguidos >= FALLOS_SEGUIDOS_MAX) {
            estadoBusqueda = EstadoBusqueda::BUSCANDO;
            empezoDentro = false;
            usarTironFino(false);
            snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                     "Falsa alarma: la distancia no se mantuvo baja.");
        }
    } else {
        if (distancia > nivelAroCm + CAIDA_CM || distancia >= DISTANCIA_MAX_ARO_CM) {
            if (medidasAltas == 0) {
                anguloFinAro = angulo - anguloPorTiron() / 2.0f;  // punto medio, igual que el inicio
            }
            medidasAltas++;
            fallosAltos = 0;

            if (medidasAltas >= MEDIDAS_PARA_CONFIRMAR_FIN) {
                const float observado = anguloFinAro - anguloInicioAro;

                if (observado <= anchoMaxAro(nivelAroCm)) {
                    aroTermino = true;
                    snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                             ">>> TERMINO EL ARO (ancho observado %.1f grados a %.1f cm)",
                             observado, nivelAroCm);
                } else {
                    estadoBusqueda = EstadoBusqueda::BUSCANDO;
                    empezoDentro = false;
                    usarTironFino(false);
                    snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                             "Descartado: ancho %.1f grados, maximo %.1f.",
                             observado, anchoMaxAro(nivelAroCm));
                }
            }
        } else if (medidasAltas > 0) {
            if (++fallosAltos >= FALLOS_SEGUIDOS_MAX) {
                medidasAltas = 0;
                fallosAltos = 0;
            }
        } else if (angulo - anguloInicioAro > anchoMaxAro(nivelAroCm)) {
            estadoBusqueda = EstadoBusqueda::BUSCANDO;
            empezoDentro = false;
            usarTironFino(false);
            snprintf(mensajeBusqueda, sizeof(mensajeBusqueda),
                     "Descartado: la distancia baja supera %.1f grados.", anchoMaxAro(nivelAroCm));
        }
    }

    agregarAlHistorial(distancia);
    return aroTermino;
}

// Mediana de "cantidad" disparos válidos (con 2 es el promedio). Un disparo falso suelto,
// como un rebote de 300 cm, no la altera, a diferencia del promedio. -1 si no hay ninguno.
float medianaDeDisparos(float disparos[], int cantidad) {
    for (int i = 1; i < cantidad; i++) {
        const float valor = disparos[i];
        int j = i - 1;
        while (j >= 0 && disparos[j] > valor) {
            disparos[j + 1] = disparos[j];
            j--;
        }
        disparos[j + 1] = valor;
    }

    if (cantidad == 0) {
        return -1;
    }
    if (cantidad % 2 == 0) {
        return (disparos[cantidad / 2 - 1] + disparos[cantidad / 2]) / 2.0f;
    }
    return disparos[cantidad / 2];
}

// Mediana de DISPAROS_POR_MEDIDA disparos en reposo. Devuelve -1 si ninguno tuvo eco.
float medirEnReposo() {
    float disparos[DISPAROS_POR_MEDIDA];
    int validos = 0;

    for (int i = 0; i < DISPAROS_POR_MEDIDA; i++) {
        const float distancia = medirDistancia1();
        if (distancia >= 0) {
            disparos[validos++] = distancia;
        }
        delay(INTERVALO_MEDICION_MS);
    }

    return medianaDeDisparos(disparos, validos);
}

// Cuando la búsqueda empezó ya apuntando al aro, al terminar por la derecha se desconoce dónde
// empieza. Acá se gira a la izquierda de a un tirón: primero hasta volver a verlo (tirón S), luego hasta
// perderlo MEDIDAS_PARA_PERDER_ARO veces seguidas (tirón N). El ancho es (N - S - confirmaciones + 1)
// tirones y el centro queda a (N - S + confirmaciones) / 2 tirones a la derecha del punto actual.
// Si el ancho supera ANCHO_MAX_ARO_GRADOS no es el aro (pared): se vuelve al borde derecho.
enum class ResultadoConfirmar { CENTRADO, ES_PARED, NO_REAPARECIO, INTERRUMPIDO };
constexpr int MAX_TIRONES_CONFIRMAR = 30;
constexpr int MAX_TIRONES_REAPARECER = 8;

ResultadoConfirmar confirmarAroPorIzquierda() {
    const float gradosPorTironIzq = ANGULO_TIRON_FINO * RELACION_GIRO_IZQUIERDA_FINO;
    Salida.println("Empezo apuntando al aro: se confirma girando a la izquierda.");

    int tirones = 0;
    int primeroDentro = 0;
    int perdidas = 0;
    bool visto = false;
    bool perdido = false;

    while (tirones < MAX_TIRONES_CONFIRMAR && !perdido) {
        girarTirones(-1, 1);
        if (giroInterrumpido) {
            Salida.println("Giro interrumpido.");
            return ResultadoConfirmar::INTERRUMPIDO;
        }
        tirones++;

        delay(ASENTAR_MS);
        const float d = medirEnReposo();
        const bool dentro = d >= 0 && d < DISTANCIA_MAX_ARO_CM && d <= nivelAroCm + CAIDA_CM;

        if (!visto) {
            if (dentro) {
                visto = true;
                primeroDentro = tirones;
            }
        } else {
            perdidas = dentro ? 0 : perdidas + 1;
            perdido = perdidas >= MEDIDAS_PARA_PERDER_ARO;
        }

        Salida.print("Confirmar izquierda | tiron ");
        Salida.print(tirones);
        Salida.print(" | ");
        if (d < 0) {
            Salida.print("sin eco");
        } else {
            Salida.print(d, 1);
            Salida.print(" cm");
        }
        Salida.println(visto ? (dentro ? " | en el aro" : " | fuera") : " | todavia no lo ve");

        if (!visto && tirones >= MAX_TIRONES_REAPARECER) {
            Salida.println("No reaparecio a la izquierda: no era el aro.");
            // Vuelve al borde donde estaba.
            girarTirones(1, static_cast<int>(tirones * RELACION_GIRO_IZQUIERDA_FINO + 0.5f));
            return giroInterrumpido ? ResultadoConfirmar::INTERRUMPIDO : ResultadoConfirmar::NO_REAPARECIO;
        }
    }

    const float ancho = (tirones - primeroDentro - MEDIDAS_PARA_PERDER_ARO + 1) * gradosPorTironIzq;
    Salida.print("Ancho observado: ");
    Salida.print(ancho, 1);
    Salida.print(" grados (maximo ");
    Salida.print(anchoMaxAro(nivelAroCm), 1);
    Salida.println(")");

    if (!perdido || ancho > anchoMaxAro(nivelAroCm)) {
        Salida.println("Demasiado ancho: parece una pared, no el aro. Vuelve al borde y sigue buscando.");
        girarTirones(1, static_cast<int>(tirones * RELACION_GIRO_IZQUIERDA_FINO + 0.5f));
        return giroInterrumpido ? ResultadoConfirmar::INTERRUMPIDO : ResultadoConfirmar::ES_PARED;
    }

    const int haciaDerecha = static_cast<int>(
        (tirones - primeroDentro + MEDIDAS_PARA_PERDER_ARO) / 2.0f * RELACION_GIRO_IZQUIERDA_FINO + 0.5f);
    Salida.printf("Aro confirmado. Vuelve %d tirones a la derecha para centrarse.\n", haciaDerecha);
    girarTirones(1, haciaDerecha);
    return giroInterrumpido ? ResultadoConfirmar::INTERRUMPIDO : ResultadoConfirmar::CENTRADO;
}

// Gira a la derecha midiendo una vez por tirón hasta ver bajada, mantenimiento y subida.
// Al sospechar del aro pasa a tirones finos. Después vuelve a la izquierda B + A/2 grados,
// donde A es el ancho del aro y B lo girado desde que se dejó de verlo.
// Con 's' se interrumpe.
// Devuelve true si encontró el aro y quedó apuntando a su centro.
// Con esperar = false no hace la espera inicial (para encadenarla con otras fases).
// Se pone en true cuando el robot recién perdió el aro girando a la izquierda: se sabe que está
// afuera del aro, así que la primera medida baja ya cuenta como entrada al aro (aunque el aro
// aparezca de golpe desde la primera medida, sin una bajada visible).
bool buscarDesdeFuera = false;

bool buscarAroInterno(bool esperar) {
    const bool desdeFuera = buscarDesdeFuera;
    buscarDesdeFuera = false;
    if (!tironesActivos) {
        Salida.println("Activar los tirones con 'o' antes de buscar.");
        return false;
    }

    const unsigned long tiempoLibreMs = periodoTironMs - periodoTironMs * tironPorcentaje / 100.0f;
    if (tiempoLibreMs < ASENTAR_MS + (DISPAROS_POR_MEDIDA - 1) * INTERVALO_MEDICION_MS + 40) {
        Salida.println("El tiron es muy largo: no entran los disparos en la pausa. Bajar el %.");
        return false;
    }

    if (esperar) {
        Salida.print("Busqueda del aro: empieza en ");
        Salida.print(ESPERA_BUSQUEDA_MS / 1000);
        Salida.println(" s. Con 's' se interrumpe.");
        delay(ESPERA_BUSQUEDA_MS);
    } else {
        Salida.println("Busqueda del aro (sin espera).");
    }
    porcentajeGrueso = tironPorcentaje;
    reiniciarBusqueda();
    if (desdeFuera) {
        agregarAlHistorial(DISTANCIA_MAX_ARO_CM);  // "venía de lejos"
    }
    iniciarGiro(1);

    int tironMedido = 0;
    int disparos = 0;
    float disparosTiron[DISPAROS_POR_MEDIDA];
    int validos = 0;
    unsigned long finPulso = 0;
    unsigned long proximoDisparo = 0;
    bool aroEncontrado = false;
    bool centradoDesdeDentro = false;

    while (true) {
        actualizarTirones();
        const unsigned long ahora = millis();

        if (tironesContados != tironMedido) {  // empezó un tirón nuevo
            tironMedido = tironesContados;
            anguloActual += anguloPorTiron();
            disparos = 0;
            validos = 0;
            finPulso = 0;
        }

        if (!ruedasEncendidas && finPulso == 0) {
            finPulso = ahora;
        }

        if (finPulso != 0 && disparos < DISPAROS_POR_MEDIDA
            && ahora >= finPulso + ASENTAR_MS && ahora >= proximoDisparo) {
            const float distancia = medirDistancia1();
            disparos++;
            proximoDisparo = millis() + INTERVALO_MEDICION_MS;

            if (distancia >= 0) {
                disparosTiron[validos++] = distancia;
            }

            if (disparos == DISPAROS_POR_MEDIDA) {
                Salida.print("Tiron ");
                Salida.print(tironMedido);
                Salida.print(" | ");
                Salida.print(anguloActual, 1);
                Salida.print(" grados");

                if (validos == 0) {
                    Salida.println(" | sin eco");
                } else {
                    const float medida = medianaDeDisparos(disparosTiron, validos);
                    aroEncontrado = procesarMedida(medida, anguloActual);
                    Salida.print(" | ");
                    Salida.print(medida, 1);
                    Salida.print(" cm | ");
                    Salida.println(nombreEstado());

                    if (mensajeBusqueda[0] != '\0') {
                        Salida.println(mensajeBusqueda);
                    }
                }

                if (aroEncontrado && !empezoDentro) {
                    break;
                }

                if (aroEncontrado) {
                    // Empezó dentro del aro: se confirma y se centra girando a la izquierda.
                    sentido = 0;
                    aplicarGiro();
                    const ResultadoConfirmar confirmado = confirmarAroPorIzquierda();
                    if (confirmado == ResultadoConfirmar::CENTRADO) {
                        centradoDesdeDentro = true;
                        break;
                    }
                    if (confirmado == ResultadoConfirmar::INTERRUMPIDO) {
                        aroEncontrado = false;
                        break;
                    }

                    // Pared o falsa alarma: se sigue buscando hacia la derecha desde el borde derecho.
                    const float anguloGuardado = anguloActual;
                    aroEncontrado = false;
                    reiniciarBusqueda();
                    usarTironFino(false);
                    anguloActual = anguloGuardado;
                    agregarAlHistorial(DISTANCIA_MAX_ARO_CM);  // se está afuera del aro
                    iniciarGiro(1);
                    tironMedido = 0;
                    disparos = 0;
                    validos = 0;
                    finPulso = 0;
                    proximoDisparo = 0;
                }
            }
        }

        if (estadoBusqueda == EstadoBusqueda::BUSCANDO && anguloActual >= MAX_GIRO_BUSQUEDA_GRADOS) {
            Salida.println("No se encontro el aro en 2 vueltas.");
            break;
        }

        if (Serial.available() > 0 && Serial.read() == 's') {
            Salida.println("Busqueda interrumpida.");
            break;
        }
    }

    sentido = 0;
    aplicarGiro();

    if (!aroEncontrado) {
        usarTironFino(false);
        return false;
    }

    if (centradoDesdeDentro) {
        delay(ASENTAR_MS);
        Salida.print("Listo. Distancia al frente: ");
        Salida.print(medirEnReposo(), 1);
        Salida.print(" cm (nivel del aro: ");
        Salida.print(nivelAroCm, 1);
        Salida.println(" cm)");
        usarTironFino(false);
        return true;
    }

    const float anchoAro = anguloFinAro - anguloInicioAro;          // A
    const float despues = anguloActual - anguloFinAro;              // B
    const float aVolver = despues + anchoAro / 2.0f;
    const int tironesIzquierda =
        static_cast<int>(aVolver / (ANGULO_TIRON_FINO * RELACION_GIRO_IZQUIERDA_FINO) + 0.5f);

    Salida.print("Ancho del aro A=");
    Salida.print(anchoAro, 1);
    Salida.print(" grados | despues del aro B=");
    Salida.print(despues, 1);
    Salida.print(" | B + A/2 = ");
    Salida.print(aVolver, 1);
    Salida.print(" grados -> a la izquierda: ");
    Salida.print(tironesIzquierda);
    Salida.println(" tirones finos");

    delay(1000);
    if (tironesIzquierda > 0) {
        girarTirones(-1, tironesIzquierda);
    }

    delay(ASENTAR_MS);
    const float distanciaFinal = medirEnReposo();
    Salida.print("Listo. Distancia al frente: ");
    Salida.print(distanciaFinal, 1);
    Salida.print(" cm (nivel del aro: ");
    Salida.print(nivelAroCm, 1);
    Salida.println(" cm)");
    usarTironFino(false);
    return true;
}

// Búsqueda con el período PERIODO_BUSQUEDA_MS. El porcentaje de tirón se reescala para que
// el pulso dure lo mismo (y los ángulos por tirón sigan valiendo). Restaura todo al terminar.
bool buscarAro(bool esperar = true) {
    const float porcentajeOriginal = tironPorcentaje;
    periodoTironMs = PERIODO_BUSQUEDA_MS;
    tironPorcentaje = porcentajeOriginal * PERIODO_TIRON_MS / PERIODO_BUSQUEDA_MS;

    const bool encontrado = buscarAroInterno(esperar);
    guardarRegistroSiQuieto();

    periodoTironMs = PERIODO_TIRON_MS;
    tironPorcentaje = porcentajeOriginal;
    return encontrado;
}

// ---- Acercarse al aro y centrarse (tecla 'a') ----

constexpr float DISTANCIA_OBJETIVO_CM = 26.0f;
constexpr float TOLERANCIA_OBJETIVO_CM = 2.0f;
constexpr float SALTO_PERDER_ARO_CM = 15.0f;     // salto que indica que se dejó de ver el aro
// Avance y retroceso por pulsos de PULSO_AVANCE_MS, con pwmAvance / pwmRetroceso (sin tirones).
// Tras cada pulso se frena, se espera ASENTAR_AVANCE_MS y se mide la distancia al aro.
constexpr unsigned long PULSO_AVANCE_MS = 400;
constexpr unsigned long ASENTAR_AVANCE_MS = 400;
// Cuántos cm avanza el robot en un pulso. 0 = sin calibrar: se usan las teclas 'c' y 'r'
// para medirlo. Si es mayor que 0, un pulso cuyo recorrido medido difiera más de
// TOLERANCIA_COHERENCIA_CM de este valor significa que el sensor ya no apunta al aro.
constexpr float AVANCE_POR_PULSO_CM = 0.0f;
constexpr float TOLERANCIA_COHERENCIA_CM = 5.0f;
constexpr int MAX_PULSOS_ACERCAR = 25;
// Avance típico de un pulso completo (medido: ~9 cm). Cuando falta menos que esto, el pulso se
// acorta en proporción (mínimo PULSO_MINIMO_MS) para no pasarse del objetivo.
constexpr float AVANCE_ESTIMADO_CM = 9.5f;
constexpr unsigned long PULSO_MINIMO_MS = 120;
constexpr int MAX_TIRONES_PERDER_ARO = 50;       // unas 1,05 vueltas con tirones gruesos
constexpr int MAX_INTENTOS_ACERCAR = 6;  // cada recuperación por la izquierda gasta uno

enum class ResultadoAcercar { LLEGO, PERDIO_ARO, ABORTADO };

// Si queda a menos de esta distancia (se pasó de largo), retrocede enseguida hasta salir de ella y se
// busca el aro hacia la izquierda: al avanzar el robot se desvía a la derecha.
constexpr float DISTANCIA_MUY_CERCA_CM = 22.0f;
constexpr int MAX_PULSOS_RETROCESO_CERCA = 6;

// Si tras PULSOS_PARA_TRABADO pulsos seguidos hacia adelante la distancia no bajó al menos
// AVANCE_MINIMO_CM, el robot está trabado: el siguiente pulso es más fuerte (PWM + IMPULSO_EXTRA_PWM,
// misma duración que correspondía) para salir. Si sigue trabado, cada pulso siguiente escala:
// nivel 2 y 3 suman otro IMPULSO_EXTRA_PWM y alargan el pulso un tercio más (hasta MAX_NIVEL_IMPULSO).
constexpr int PULSOS_PARA_TRABADO = 2;
constexpr float AVANCE_MINIMO_CM = 2.0f;
constexpr int IMPULSO_EXTRA_PWM = 60;
constexpr int MAX_NIVEL_IMPULSO = 3;

// Un pulso de avance (direccion 2) o retroceso (-2) de duracionMs. Devuelve false si se
// interrumpió con 's'.
bool pulsoMovimiento(int direccion, unsigned long duracionMs = PULSO_AVANCE_MS) {
    iniciarGiro(direccion);
    const unsigned long inicio = millis();
    bool completo = true;

    while (millis() - inicio < duracionMs) {
        if (Serial.available() > 0 && Serial.read() == 's') {
            completo = false;
            break;
        }
    }

    sentido = 0;
    aplicarGiro();
    delay(ASENTAR_AVANCE_MS);
    return completo;
}

// Teclas 'c' (avanza un pulso) y 'r' (retrocede un pulso): miden antes y después y muestran
// cuánto recorrió el robot. Repetir varias veces y promediar para fijar AVANCE_POR_PULSO_CM.
void pruebaPulso(int direccion) {
    const float antes = medirEnReposo();
    Salida.print(direccion > 0 ? "Avance" : "Retroceso");
    Salida.print(" de ");
    Salida.print(PULSO_AVANCE_MS);
    Salida.print(" ms. Distancia antes: ");
    if (antes < 0) {
        Salida.println("sin eco");
    } else {
        Salida.print(antes, 1);
        Salida.println(" cm");
    }

    if (!pulsoMovimiento(direccion)) {
        Salida.println("Pulso interrumpido.");
        return;
    }

    const float despues = medirEnReposo();
    Salida.print("Distancia despues: ");
    if (despues < 0) {
        Salida.println("sin eco");
        return;
    }
    Salida.print(despues, 1);
    Salida.println(" cm");

    if (antes >= 0) {
        Salida.print("Recorrido: ");
        Salida.print(direccion > 0 ? antes - despues : despues - antes, 1);
        Salida.println(" cm");
    }
}

// Avanza (o retrocede si se pasó) con pulsos de PULSO_AVANCE_MS hasta quedar a
// DISTANCIA_OBJETIVO_CM ± TOLERANCIA_OBJETIVO_CM, midiendo tras cada pulso.
// Se perdió el aro si no hay eco, la distancia llega a 100 cm o más, o el recorrido del pulso
// no es coherente: con AVANCE_POR_PULSO_CM calibrado, difiere más de TOLERANCIA_COHERENCIA_CM;
// sin calibrar, al avanzar la distancia salta más de SALTO_PERDER_ARO_CM hacia arriba.
// Deja en distanciaLlegada la última medida y en pulsosDados cuántos pulsos hizo.
ResultadoAcercar acercarseAlAro(float& distanciaLlegada, int& pulsosDados, bool permitirRetrocesoCerca = true) {
    pulsosDados = 0;
    float quieta = medirEnReposo();
    ResultadoAcercar resultado = ResultadoAcercar::ABORTADO;
    bool terminado = false;
    int pulsosRacha = 0;       // pulsos seguidos hacia adelante sin avanzar lo suficiente
    float distanciaRacha = 0;  // distancia al empezar la racha
    bool trabado = false;
    int nivelImpulso = 0;

    for (int pulso = 1; pulso <= MAX_PULSOS_ACERCAR && !terminado; pulso++) {
        if (quieta < 0 || quieta >= DISTANCIA_MAX_ARO_CM) {
            Salida.println("Sin eco o a 100 cm o mas: se perdio el aro.");
            resultado = ResultadoAcercar::PERDIO_ARO;
            terminado = true;
            break;
        }

        // Recién centrado, la lectura al centro del aro es unos cm menor que al costado: no se vuelve a
        // retroceder y recentrar (se entraba en un ciclo); los pulsos normales ajustan la distancia.
        if (permitirRetrocesoCerca && quieta < DISTANCIA_MUY_CERCA_CM) {
            Salida.printf("Demasiado cerca (%.1f cm): retrocede y busca a la izquierda.\n", quieta);
            for (int r = 0; r < MAX_PULSOS_RETROCESO_CERCA && quieta >= 0 && quieta < DISTANCIA_OBJETIVO_CM - TOLERANCIA_OBJETIVO_CM; r++) {
                const unsigned long retrocesoMs = constrain(
                    static_cast<unsigned long>(PULSO_AVANCE_MS * (DISTANCIA_OBJETIVO_CM - quieta) / AVANCE_ESTIMADO_CM),
                    PULSO_MINIMO_MS, PULSO_AVANCE_MS);
                if (!pulsoMovimiento(-2, retrocesoMs)) {
                    Salida.println("Retroceso interrumpido.");
                    distanciaLlegada = quieta;
                    return ResultadoAcercar::ABORTADO;
                }
                pulsosDados++;
                quieta = medirEnReposo();
                Salida.print("Retrocede | ");
                Salida.print(quieta, 1);
                Salida.println(" cm");
            }
            resultado = ResultadoAcercar::PERDIO_ARO;  // el que llama busca hacia la izquierda
            terminado = true;
            break;
        }

        const float error = quieta - DISTANCIA_OBJETIVO_CM;
        if (fabsf(error) <= TOLERANCIA_OBJETIVO_CM) {
            resultado = ResultadoAcercar::LLEGO;
            terminado = true;
            break;
        }

        const int direccion = error > 0 ? 2 : -2;
        unsigned long duracionMs = constrain(
            static_cast<unsigned long>(PULSO_AVANCE_MS * fabsf(error) / AVANCE_ESTIMADO_CM),
            PULSO_MINIMO_MS, PULSO_AVANCE_MS);
        if (direccion > 0 && pulsosRacha == 0) {
            distanciaRacha = quieta;
        }
        if (trabado && direccion > 0) {
            nivelImpulso = min(nivelImpulso + 1, MAX_NIVEL_IMPULSO);
            Salida.printf("Trabado: pulso fuerte (nivel %d) para salir.\n", nivelImpulso);
            // Impulso corto (el que ya correspondía a la distancia que falta): lo que lo destraba es la
            // potencia, no la duración. Un pulso de 400 ms lo hizo pasar de 30,6 a 19,5 cm. Los niveles
            // 2 y 3 lo alargan un tercio y dos tercios más (máximo PULSO_AVANCE_MS).
            duracionMs = min(PULSO_AVANCE_MS, duracionMs * (2 + nivelImpulso) / 3);
            impulsoAvanceExtra = IMPULSO_EXTRA_PWM * nivelImpulso;
        }
        const bool pulsoCompleto = pulsoMovimiento(direccion, duracionMs);
        impulsoAvanceExtra = 0;
        if (!pulsoCompleto) {
            Salida.println("Acercamiento interrumpido.");
            terminado = true;
            break;
        }
        pulsosDados++;

        const float d = medirEnReposo();
        const float recorrido = direccion > 0 ? quieta - d : d - quieta;

        Salida.print(direccion > 0 ? "Avanza" : "Retrocede");
        Salida.print(" | pulso ");
        Salida.print(pulso);
        Salida.print(" (");
        Salida.print(duracionMs);
        Salida.print(" ms) | ");
        if (d < 0) {
            Salida.println("sin eco");
        } else {
            Salida.print(d, 1);
            Salida.print(" cm | recorrido ");
            Salida.print(recorrido, 1);
            Salida.println(" cm");
        }

        if (direccion > 0 && d >= 0) {
            pulsosRacha++;
            if (distanciaRacha - d >= AVANCE_MINIMO_CM) {
                pulsosRacha = 0;  // se movió: no está trabado
                trabado = false;
                nivelImpulso = 0;
            } else if (pulsosRacha >= PULSOS_PARA_TRABADO) {
                trabado = true;  // sigue así hasta que se mueva: el impulso escala
            }
        } else {
            pulsosRacha = 0;
            trabado = false;
            nivelImpulso = 0;
        }

        if (d < 0 || d >= DISTANCIA_MAX_ARO_CM) {
            Salida.println("Sin eco o a 100 cm o mas: se perdio el aro.");
            quieta = d;
            resultado = ResultadoAcercar::PERDIO_ARO;
            terminado = true;
            break;
        }

        {
            const bool incoherente = AVANCE_POR_PULSO_CM > 0
                ? fabsf(recorrido - AVANCE_POR_PULSO_CM) > TOLERANCIA_COHERENCIA_CM
                : (direccion > 0 && d > quieta + SALTO_PERDER_ARO_CM);

            if (incoherente) {
                Salida.println("El recorrido no es coherente con el pulso: se perdio el aro.");
                quieta = d;
                resultado = ResultadoAcercar::PERDIO_ARO;
                terminado = true;
                break;
            }
        }

        quieta = d;
    }

    if (!terminado) {
        Salida.println("Se agotaron los pulsos sin llegar.");
    }

    distanciaLlegada = quieta;
    guardarRegistroSiQuieto();
    return resultado;
}

// Gira a la izquierda con tirones gruesos hasta que la distancia sube SALTO_PERDER_ARO_CM
// sobre "referencia" (o no hay eco) durante MEDIDAS_PARA_PERDER_ARO medidas seguidas.
bool girarHastaPerderAro(float referencia, int* tironesUsados = nullptr) {
    int perdidas = 0;

    for (int i = 0; i < MAX_TIRONES_PERDER_ARO; i++) {
        girarTirones(-1, 1);
        if (giroInterrumpido) {
            Salida.println("Giro interrumpido.");
            return false;
        }

        delay(ASENTAR_MS);
        const float distancia = medirEnReposo();
        const bool perdido = distancia < 0 || distancia > referencia + SALTO_PERDER_ARO_CM;
        perdidas = perdido ? perdidas + 1 : 0;

        Salida.print("Izquierda | tiron ");
        Salida.print(i + 1);
        Salida.print(" | ");
        if (distancia < 0) {
            Salida.print("sin eco");
        } else {
            Salida.print(distancia, 1);
            Salida.print(" cm");
        }
        Salida.print(" | perdidas seguidas ");
        Salida.println(perdidas);

        if (perdidas >= MEDIDAS_PARA_PERDER_ARO) {
            Salida.println("Aro perdido hacia la izquierda.");
            if (tironesUsados != nullptr) {
                *tironesUsados = i + 1;
            }
            return true;
        }
    }

    Salida.println("No se perdio el aro girando a la izquierda.");
    return false;
}

// El robot suele terminar apuntando un poco a la derecha del centro: al final se corrige con
// esta cantidad de tirones finos a la izquierda.
constexpr int TIRONES_CORRECCION_FINAL = 3;

// Si el robot pierde el aro yendo hacia adelante, casi seguro se desvió hacia la derecha: el aro
// quedó a su izquierda. Gira a la izquierda (hasta MAX_TIRONES_RECUPERAR_IZQ tirones) midiendo tras cada
// uno. Si lo ve (a menos de 100 cm), sigue girando hasta perderlo, para que la búsqueda hacia la derecha
// lo recorra completo. Si no lo ve, el aro quedó hacia la derecha y la búsqueda normal lo encuentra.
// Si lo vio y lo perdió hacia la izquierda, vuelve a la derecha la mitad de lo girado desde que lo
// vio (más la mitad de las medidas de confirmación): queda apuntando al centro, sin paneo completo.
enum class ResultadoRecuperar { INTERRUMPIDO, NO_ENCONTRADO, CENTRADO };
constexpr int MAX_TIRONES_RECUPERAR_IZQ = 5;

ResultadoRecuperar recuperarAroPorIzquierda() {
    Salida.println("Se busca primero hacia la izquierda (el robot se desvia a la derecha al avanzar).");

    for (int i = 1; i <= MAX_TIRONES_RECUPERAR_IZQ; i++) {
        girarTirones(-1, 1);
        if (giroInterrumpido) {
            Salida.println("Giro interrumpido.");
            return ResultadoRecuperar::INTERRUMPIDO;
        }

        delay(ASENTAR_MS);
        const float distancia = medirEnReposo();
        Salida.print("Recuperar izquierda | tiron ");
        Salida.print(i);
        Salida.print(" | ");
        if (distancia < 0) {
            Salida.println("sin eco");
        } else {
            Salida.print(distancia, 1);
            Salida.println(" cm");
        }

        if (distancia >= 0 && distancia < DISTANCIA_MAX_ARO_CM) {
            Salida.println("Aro visto a la izquierda: se sigue hasta perderlo.");
            int usados = 0;
            if (!girarHastaPerderAro(distancia, &usados)) {
                return giroInterrumpido ? ResultadoRecuperar::INTERRUMPIDO : ResultadoRecuperar::NO_ENCONTRADO;
            }

            // usados = tirones desde que se vio el aro hasta confirmar que se perdió (incluye las
            // MEDIDAS_PARA_PERDER_ARO medidas de confirmación, ya pasado el borde).
            const int haciaDerecha = static_cast<int>((usados + MEDIDAS_PARA_PERDER_ARO) / 2.0f + 0.5f);
            Salida.printf("Aro visto durante %d tirones: vuelve %d a la derecha para centrarse.\n",
                          usados, haciaDerecha);
            girarTirones(1, haciaDerecha);
            if (giroInterrumpido) {
                Salida.println("Giro interrumpido.");
                return ResultadoRecuperar::INTERRUMPIDO;
            }
            delay(ASENTAR_MS);
            Salida.print("Distancia al centrarse: ");
            Salida.print(medirEnReposo(), 1);
            Salida.println(" cm");
            return ResultadoRecuperar::CENTRADO;
        }
    }

    Salida.println("No aparecio a la izquierda en 5 tirones: debe estar a la derecha.");
    return ResultadoRecuperar::NO_ENCONTRADO;
}

float distanciaFinalCm = -1;  // última distancia medida al terminar la corrección final

void correccionFinalIzquierda() {
    distanciaFinalCm = -1;
    const float porcentajeOriginal = tironPorcentaje;
    tironPorcentaje = TIRON_PORCENTAJE_FINO;
    Salida.print("Correccion final: ");
    Salida.print(TIRONES_CORRECCION_FINAL);
    Salida.println(" tiron(es) fino(s) a la izquierda.");
    girarTirones(-1, TIRONES_CORRECCION_FINAL);
    tironPorcentaje = porcentajeOriginal;

    if (!giroInterrumpido) {
        delay(ASENTAR_MS);
        distanciaFinalCm = medirEnReposo();
        Salida.print("Distancia al terminar: ");
        Salida.print(distanciaFinalCm, 1);
        Salida.println(" cm");
    }
}

// 1) busca el aro y se centra; 2) se acerca a 26 cm; 3) gira a la izquierda hasta perderlo;
// 4) vuelve a buscarlo hacia la derecha y se centra; 5) verifica que siga a 26 cm.
// Si pierde el aro al acercarse, o queda lejos de los 26 cm, repite. Con 's' se interrumpe.
bool omitirEsperaInicial = false;  // true en el arranque automático: ya esperó 7 s, no espera otra vez
bool aroListoParaLanzar = false;  // true si acercarseYCentrar terminó centrado y a distancia

void acercarseYCentrar() {
    aroListoParaLanzar = false;
    if (!tironesActivos) {
        Salida.println("Activar los tirones con 'o' antes de acercarse.");
        return;
    }

    bool centrado = false;
    bool recienCentrado = false;  // acaba de recentrarse: la lectura al centro es la de referencia

    for (int intento = 1; intento <= MAX_INTENTOS_ACERCAR; intento++) {
        Salida.print("=== Intento ");
        Salida.print(intento);
        Salida.print(" de ");
        Salida.println(MAX_INTENTOS_ACERCAR);

        if (!centrado) {
            Salida.println("Fase: buscar y centrar el aro.");
            if (!buscarAro(intento == 1 && !omitirEsperaInicial)) {
                Salida.println("No se pudo centrar el aro. Fin.");
                return;
            }
            centrado = true;
        }

        Salida.println("Fase: acercarse.");
        float distancia = 0;
        int pulsosDados = 0;
        const ResultadoAcercar resultado = acercarseAlAro(distancia, pulsosDados, !recienCentrado);
        recienCentrado = false;

        if (resultado == ResultadoAcercar::ABORTADO) {
            Salida.println("Acercamiento abortado. Fin.");
            return;
        }
        if (resultado == ResultadoAcercar::PERDIO_ARO) {
            const ResultadoRecuperar recuperado = recuperarAroPorIzquierda();
            if (recuperado == ResultadoRecuperar::INTERRUMPIDO) {
                Salida.println("Fin.");
                return;
            }
            centrado = recuperado == ResultadoRecuperar::CENTRADO;
            recienCentrado = centrado;
            continue;
        }

        // Con un pulso (o ninguno) el rumbo casi no cambia: el aro sigue centrado y no se busca de nuevo.
        if (centrado && pulsosDados <= 1) {
            Salida.print("Llego a ");
            Salida.print(distancia, 1);
            Salida.print(" cm con ");
            Salida.print(pulsosDados);
            Salida.println(" pulso(s): el rumbo no cambio, no se busca de nuevo.");
            correccionFinalIzquierda();
            Salida.println("Listo.");
            aroListoParaLanzar = true;
            return;
        }

        Salida.print("Llego a ");
        Salida.print(distancia, 1);
        Salida.println(" cm. Fase: recentrar con un solo paneo a la izquierda.");
        // Tras acercarse, el robot sigue apuntando al aro (con el desvío a la derecha). En vez de dos
        // pasadas completas (izquierda hasta perderlo y derecha hasta encontrarlo), se hace una:
        // izquierda hasta perderlo y vuelve la mitad.
        nivelAroCm = distancia;
        const ResultadoConfirmar recentrado = confirmarAroPorIzquierda();
        if (recentrado == ResultadoConfirmar::INTERRUMPIDO) {
            Salida.println("Fin.");
            return;
        }
        if (recentrado != ResultadoConfirmar::CENTRADO) {
            Salida.println("No se pudo recentrar con un paneo: se busca de nuevo.");
            centrado = false;
            continue;
        }
        recienCentrado = true;

        delay(ASENTAR_MS);
        const float distFinal = medirEnReposo();
        Salida.print("Distancia final: ");
        Salida.print(distFinal, 1);
        Salida.println(" cm");

        if (distFinal >= 0 && fabsf(distFinal - DISTANCIA_OBJETIVO_CM) <= TOLERANCIA_OBJETIVO_CM) {
            Salida.println("Listo: centrado y a 26 cm.");
            correccionFinalIzquierda();
            aroListoParaLanzar = true;
            return;
        }
        Salida.println("Fuera de 26 cm +-2: se acerca de nuevo.");
    }

    Salida.println("Se agotaron los intentos.");
}

// Prueba del sensor (tecla 'x'): dispara cada INTERVALO_MEDICION_MS y cada
// PERIODO_PRUEBA_SENSOR_MS imprime el promedio, el mínimo y el máximo de ese lapso.
constexpr unsigned long PERIODO_PRUEBA_SENSOR_MS = 500;
bool probandoSensor = false;
unsigned long inicioSegundo = 0;
unsigned long proximaLectura = 0;
float sumaLecturas = 0;
float minimoLectura = 0;
float maximoLectura = 0;
int lecturasValidas = 0;
int lecturasSinEco = 0;

void reiniciarSegundo() {
    inicioSegundo = millis();
    sumaLecturas = 0;
    minimoLectura = 1e9f;
    maximoLectura = 0;
    lecturasValidas = 0;
    lecturasSinEco = 0;
}

void actualizarPruebaSensor() {
    if (!probandoSensor) {
        return;
    }

    Salida.registrar = false;  // imprime cada 0,5 s: no se guarda
    const unsigned long ahora = millis();

    if (ahora >= proximaLectura) {
        proximaLectura = ahora + INTERVALO_MEDICION_MS;
        const float distancia = medirDistancia1();

        if (distancia < 0) {
            lecturasSinEco++;
        } else {
            lecturasValidas++;
            sumaLecturas += distancia;
            minimoLectura = min(minimoLectura, distancia);
            maximoLectura = max(maximoLectura, distancia);
        }
    }

    if (ahora - inicioSegundo >= PERIODO_PRUEBA_SENSOR_MS) {
        if (lecturasValidas > 0) {
            Salida.print("Promedio ");
            Salida.print(sumaLecturas / lecturasValidas, 1);
            Salida.print(" cm | min ");
            Salida.print(minimoLectura, 1);
            Salida.print(" | max ");
            Salida.print(maximoLectura, 1);
            Salida.print(" | validas ");
            Salida.print(lecturasValidas);
            Salida.print(" | sin eco ");
            Salida.println(lecturasSinEco);
        } else {
            Salida.print("Sin eco en ");
            Salida.print(lecturasSinEco);
            Salida.println(" disparos");
        }
        reiniciarSegundo();
    }
    Salida.registrar = true;
}

// Imprime las mediciones como CSV. El ángulo se calcula acá a partir del tiempo.
void imprimirMediciones() {
    Salida.registrar = false;
    Salida.println("# INICIO_DATOS");
    Salida.print("# periodo_vuelta_ms=");
    Salida.print(PERIODO_VUELTA_MS);
    Salida.print(" intervalo_ms=");
    Salida.print(INTERVALO_MEDICION_MS);
    Salida.print(" tirones=");
    Salida.print(tironEscaneo, 1);
    Salida.print("% de ");
    Salida.print(PERIODO_TIRON_MS);
    Salida.println(" ms");
    Salida.print("# PWM FL=");
    Salida.print(pwmEscaneo[0]);
    Salida.print(" FR=");
    Salida.print(pwmEscaneo[1]);
    Salida.print(" RL=");
    Salida.print(pwmEscaneo[2]);
    Salida.print(" RR=");
    Salida.println(pwmEscaneo[3]);
    Salida.println("medicion,sensor,distancia_cm,tiempo_ms,vuelta,angulo_grados");

    for (int i = 0; i < cantidadMediciones; i++) {
        const float anguloTotal = 360.0f * mediciones[i].tiempoMs / PERIODO_VUELTA_MS;

        Salida.print(i + 1);
        Salida.print(",1,");
        if (mediciones[i].distanciaDecimasCm == SIN_ECO) {
            Salida.print("-1");
        } else {
            Salida.print(mediciones[i].distanciaDecimasCm / 10.0f, 1);
        }
        Salida.print(",");
        Salida.print(mediciones[i].tiempoMs);
        Salida.print(",");
        Salida.print(static_cast<int>(anguloTotal / 360.0f) + 1);
        Salida.print(",");
        Salida.println(fmodf(anguloTotal, 360.0f), 1);
    }

    Salida.println("# FIN_DATOS");
    Salida.registrar = true;
}

// Espera, gira NUM_VUELTAS midiendo cada INTERVALO_MEDICION_MS, frena e imprime.
// Con 's' se interrumpe. No abrir ni cerrar el monitor mientras tanto: reinicia la ESP32.
void escanear() {
    Salida.print("Escaneo: gira en ");
    Salida.print(ESCANEO_ESPERA_MS / 1000);
    Salida.println(" s.");
    delay(ESCANEO_ESPERA_MS);
    cantidadMediciones = 0;
    tironEscaneo = tironesActivos ? tironPorcentaje : 0;
    for (int i = 0; i < 4; i++) {
        pwmEscaneo[i] = pwm[i];
    }

    const unsigned long inicio = millis();
    const unsigned long duracion = NUM_VUELTAS * PERIODO_VUELTA_MS;
    unsigned long proximaMedicion = 0;

    iniciarGiro(1);
    while (millis() - inicio < duracion && cantidadMediciones < MAX_MEDICIONES) {
        actualizarTirones();

        const unsigned long ahora = millis();
        if (ahora >= proximaMedicion) {
            proximaMedicion = ahora + INTERVALO_MEDICION_MS;
            mediciones[cantidadMediciones].tiempoMs = ahora - inicio;
            const float distancia = medirDistancia1();
            mediciones[cantidadMediciones].distanciaDecimasCm =
                distancia < 0 ? SIN_ECO : static_cast<uint16_t>(distancia * 10.0f + 0.5f);
            cantidadMediciones++;
        }

        if (Serial.available() > 0 && Serial.read() == 's') {
            break;
        }
    }

    sentido = 0;
    aplicarGiro();
    guardarEscaneo();
    Salida.println("Escaneo terminado y guardado en flash.");
    imprimirMediciones();
}

// ---- TEMPORAL: pruebas a toda velocidad (teclas 'W' y 'G'). Borrar al terminar. ----
// Espera 5 s y luego mueve las cuatro ruedas a PWM 255 durante 3 s. Con 's' se interrumpe.
void pruebaVelocidadMaxima(bool girar) {
    Salida.println(girar ? "Prueba: gira sobre si mismo a toda velocidad. Empieza en 5 s."
                         : "Prueba: avanza a toda velocidad. Empieza en 5 s.");
    if (!esperarLanzamiento(5000)) {
        Salida.println("Prueba cancelada.");
        return;
    }

    ruedas.establecerVelocidades(255, 255, 255, 255);
    if (girar) {
        ruedas.girarDerecha();
    } else {
        ruedas.avanzar();
    }
    const bool completo = esperarLanzamiento(3000);
    ruedas.detener();
    Salida.println(completo ? "Prueba terminada." : "Prueba interrumpida.");
}

void imprimirPwm() {
    Salida.print("PWM  FL=");
    Salida.print(pwm[0]);
    Salida.print("  FR=");
    Salida.print(pwm[1]);
    Salida.print("  RL=");
    Salida.print(pwm[2]);
    Salida.print("  RR=");
    Salida.println(pwm[3]);
}

void imprimirPwmAvance() {
    Salida.print(sentidoAjuste == 2 ? "PWM avance *  " : "PWM avance    ");
    Salida.print("FL=");
    Salida.print(pwmAvance[0]);
    Salida.print("  FR=");
    Salida.print(pwmAvance[1]);
    Salida.print("  RL=");
    Salida.print(pwmAvance[2]);
    Salida.print("  RR=");
    Salida.println(pwmAvance[3]);

    Salida.print(sentidoAjuste == -2 ? "PWM retroceso * " : "PWM retroceso   ");
    Salida.print("FL=");
    Salida.print(pwmRetroceso[0]);
    Salida.print("  FR=");
    Salida.print(pwmRetroceso[1]);
    Salida.print("  RL=");
    Salida.print(pwmRetroceso[2]);
    Salida.print("  RR=");
    Salida.println(pwmRetroceso[3]);
}

// Proceso entero (tecla 'a' y arranque automático): busca el aro, se acerca, se centra, espera
// LANZAR_ESPERA_PREVIA_MS y lanza.
void rutinaCompleta() {
    acercarseYCentrar();
    guardarRegistroSiQuieto();
    if (aroListoParaLanzar) {
        Salida.printf("Aro listo. Espera %lu s antes del tiro (con 's' se cancela).\n",
                      LANZAR_ESPERA_PREVIA_MS / 1000);
        if (esperarLanzamiento(LANZAR_ESPERA_PREVIA_MS)) {
            lanzar(distanciaFinalCm, DISTANCIA_OBJETIVO_CM);
        } else {
            Salida.println("Lanzamiento cancelado.");
        }
    }
}

// Rueda elegida: 0-3 es una rueda, 4 son las cuatro juntas.
int seleccion = 4;

void procesarComando(char comando) {
    if (comando >= '1' && comando <= '6') {
        seleccion = comando - '1';
        Salida.print("Seleccionada: ");
        if (seleccion == 4) {
            Salida.println("todas las ruedas");
        } else if (seleccion == 5) {
            Salida.println("porcentaje de tirones");
        } else {
            Salida.println(seleccion + 1);
        }
    } else if ((comando == '+' || comando == '-') && seleccion == 5) {
        tironPorcentaje += comando == '+' ? PASO_TIRON : -PASO_TIRON;
        tironPorcentaje = constrain(tironPorcentaje, PASO_TIRON, 100);
        Salida.print("Tiron: ");
        Salida.print(tironPorcentaje, 1);
        Salida.print(" % de cada ");
        Salida.print(PERIODO_TIRON_MS);
        Salida.println(" ms");
    } else if (comando == '+' || comando == '-') {
        const int paso = comando == '+' ? PASO_PWM : -PASO_PWM;
        for (int i = 0; i < 4; i++) {
            if (seleccion == 4 || seleccion == i) {
                pwm[i] = constrain(pwm[i] + paso, 0, 255);
            }
        }
        aplicarGiro();
        imprimirPwm();
    } else if (comando == 'd') {
        iniciarGiro(1);
        Salida.println("Girando a la derecha.");
    } else if (comando == 'i') {
        iniciarGiro(-1);
        Salida.println("Girando a la izquierda.");
    } else if (comando == 'w' || comando == 'z') {
        sentido = comando == 'w' ? 2 : -2;
        sentidoAjuste = sentido;
        aplicarGiro();
        Salida.println(comando == 'w' ? "Avanzando." : "Retrocediendo.");
        imprimirPwmAvance();
    } else if (comando == 'm' || comando == 'n') {
        if (seleccion == 5) {
            Salida.println("Elegir una rueda (1-4) o todas (5) antes de ajustar el PWM de avance.");
        } else {
            const int paso = comando == 'm' ? PASO_PWM : -PASO_PWM;
            int* destino = sentidoAjuste == 2 ? pwmAvance : pwmRetroceso;
            for (int i = 0; i < 4; i++) {
                if (seleccion == 4 || seleccion == i) {
                    destino[i] = constrain(destino[i] + paso, 0, 255);
                }
            }
            if (sentido == 2 || sentido == -2) {
                aplicarGiro();
            }
            imprimirPwmAvance();
        }
    } else if (comando == 'c' || comando == 'r') {
        pruebaPulso(comando == 'c' ? 2 : -2);
    } else if (comando == 'y') {
        imprimirPwmAvance();
    } else if (comando == 'o') {
        tironesActivos = !tironesActivos;
        aplicarGiro();
        Salida.println(tironesActivos ? "Tirones ACTIVADOS." : "Tirones desactivados.");
    } else if (comando == 'u' || comando == 'j') {
        tironPorcentaje += comando == 'u' ? PASO_TIRON : -PASO_TIRON;
        tironPorcentaje = constrain(tironPorcentaje, PASO_TIRON, 100);
        Salida.print("Tiron: ");
        Salida.print(tironPorcentaje, 1);
        Salida.print(" % de cada ");
        Salida.print(PERIODO_TIRON_MS);
        Salida.println(" ms");
    } else if (comando == 'e') {
        escanear();
        guardarRegistroSiQuieto();
    } else if (comando == 'x') {
        probandoSensor = !probandoSensor;
        if (probandoSensor) {
            reiniciarSegundo();
            proximaLectura = 0;
        }
        Salida.println(probandoSensor ? "Prueba del sensor ACTIVADA." : "Prueba del sensor detenida.");
    } else if (comando == 'v') {
        imprimirRegistro();
    } else if (comando == 'V') {
        imprimirMediciones();
    } else if (comando == 'l') {
        borrarRegistro();
    } else if (comando == 'b') {
        buscarAro();
        guardarRegistroSiQuieto();
    } else if (comando == 'a') {
        rutinaCompleta();
    } else if (comando == 'q') {
        pruebaSimetria();
    } else if (comando == 's') {
        if (sentido != 0) {
            Salida.print("Giro: ");
            Salida.print(tironesContados);
            Salida.print(" tirones en ");
            Salida.print(millis() - inicioGiroMs);
            Salida.println(" ms");
        }
        sentido = 0;
        aplicarGiro();
        Salida.println("Motores detenidos.");
    } else if (comando == 'p') {
        imprimirPwm();
    } else if (comando == 'g') {
        servoGira(+1);
    } else if (comando == 'h') {
        servoGira(-1);
    } else if (comando == 'f') {
        servoFrena();
    } else if (comando == '[' || comando == ']') {
        pulsoFrenoUs += comando == ']' ? 5 : -5;
        servoFrena();
    } else if (comando == 'W' || comando == 'G') {  // TEMPORAL
        pruebaVelocidadMaxima(comando == 'G');
    } else if (comando == 'L') {
        lanzar();
    } else if (comando == ',' || comando == '.') {
        motorPotenciaPct = constrain(motorPotenciaPct + (comando == '.' ? MOTOR_PASO_PCT : -MOTOR_PASO_PCT), 0, 100);
        Salida.printf("Potencia del motor: %d %%\n", motorPotenciaPct);
        motorAplicar();
    } else if (comando == '<' || comando == '>') {
        lanzarServoMs = constrain((long)lanzarServoMs + (comando == '>' ? 1 : -1) * (long)LANZAR_SERVO_PASO_MS, 25L, 3000L);
        Salida.printf("Servo al lanzar: %lu ms\n", lanzarServoMs);
    } else if (comando == 'k') {
        motorOn();
    } else if (comando == 't') {
        motorOff();
    } else if (comando != '\n' && comando != '\r') {
        Salida.println("Comando no reconocido.");
    }
}

void setup() {
    Serial.begin(115200);
    registroListo = LittleFS.begin(true);
    ruedas.inicializar();

    ledcSetup(CANAL_MOTOR, MOTOR_FRECUENCIA_HZ, MOTOR_RESOLUCION_BITS);
    ledcAttachPin(PIN_MOSFET, CANAL_MOTOR);
    ledcWrite(CANAL_MOTOR, MOTOR_DUTY_MAX);  // arranca en alto = motor frenado

    ledcSetup(CANAL_SERVO, SERVO_FRECUENCIA_HZ, SERVO_RESOLUCION_BITS);
    ledcAttachPin(PIN_SERVO, CANAL_SERVO);
    servoPulso(pulsoFrenoUs);

    inicializarSensores();
    ruedas.detener();
    cargarEscaneo();
    Salida.println("=== Arranque ===");
    Salida.println("Calibracion de giro. Ruedas: 1=FL 2=FR 3=RL 4=RR 5=todas 6=tirones");
    Salida.println("Teclas: d/i girar, s parar, b buscar aro, a acercarse y centrar, w/z avanzar/retroceder, m/n PWM avance, y imprimir avance, c/r pulso de prueba, q simetria, e escaneo, x sensor, +/- PWM, p imprimir, v registro, l borrar registro, V escaneo, g/h servo, f frena servo, [ ] ajusta freno servo, k/t motor mosfet, ,/. potencia motor, </> tiempo servo, L lanzar");
    imprimirPwm();
    Salida.print("Escaneo guardado en flash: ");
    Salida.print(cantidadMediciones);
    Salida.println(" mediciones. Tecla V para imprimirlas.");

    // Arranque automático: espera ARRANQUE_AUTOMATICO_MS y empieza el proceso entero (como la tecla 'a').
    // Cualquier tecla enviada por el monitor serial durante la espera lo cancela.
    Salida.printf("Arranque automatico en %lu s. Cualquier tecla lo cancela.\n", ARRANQUE_AUTOMATICO_MS / 1000);
    const unsigned long inicioEspera = millis();
    bool cancelado = false;
    while (millis() - inicioEspera < ARRANQUE_AUTOMATICO_MS) {
        if (Serial.available() > 0) {
            while (Serial.available() > 0) {
                Serial.read();
            }
            cancelado = true;
            break;
        }
    }

    if (cancelado) {
        Salida.println("Arranque automatico cancelado.");
    } else {
        omitirEsperaInicial = true;
        rutinaCompleta();
        omitirEsperaInicial = false;
    }
}

// ---- Repetir el tiro con la mano (sin computadora) ----
// Con el robot inactivo (en el loop, fuera de 'a', 'b', 'e', etc.), pasar la mano muy cerca del
// sensor 3 veces repite el lanzamiento, sin buscar el aro ni moverse. Cada pasada es: distancia menor
// a GESTO_CERCA_CM y luego una lectura de GESTO_LEJOS_CM o más. Entre pasadas pueden pasar como mucho
// GESTO_VENTANA_MS, o el conteo vuelve a cero. Usa la misma potencia que el último tiro automático.
constexpr float GESTO_CERCA_CM = 6.0f;
constexpr float GESTO_LEJOS_CM = 10.0f;
constexpr int GESTO_PASADAS = 3;
constexpr unsigned long GESTO_VENTANA_MS = 3000;
constexpr unsigned long GESTO_ESPERA_MS = 3000;  // para sacar la mano antes de que el motor arranque

int gestoPasadas = 0;
bool gestoManoCerca = false;
unsigned long gestoUltimaPasada = 0;
unsigned long gestoProximaLectura = 0;

void actualizarGestoLanzar() {
    // Solo con el robot inactivo: sin movimiento y sin la prueba de sensor corriendo.
    if (sentido != 0 || probandoSensor) {
        gestoPasadas = 0;
        gestoManoCerca = false;
        return;
    }

    const unsigned long ahora = millis();
    if (ahora < gestoProximaLectura) {
        return;
    }
    gestoProximaLectura = ahora + INTERVALO_MEDICION_MS;

    if (gestoPasadas > 0 && !gestoManoCerca && ahora - gestoUltimaPasada > GESTO_VENTANA_MS) {
        gestoPasadas = 0;  // tardó demasiado: empieza de nuevo
    }

    const float d = medirDistancia1();
    if (d >= 0 && d < GESTO_CERCA_CM) {
        if (!gestoManoCerca) {
            gestoManoCerca = true;
            gestoPasadas++;
            gestoUltimaPasada = ahora;
            Salida.printf("Gesto: pasada %d de %d.\n", gestoPasadas, GESTO_PASADAS);
        }
    } else if (d >= GESTO_LEJOS_CM) {
        gestoManoCerca = false;  // la mano se alejó: la próxima cuenta como otra pasada
    }

    if (gestoPasadas >= GESTO_PASADAS) {
        gestoPasadas = 0;
        gestoManoCerca = false;
        Salida.printf("Gesto reconocido: repite el tiro en %lu s (con 's' se cancela).\n", GESTO_ESPERA_MS / 1000);
        if (esperarLanzamiento(GESTO_ESPERA_MS)) {
            lanzar(distanciaFinalCm, DISTANCIA_OBJETIVO_CM);
        } else {
            Salida.println("Tiro cancelado.");
        }
        gestoProximaLectura = millis() + 2000;  // no re-detectar la mano que se retira
        guardarRegistroSiQuieto();
    }
}

void loop() {
    while (Serial.available() > 0) {
        procesarComando(Serial.read());
    }

    actualizarTirones();
    actualizarPruebaSensor();
    actualizarGestoLanzar();

    // Con el robot quieto, vuelca el registro a flash cada tanto.
    if (sentido == 0 && Salida.pendiente() > 0 && millis() - ultimoGuardadoRegistro > 3000) {
        guardarRegistro();
    }
}
