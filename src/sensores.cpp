#include <Arduino.h>
#include <math.h>
#include "sensores.h"

namespace {

constexpr int TRIG_PIN = 23;
constexpr int ECHO_PIN = 22;
constexpr unsigned long INTERVALO_PROMEDIO_MS = 500;
constexpr int CANTIDAD_ERRORES_HISTORICOS = 8;
constexpr int ERRORES_PARA_CALIBRAR = 5;
constexpr float FACTOR_SIGMA_RUPTURA = 4.0f;

float mediana(const float valores[], int cantidad) {
    float ordenados[CANTIDAD_ERRORES_HISTORICOS];

    for (int i = 0; i < cantidad; i++) {
        ordenados[i] = valores[i];
    }

    for (int i = 1; i < cantidad; i++) {
        float valor = ordenados[i];
        int j = i - 1;

        while (j >= 0 && ordenados[j] > valor) {
            ordenados[j + 1] = ordenados[j];
            j--;
        }

        ordenados[j + 1] = valor;
    }

    if (cantidad % 2 == 0) {
        return (ordenados[cantidad / 2 - 1] + ordenados[cantidad / 2]) / 2.0f;
    }

    return ordenados[cantidad / 2];
}

bool detectarRupturaParabolica(float distanciaActual, float errorEstandarActual) {
    static float distancias[3];
    static float erroresEstandar[3];
    static int cantidadDistancias = 0;
    static float erroresHistoricos[CANTIDAD_ERRORES_HISTORICOS];
    static int cantidadErrores = 0;
    static int rupturasConsecutivas = 0;
    static bool aroDetectado = false;
    static bool barridoActivo = false;

    if (cantidadDistancias < 3) {
        distancias[cantidadDistancias] = distanciaActual;
        erroresEstandar[cantidadDistancias] = errorEstandarActual;
        cantidadDistancias++;
        return false;
    }

    float distanciaPredicha = 3.0f * distancias[2]
                            - 3.0f * distancias[1]
                            + distancias[0];
    float incertidumbre = sqrtf(
        errorEstandarActual * errorEstandarActual
        + 9.0f * erroresEstandar[2] * erroresEstandar[2]
        + 9.0f * erroresEstandar[1] * erroresEstandar[1]
        + erroresEstandar[0] * erroresEstandar[0]);
    float errorNormalizado = fabsf(distanciaActual - distanciaPredicha)
                           / (incertidumbre > 0.0f ? incertidumbre : 0.0001f);

    float cambio1 = distancias[1] - distancias[0];
    float cambio2 = distancias[2] - distancias[1];
    float cambio3 = distanciaActual - distancias[2];
    float incertidumbreCambio1 = sqrtf(
        erroresEstandar[1] * erroresEstandar[1]
        + erroresEstandar[0] * erroresEstandar[0]);
    float incertidumbreCambio2 = sqrtf(
        erroresEstandar[2] * erroresEstandar[2]
        + erroresEstandar[1] * erroresEstandar[1]);
    float incertidumbreCambio3 = sqrtf(
        errorEstandarActual * errorEstandarActual
        + erroresEstandar[2] * erroresEstandar[2]);
    bool tendenciaAscendente = cambio1 > 2.0f * incertidumbreCambio1
                            && cambio2 > 2.0f * incertidumbreCambio2
                            && cambio3 > 2.0f * incertidumbreCambio3;
    bool tendenciaDescendente = cambio1 < -2.0f * incertidumbreCambio1
                             && cambio2 < -2.0f * incertidumbreCambio2
                             && cambio3 < -2.0f * incertidumbreCambio3;
    if (tendenciaAscendente || tendenciaDescendente) {
        barridoActivo = true;
    }

    bool ruptura = false;

    if (cantidadErrores >= ERRORES_PARA_CALIBRAR) {
        float errorMediano = mediana(erroresHistoricos, cantidadErrores);
        float desviaciones[CANTIDAD_ERRORES_HISTORICOS];

        for (int i = 0; i < cantidadErrores; i++) {
            desviaciones[i] = fabsf(erroresHistoricos[i] - errorMediano);
        }

        float mad = mediana(desviaciones, cantidadErrores);
        float escalaRobusta = 1.4826f * mad;
        if (escalaRobusta < 1.0f) {
            escalaRobusta = 1.0f;
        }

        float limite = errorMediano + FACTOR_SIGMA_RUPTURA * escalaRobusta;
        ruptura = barridoActivo && errorNormalizado > limite;
    }

    if (ruptura) {
        rupturasConsecutivas++;
    } else {
        rupturasConsecutivas = 0;
    }

    bool nuevaRuptura = !aroDetectado && rupturasConsecutivas >= 2;
    if (nuevaRuptura) {
        aroDetectado = true;
    }

    if (!ruptura) {
        if (cantidadErrores < CANTIDAD_ERRORES_HISTORICOS) {
            erroresHistoricos[cantidadErrores++] = errorNormalizado;
        } else {
            for (int i = 1; i < CANTIDAD_ERRORES_HISTORICOS; i++) {
                erroresHistoricos[i - 1] = erroresHistoricos[i];
            }
            erroresHistoricos[CANTIDAD_ERRORES_HISTORICOS - 1] = errorNormalizado;
        }
    }

    distancias[0] = distancias[1];
    distancias[1] = distancias[2];
    distancias[2] = distanciaActual;
    erroresEstandar[0] = erroresEstandar[1];
    erroresEstandar[1] = erroresEstandar[2];
    erroresEstandar[2] = errorEstandarActual;

    return nuevaRuptura;
}

} // namespace

void inicializarSensores() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

bool actualizarSensores() {
    unsigned long inicio = millis();
    float promedioLectura = 0.0f;
    float sumaCuadradosDiferencias = 0.0f;
    int cantidad = 0;

    while (millis() - inicio < INTERVALO_PROMEDIO_MS) {
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);

        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);

        unsigned long duracion = pulseIn(ECHO_PIN, HIGH, 30000);

        if (duracion > 0) {
            float distancia = duracion * 0.0343f / 2.0f;
            cantidad++;
            float diferencia = distancia - promedioLectura;
            promedioLectura += diferencia / cantidad;
            float diferenciaPosterior = distancia - promedioLectura;
            sumaCuadradosDiferencias += diferencia * diferenciaPosterior;
        }

        delay(20);
    }

    if (cantidad > 0) {
        float promedio = promedioLectura;
        float desviacion = cantidad > 1
                         ? sqrtf(sumaCuadradosDiferencias / (cantidad - 1))
                         : 0.0f;
        float errorMuestreo = desviacion / sqrtf(static_cast<float>(cantidad));
        float errorCuantizacion = (0.0343f / 2.0f)
                    / sqrtf(12.0f * cantidad);
        float errorEstandar = sqrtf(errorMuestreo * errorMuestreo
                      + errorCuantizacion * errorCuantizacion);

        Serial.print("Distancia: ");
        Serial.print(promedio, 2);
        Serial.print(" cm (");
        Serial.print(cantidad);
        Serial.println(" mediciones)");

        if (detectarRupturaParabolica(promedio, errorEstandar)) {
            Serial.println("POSIBLE ARO DETECTADO (ruptura de la curva)");
            return true;
        }
    } else {
        Serial.println("Sin mediciones validas");
    }

    return false;
}

// Un solo disparo del sensor. Devuelve la distancia en cm, o -1 si no hubo eco.
float medirDistancia1() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duracion = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duracion == 0) {
        return -1;
    }

    return duracion * 0.0343f / 2.0f;
}

float medirDistancia2() {
    return 0;
}