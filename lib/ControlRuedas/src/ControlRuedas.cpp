#include <Arduino.h>
#include "ControlRuedas.h"

namespace {

struct MotorL298N {
    uint8_t enable;
    uint8_t entrada1;
    uint8_t entrada2;
    bool invertirSentido;
};

// Orden por motor: ENA, IN1, IN2. GPIO34 y GPIO35 no son salidas en ESP32.
constexpr MotorL298N MOTOR_DELANTERO_IZQUIERDO = {16, 17, 32, false};
constexpr MotorL298N MOTOR_TRASERO_IZQUIERDO = {5, 18, 19, false};
constexpr MotorL298N MOTOR_DELANTERO_DERECHO = {26, 25, 33, true};
constexpr MotorL298N MOTOR_TRASERO_DERECHO = {15, 2, 4, true};

void configurarMotor(const MotorL298N& motor) {
    pinMode(motor.enable, OUTPUT);
    pinMode(motor.entrada1, OUTPUT);
    pinMode(motor.entrada2, OUTPUT);
    analogWrite(motor.enable, 0);
    digitalWrite(motor.entrada1, LOW);
    digitalWrite(motor.entrada2, LOW);
}

void escribirMotor(const MotorL298N& motor, int comando) {
    if (motor.invertirSentido) {
        comando = -comando;
    }

    if (comando > 0) {
        digitalWrite(motor.entrada1, HIGH);
        digitalWrite(motor.entrada2, LOW);
    } else if (comando < 0) {
        digitalWrite(motor.entrada1, LOW);
        digitalWrite(motor.entrada2, HIGH);
    } else {
        analogWrite(motor.enable, 0);
        digitalWrite(motor.entrada1, LOW);
        digitalWrite(motor.entrada2, LOW);
        return;
    }

    analogWrite(motor.enable, abs(comando));
}

} // namespace

ControlRuedas::ControlRuedas()
        : velocidadDelanteraIzquierda_(180),
            velocidadDelanteraDerecha_(180),
            velocidadTraseraIzquierda_(180),
            velocidadTraseraDerecha_(180) {}

void ControlRuedas::inicializar() {
    configurarMotor(MOTOR_DELANTERO_IZQUIERDO);
    configurarMotor(MOTOR_TRASERO_IZQUIERDO);
    configurarMotor(MOTOR_DELANTERO_DERECHO);
    configurarMotor(MOTOR_TRASERO_DERECHO);
}

void ControlRuedas::establecerVelocidad(int velocidad) {
    establecerVelocidades(velocidad, velocidad, velocidad, velocidad);
}

int ControlRuedas::obtenerVelocidad() const {
    return velocidadDelanteraIzquierda_;
}

void ControlRuedas::establecerVelocidades(int delanteraIzquierda,
                                         int delanteraDerecha,
                                         int traseraIzquierda,
                                         int traseraDerecha) {
    velocidadDelanteraIzquierda_ = constrain(delanteraIzquierda, 0, 255);
    velocidadDelanteraDerecha_ = constrain(delanteraDerecha, 0, 255);
    velocidadTraseraIzquierda_ = constrain(traseraIzquierda, 0, 255);
    velocidadTraseraDerecha_ = constrain(traseraDerecha, 0, 255);
}

void ControlRuedas::aplicarMovimiento(int delanteraIzquierda,
                                      int delanteraDerecha,
                                      int traseraIzquierda,
                                      int traseraDerecha) {
    escribirMotor(MOTOR_DELANTERO_IZQUIERDO,
                  delanteraIzquierda * velocidadDelanteraIzquierda_);
    escribirMotor(MOTOR_DELANTERO_DERECHO,
                  delanteraDerecha * velocidadDelanteraDerecha_);
    escribirMotor(MOTOR_TRASERO_IZQUIERDO,
                  traseraIzquierda * velocidadTraseraIzquierda_);
    escribirMotor(MOTOR_TRASERO_DERECHO,
                  traseraDerecha * velocidadTraseraDerecha_);
}

void ControlRuedas::avanzar() {
    aplicarMovimiento(1, 1, 1, 1);
}

void ControlRuedas::retroceder() {
    aplicarMovimiento(-1, -1, -1, -1);
}

void ControlRuedas::desplazarIzquierda() {
    aplicarMovimiento(-1, 1, 1, -1);
}

void ControlRuedas::desplazarDerecha() {
    aplicarMovimiento(1, -1, -1, 1);
}

void ControlRuedas::avanzarDiagonalIzquierda() {
    aplicarMovimiento(0, 1, 1, 0);
}

void ControlRuedas::avanzarDiagonalDerecha() {
    aplicarMovimiento(1, 0, 0, 1);
}

void ControlRuedas::retrocederDiagonalIzquierda() {
    aplicarMovimiento(0, -1, -1, 0);
}

void ControlRuedas::retrocederDiagonalDerecha() {
    aplicarMovimiento(-1, 0, 0, -1);
}

void ControlRuedas::girarIzquierda() {
    aplicarMovimiento(-1, 1, -1, 1);
}

void ControlRuedas::girarDerecha() {
    aplicarMovimiento(1, -1, 1, -1);
}

void ControlRuedas::detener() {
    aplicarMovimiento(0, 0, 0, 0);
}
