# Proyecto 1 - Levinas - Comas

Robot con una ESP32 que **busca un aro, se acerca, se centra y le lanza una pelota**.
Todo el proceso es automático: el robot detecta el aro con un sensor ultrasónico, se ubica a unos 26 cm,
se corrige hacia la izquierda (el tiro sale desviado a la derecha) y dispara con una rueda lanzadora.

## Integrantes

| Nombre | Número de estudiante |
|---|---|
| Uriel Levinas | 306535 |
| Matias Comas | 346072 |

## Hardware

- Placa **ESP32** (`esp32dev`).
- 4 ruedas con motores controlados por drivers **L298N** (pines en `lib/ControlRuedas/src/ControlRuedas.cpp`):

  | Rueda | ENA | IN1 | IN2 |
  |---|---|---|---|
  | Delantera izquierda | 16 | 17 | 32 |
  | Trasera izquierda | 5 | 18 | 19 |
  | Delantera derecha | 26 | 25 | 33 |
  | Trasera derecha | 15 | 2 | 4 |

- Sensor ultrasónico **HC-SR04**: TRIG en GPIO 23, ECHO en GPIO 22.
- Motor del lanzador, por un **MOSFET** en GPIO 14 (lógica invertida: señal en alto = frenado).
- **Servo** de rotación continua en GPIO 13, que sostiene la pelota y la suelta.

## Cómo funciona

1. **Búsqueda del aro:** gira sobre su eje con tirones cortos y mide una distancia por tirón. Reconoce el aro por
   su patrón de distancia (bajada, tramo estable y subida). Descarta lo que es demasiado ancho (paredes).
2. **Acercamiento:** avanza en pulsos hasta quedar a 26 cm. Si se traba, da un pulso más fuerte; si se pasa de
   largo, retrocede. Como al avanzar se desvía a la derecha, si pierde el aro lo busca primero a la izquierda.
3. **Centrado:** barre el aro para encontrar su centro y corrige unos tirones a la izquierda.
4. **Lanzamiento:** acelera la rueda lanzadora, el servo suelta la pelota y luego frena el motor. La potencia se
   ajusta un poco según la distancia final.

Al encenderse, espera 7 segundos y arranca solo el proceso completo. Cualquier tecla enviada por el monitor
serial en esos 7 segundos lo cancela. Para **repetir el tiro sin la computadora**, con el robot quieto se pasa la
mano 3 veces a menos de 6 cm del sensor.

## Comandos por el monitor serial (115200 baudios)

| Tecla | Acción |
|---|---|
| `a` | Proceso completo: busca, se acerca, se centra y lanza |
| `b` | Solo busca el aro y se centra |
| `L` | Lanza la pelota (sin buscar) |
| `s` | Detiene todo / interrumpe la rutina en curso |
| `d` / `i` | Gira a la derecha / izquierda |
| `w` / `z` | Avanza / retrocede |
| `g` / `h` / `f` | Servo gira en un sentido / en el otro / frena |
| `k` / `t` | Motor del lanzador prendido / apagado |
| `,` / `.` | Baja / sube la potencia del motor |
| `x` | Prueba del sensor ultrasónico |
| `v` | Imprime el registro guardado en la memoria flash |

Hay más teclas de calibración (PWM de cada rueda, tirones, escaneo); están listadas al inicio de `src/main.cpp`.

## Estructura

- `src/main.cpp`: programa principal (búsqueda, acercamiento, lanzamiento y comandos).
- `src/sensores.cpp`, `src/sensores.h`: lectura del sensor ultrasónico.
- `lib/ControlRuedas/`: control de las cuatro ruedas.
- `platformio.ini`: configuración del proyecto.

## Compilar y subir

Con [PlatformIO](https://platformio.org/) (extensión de VS Code o línea de comandos):

```
pio run              # compilar
pio run -t upload    # subir a la ESP32
pio device monitor   # monitor serial
```
