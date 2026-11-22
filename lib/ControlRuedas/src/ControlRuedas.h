#ifndef CONTROL_RUEDAS_H
#define CONTROL_RUEDAS_H

class ControlRuedas {
public:
    ControlRuedas();

    void inicializar();
    void establecerVelocidad(int velocidad);
    int obtenerVelocidad() const;
    void establecerVelocidades(int delanteraIzquierda,
                              int delanteraDerecha,
                              int traseraIzquierda,
                              int traseraDerecha);

    void avanzar();
    void retroceder();
    void desplazarIzquierda();
    void desplazarDerecha();
    void avanzarDiagonalIzquierda();
    void avanzarDiagonalDerecha();
    void retrocederDiagonalIzquierda();
    void retrocederDiagonalDerecha();
    void girarIzquierda();
    void girarDerecha();
    void detener();

private:
    int velocidadDelanteraIzquierda_;
    int velocidadDelanteraDerecha_;
    int velocidadTraseraIzquierda_;
    int velocidadTraseraDerecha_;

    void aplicarMovimiento(int delanteraIzquierda,
                           int delanteraDerecha,
                           int traseraIzquierda,
                           int traseraDerecha);
};

#endif
