# Piano
## Piano controlado por UART

Este programa implementa:

- Una forma de reportar el funcionamiento del PIC.
    - *Se utiliza **U1TXInterrupt*** 

- La interrupción RX activa las notas programadas en el PIC

    - *Se utiliza **UART1** junto **U1RXInterrupt** con salida del sonido en **PORTB** en el **PIC***


### Materiales
- dsPIC30F4013
- Modulo Bluetooth HC-06
- Bluetooth terminal (App de Android)
- Bocina

### Información adicional

- Compilador: xc16
- IDE: MPLAB X 5.25

