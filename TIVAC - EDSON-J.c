#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>     //atof

#include "inc/hw_memmap.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

#include "driverlib/interrupt.h"

#include <math.h>

//// DIRECTIVA ////
#define SIZE    38      // YPR VN300
#define SIZE2   1       // JETSON TX2
#define tiempo 0.05;        //50ms
///////////////////

//// DECLARACION DE VARIABLES GLOBALES ////
int tam = 38;
int num = 16;
int i, j, H,K,L;
int CASO = 0;
int signo = 0;          //0 positivo ... 1 negativo
int gaa;
int s1, s2, SAL1neg, SAL2neg, cifA, cifB, ii;
int TSAL1, TSAL2;
int count_0 = 0;
int NUMERO = 0;
int NUMERO2 = 0;
int contador = 0;
int fact;
int fact2=2;
int STOP=1;

int vA=80;
int vB=-80;
int vC=120;
int vD=-120;
int vE=40; //usado para zigzag
int vF=80;
int SALIR=1;


int alpha,beta;
int condicion=0;
int flag1=0;
int offsetZIGZAG=5;
int ang=10;

signed int SAL1 = 0;
signed int SAL2 = 0;

char IN2 = '-';
char *str1;         //conversion int1
char *str2;         //conversion int2
char salidaArduino[10] = "";

double YAW;
double PITCH;
double ROLL;
bool VAL = false;

const uint8_t COMANDO1[] = " $VNSIH,0*06DE\n";
const uint8_t COMANDO2[] = " $VNWRG,57,0,-0.3,0.02*74\n";
const uint8_t COMANDO3[] = " $VNWRG,93,-1.000,+0.000,+0.000,+0.0254,+0.0254,+0.0254*33FF\n";

char DATARX[SIZE] = "";
char DATARX2[SIZE2] = "-";
char delimitador[] = ",\n";
char *token;
/////////////////////////////////////////////

//// PROTOTIPOS DE FUNCIONES ADICIONALES ////
void INTtoSTRING ();
void RECEPCIONVN300 ();
int maxvalue (int num1, int num2);
int minvalue (int num1, int num2);
int cifras (int num);
void UARTIntHandler (void);
void UART3Salida (void);
void ConfigureUART0 (void);
void ConfigureUART3 (void);
void ConfigureUART7 (void);
void ConfigureInitial (void);
void RECEPCIONJetsontx2(void);
void STOPtheBOAT(void);
void RAMPAgiro(int A);
void mediavuelta(int A);

/////////////////////////////////////////////





int main (void)
{

ConfigureInitial ();
ConfigureUART3 ();      // FUNCION: RECIBE JETSON Y ENVIA ARDUINO, a 9600 BAUDIOS;
ConfigureUART7 ();        // FUNCION: RECIBE Y ENVIA VECTORNAV;
ConfigureUART0 ();        // FUNCION: VISUALIZAR EN TERMINAL 0;

UARTprintf ("SE CONFIGURO INICIALMENTE\n");
SysCtlDelay ((SysCtlClockGet () / 3) * 0.2);    // 0.2 segundos

for (H = 0; H < 1; H++)
{
    for (j = 0; j < sizeof (COMANDO1) - 1; j++)
    {
          UARTCharPut (UART7_BASE, COMANDO1[j]); // ENVIA CON CHARPUT UART7
    }
UARTprintf ("--CONFIGURADO COMANDO1--\n");
}

SysCtlDelay ((SysCtlClockGet () / 3) * 0.2);    // 0.2 segundos
for (K = 0; K < 1; K++)
{
    for (j = 0; j < sizeof (COMANDO2) - 1; j++)
    {
          UARTCharPut (UART7_BASE, COMANDO2[j]); // ENVIA CON CHARPUT UART7
    }
UARTprintf ("--CONFIGURADO COMANDO2--\n");
}
SysCtlDelay ((SysCtlClockGet () / 3) * 0.2);    // 0.2 segundos
for (L = 0; L < 1; L++)
{
    for (j = 0; j < sizeof (COMANDO3) - 1; j++)
    {
          UARTCharPut (UART7_BASE, COMANDO3[j]); // ENVIA CON CHARPUT UART7
    }
UARTprintf ("--CONFIGURADO COMANDO3--\n");
}
SysCtlDelay ((SysCtlClockGet () / 3) * 0.2);    // 0.2 segundos

SysCtlDelay ((15000000) * 0.2); // 200ms
H = 0;
CASO = 0;
SALIR=1;

RECEPCIONJetsontx2();
    while (1)
    {

        UARTprintf ("--INICIO BUCLE--\n");

        RECEPCIONVN300();
        RECEPCIONJetsontx2();

        // FSM
        switch (CASO)
        {
            case 0:     // BREAK
                SAL1 = 0;     //regresar a 0
                SAL2 = 0;
                INTtoSTRING ();
                if (IN2 == 'W')
                {
                    CASO = 10;  //CASO  1: PARA ADELANTE
                }
                else if (IN2 == 'S')
                {
                    CASO = 17;  //CASO  2: PARA ATRAS
                }
                else if (IN2 == 'F')
                {
                    CASO = 30;  //CASO FUNCION
                }
                else if (IN2 == 'M')
                {
                    CASO = 35;  //CASO FUNCION
                }
                else if (IN2 == 'N')
                {
                    CASO = 36;  //CASO FUNCION
                }
                else
                {
                    CASO = 0;
                }


                break;

            case 1:     // ADELANTE TECLA W
                SAL1 = vA;
                SAL2 = vA;
                INTtoSTRING ();
                if (IN2 == 'A')
                {
                    CASO = 13;  //CASO  3: PARA IZQUIERDA
                }
                else if (IN2 == 'D')
                {
                    CASO = 11;  //CASO  4: PARA DERECHA
                }
                else if (IN2 == 'B')
                {
                    CASO = 24;  //CASO   5: BREAK
                }
                else
                {
                    CASO = 1;
                }
                break;

            case 2:     // ATRAS TECLA S
                SAL1 = vB;
                SAL2 = vB;
                INTtoSTRING ();
                if (IN2 == 'A')
                {
                    CASO = 20;  //CASO  5: PARA IZQUIERDA ATRAS
                }
                else if (IN2 == 'D')
                {
                    CASO = 18;  //CASO  4: PARA DERECHA ATRAS
                }
                else if (IN2 == 'B')
                {
                    CASO = 24;  //CASO   5: BREAK
                }
                else
                {
                    CASO = 2;
                }
                break;

            case 3:     // IZQUIERDA DEFRENTE TECLA A
                SAL1 = vA;
                SAL2 = vC;
                INTtoSTRING ();
                if (IN2 == 'W')
                {
                    CASO = 14;  //CASO  1: PARA ADELANTE
                }
                else if (IN2 == 'D')
                {
                    CASO = 15;  //CASO  4: PARA DERECHA
                }
                else if (IN2 == 'B')
                {
                    CASO = 24;  //CASO   5: BREAK
                }
                else
                {
                    CASO = 3;
                }
                break;

            case 4:     // DERECHA DEFRENTE TECLA D
                SAL1 = vC;
                SAL2 = vA;
                INTtoSTRING ();
                if (IN2 == 'W')
                {
                    CASO = 12;  //CASO  1: PARA ADELANTE
                }
                else if (IN2 == 'A')
                {
                    CASO = 16;  //CASO  4: PARA IZQUIERDA
                }
                else if (IN2 == 'B')
                {
                    CASO = 24;  //CASO   5: BREAK
                }
                else
                {
                    CASO = 4;
                }
                break;

            case 5:     // IZQUEIRDA ATRAS
                SAL1 = vB;
                SAL2 = vD;
                INTtoSTRING ();
                if (IN2 == 'S')
                {
                    CASO = 21;  //CASO  2: PARA ATRAS
                }
                else if (IN2 == 'D')
                {
                    CASO = 22;  //CASO  6: PARA DERECHA
                }
                else if (IN2 == 'B')
                {
                    CASO = 24;  //CASO   5: BREAK
                }
                else
                {
                    CASO = 5;
                }
                break;

            case 6:     // DERECHA ATRAS
                SAL1 = vD;
                SAL2 = vB;
                INTtoSTRING ();
                if (IN2 == 'S')
                {
                    CASO = 19;  //CASO  2: PARA ATRAS
                }
                else if (IN2 == 'A')
                {
                    CASO = 23;  //CASO  5: PARA IZQUIERDA
                }
                else if (IN2 == 'B')
                {
                    CASO = 24;  //CASO   5: BREAK
                }
                else
                {
                    CASO = 6;
                }
                break;

            case 10:
                CASO = 1;

                for (ii = 0; ii < (vA/fact2); ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 += fact2;
                    SAL2 += fact2;

                    //CONVERSION 2 INTEGER A STRING
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vA/fact2);
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 11:
                CASO = 4;

                for (ii = 0; ii < (vC-vA)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 += fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vC-vA)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }

                }
                break;

            case 12:
                CASO = 1;

                for (ii = 0; ii < (vC-vA)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 -= fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vC-vA)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 13:
                CASO = 3;

                for (ii = 0; ii < (vC-vA)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL2 += fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vC-vA)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 14:
                CASO = 1;

                for (ii = 0; ii < (vC-vA)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL2 -= fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vC-vA)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 15:
                CASO = 4;

                for (ii = 0; ii < (vC-vA)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 += fact2;
                    SAL2 -= fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vC-vA)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 16:
                CASO = 3;

                for (ii = 0; ii < (vC-vA)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 -= fact2;
                    SAL2 += fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vC-vA)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 17:
                CASO = 2;

                for (ii = 0; ii < vA/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 -= fact2;
                    SAL2 -= fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=vA/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }

                break;

            case 18:
                CASO = 6;

                for (ii = 0; ii < (vB-vD)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();

                    SAL1 -= fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vB-vD)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 19:
                CASO = 2;

                for (ii = 0; ii < (vB-vD)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 += fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vB-vD)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 20:
                CASO = 5;

                for (ii = 0; ii < (vB-vD)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();

                    SAL2 -= fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vB-vD)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 21:
                CASO = 2;

                for (ii = 0; ii < (vB-vD)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL2 += fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vB-vD)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 22:
                CASO = 6;

                for (ii = 0; ii < (vB-vD)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 -= fact2;
                    SAL2 += fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vB-vD)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 23:
                CASO = 5;

                for (ii = 0; ii < (vB-vD)/fact2; ii++)
                {
                    RECEPCIONJetsontx2();
                    SAL1 += fact2;
                    SAL2 -= fact2;

                    //INICIO DE TRANSICION//
                    INTtoSTRING ();

                    //ENVIO DE COMANDOS A ARDUINO//
                    UART3Salida ();
                    if (IN2 == 'B')
                    {
                        STOPtheBOAT();
                        ii=(vB-vD)/fact2;
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                    }
                }
                break;

            case 24:
                STOPtheBOAT();
                CASO = 0;

                break;

            case 30: // caso ZIGZAG
                alpha=YAW; // REFERENCIA
                CASO=31;

                break;

            case 31: // go izquierda

                RAMPAgiro(0);
                SALIR=1;
                do
                {

                    RECEPCIONVN300(); // LECTURA DEL YAW
                    RECEPCIONJetsontx2(); // LECTURA DE TECLA


                    if (YAW > alpha+ang)  // DETECCION DEL YAW
                    {
                        SALIR=0;
                        CASO=32;  // SALE PARA GO IZQUIERDA
                        IN2='-';
                    }
                    SysCtlDelay(3);

                    if (IN2 == 'B')  // SI SE PRESIONA B
                    {
                        STOPtheBOAT();
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                        SALIR=0;
                    }
                }while(SALIR);

                SysCtlDelay(3);
                break;

            case 32:
                RAMPAgiro(1);
                SALIR=1;
                do
                {

                    RECEPCIONVN300(); // LECTURA DEL YAW
                    RECEPCIONJetsontx2(); // LECTURA DE TECLA


                    if (YAW < alpha-ang)  // DETECCION DEL YAW
                    {
                        SALIR=0;
                        CASO=33;  //
                        IN2='-';
                    }
                    SysCtlDelay(3);

                    if (IN2 == 'B')  // SI SE PRESIONA B
                    {
                        STOPtheBOAT();
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                        SALIR=0;
                    }
                }while(SALIR);

                SysCtlDelay(3);
                break;


            case 33: // go derecha
                RAMPAgiro(2);
                SALIR=1;
                do
                {

                    RECEPCIONVN300(); // LECTURA DEL YAW
                    RECEPCIONJetsontx2(); // LECTURA DE TECLA


                    if (YAW > alpha+ang)  // DETECCION DEL YAW
                    {
                        SALIR=0;
                        CASO=32;  //
                    }
                    SysCtlDelay(3);

                    if (IN2 == 'B')  // SI SE PRESIONA B
                    {
                        STOPtheBOAT();
                        IN2='-';
                        CASO=0;
                        SAL1 = 0;
                        SAL2 = 0;
                        SALIR=0;
                    }
                }while(SALIR);

                SysCtlDelay(3);
                break;

            case 35:  //SAL1 AUMENTA, SAL2 DISMINUYE
                do
                {
                    mediavuelta(0);
                    SALIR=0;
                }while(SALIR);
                CASO=37;


                break;
            case 36: //SAL1 DISMINUYE, SAL2 AUMENTA
                do
                {
                    mediavuelta(1);
                    SALIR = 0;
                }while(SALIR);
                CASO=37;
                break;

            case 37: //transicion

                if (IN2 == 'B')  // SI SE PRESIONA B
                {
                    STOPtheBOAT();
                    IN2='-';
                    CASO=0;
                    SAL1 = 0;
                    SAL2 = 0;
                    SALIR=0;
                }
                break;

            default:
                break;
        }
        /////////// FIN FSM ///////////

        UARTprintf ("============fin bucle========\n");
        ////////// ENVIO DE COMANDOS A ARDUINO  ////////////
        UART3Salida ();
        contador++;
    }

}


int cifras (int num)
{
    int count;
    count = (num == 0) ? 1 : (log10 (num) + 1);
    return count;
}

int maxvalue (int num1, int num2)
{
    int max;
        if (num1 > num2)
        {
            max = num1;
        }
        else if (num1 < num2)
        {
            max = num2;
        }
        else
        {
            max = num1;     // cualquiera... son iguales
        }
        return max;
}

int minvalue (int num1, int num2)
{
    int min;
        if (num1 > num2)
        {
            min = num2;
        }
        else if (num1 < num2)
        {
            min = num1;
        }
        else
        {
            min = num1;     // cualquiera... son iguales
        }
        return min;
}

void RECEPCIONVN300 ()
{
    if (UARTCharsAvail (UART7_BASE) > 0)
    {
        while (UARTCharsAvail (UART7_BASE))
        {
            SysCtlDelay (3);
            while (NUMERO < SIZE)
            {
                DATARX[NUMERO] = UARTCharGet (UART7_BASE);
                SysCtlDelay (3);
                NUMERO++;
            }
        }
        NUMERO = 0;
    }
    char salida[38] = "";
    char salida2[38] = "";
    for (i = 0; i < (tam - num); i++)
    {
        salida[i] = DATARX[i + num - 1];
    }
    for (i = 0; i < (num - 2); i++)
    {
        salida2[i] = DATARX[i];
    }
    strcat (salida, salida2);
    UARTprintf ("RECIBIDO: %s \n", salida);
    contador++;
    if (contador > 10)
    {
        token = strtok (salida, delimitador);
        UARTprintf ("TOKEN 1: %s \n", token);
        if (strcmp (token, " $VNYPR"))
        {           //cuidado con el stapcio
            VAL = true;
        }
        else
        {
            VAL = false;
        }
        while (VAL)
        {
            token = strtok (NULL, delimitador);
            count_0++;
            switch (count_0)
            {
                case 1:
                    YAW = atof (token);
                    break;

                case 2:
                    PITCH = atof (token);
                    break;

                case 3:
                    ROLL = atof (token);
                    VAL = false;
                    count_0 = 0;
                    break;

                default:
                    break;

            }
        }
    }
}

void INTtoSTRING ()
{
    /////// GENERADOR DE STRING DE 10 DE TAMACO CON 2 INTEGERS
    if (SAL1 > 0 && SAL2 > 0)
    {
        cifA = cifras (SAL1);
        cifB = cifras (SAL2);
        if (cifA == 1 && cifB == 1)
        {
            snprintf (salidaArduino, 10, "+00%d,+00%d\n", SAL1, SAL2);
        }
        else if (cifA == 2 && cifB == 1)
        {
            snprintf (salidaArduino, 10, "+0%d,+00%d\n", SAL1, SAL2);
        }
        else if (cifA == 3 && cifB == 1)
        {
            snprintf (salidaArduino, 10, "+%d,+00%d\n", SAL1, SAL2);
        }
        else if (cifA == 1 && cifB == 2)
        {
            snprintf (salidaArduino, 10, "+00%d,+0%d\n", SAL1, SAL2);
        }
        else if (cifA == 2 && cifB == 2)
        {
            snprintf (salidaArduino, 10, "+0%d,+0%d\n", SAL1, SAL2);
        }
        else if (cifA == 3 && cifB == 2)
        {
            snprintf (salidaArduino, 10, "+%d,+0%d\n", SAL1, SAL2);
        }
        else if (cifA == 1 && cifB == 3)
        {
            snprintf (salidaArduino, 10, "+00%d,+%d\n", SAL1, SAL2);
        }
        else if (cifA == 2 && cifB == 3)
        {
            snprintf (salidaArduino, 10, "+0%d,+%d\n", SAL1, SAL2);
        }
        else if (cifA == 3 && cifB == 3)
        {
            snprintf (salidaArduino, 10, "+%d,+%d\n", SAL1, SAL2);
        }
    }
    else if (SAL1 == 0 && SAL2 == 0)
    {
        snprintf (salidaArduino, 10, "+000,+000\n");
    }
    else if (SAL1 < 0 && SAL2 < 0)
    {
        SAL1neg = SAL1 * (-1);
        SAL2neg = SAL2 * (-1);
        cifA = cifras (SAL1neg);
        cifB = cifras (SAL2neg);
        if (cifA == 1 && cifB == 1)
        {
            snprintf (salidaArduino, 10, "-00%d,-00%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 2 && cifB == 1)
        {
            snprintf (salidaArduino, 10, "-0%d,-00%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 3 && cifB == 1)
        {
            snprintf (salidaArduino, 10, "-%d,-00%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 1 && cifB == 2)
        {
            snprintf (salidaArduino, 10, "-00%d,-0%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 2 && cifB == 2)
        {
            snprintf (salidaArduino, 10, "-0%d,-0%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 3 && cifB == 2)
        {
            snprintf (salidaArduino, 10, "-%d,-0%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 1 && cifB == 3)
        {
            snprintf (salidaArduino, 10, "-00%d,-%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 2 && cifB == 3)
        {
            snprintf (salidaArduino, 10, "-0%d,-%d\n", SAL1neg, SAL2neg);
        }
        else if (cifA == 3 && cifB == 3)
        {
            snprintf (salidaArduino, 10, "-%d,-%d\n", SAL1neg, SAL2neg);
        }
    }
    else if (SAL1 == 0 && SAL2 < 0)
    {
        SAL2neg = SAL2 * (-1);
        cifB = cifras (SAL2neg);
        if (cifB == 1)
        {
            snprintf (salidaArduino, 10, "+000,-00%d\n", SAL2neg);
        }
        else if (cifB == 2)
        {
            snprintf (salidaArduino, 10, "+000,-0%d\n", SAL2neg);
        }
        else if (cifB == 3)
        {
            snprintf (salidaArduino, 10, "+000,-%d\n", SAL2neg);
        }
    }
    else if (SAL1 < 0 && SAL2 == 0)
    {
        SAL1neg = SAL1 * (-1);
        cifA = cifras (SAL1neg);
        if (cifA == 1)
        {
            snprintf (salidaArduino, 10, "-00%d,+000\n", SAL1neg);
        }
        else if (cifA == 2)
        {
            snprintf (salidaArduino, 10, "-0%d,+000\n", SAL1neg);
        }
        else if (cifA == 3)
        {
            snprintf (salidaArduino, 10, "-%d,+000\n", SAL1neg);
        }
    }
    else if (SAL1 > 0 && SAL2 == 0)
    {
      cifA = cifras (SAL1);
      if (cifA == 1)
      {
          snprintf (salidaArduino, 10, "+00%d,+000\n", SAL1);
      }
      else if (cifA == 2)
      {
          snprintf (salidaArduino, 10, "+0%d,+000\n", SAL1);
      }
      else if (cifA == 3)
      {
          snprintf (salidaArduino, 10, "+%d,+000\n", SAL1);
      }
    }
    else if (SAL1 == 0 && SAL2 > 0)
    {
        cifB = cifras (SAL2);
        if (cifB == 1)
        {
            snprintf (salidaArduino, 10, "+000,+00%d\n", SAL2);
        }
        else if (cifB == 2)
        {
            snprintf (salidaArduino, 10, "+000,+0%d\n", SAL2);
        }
        else if (cifB == 3)
        {
            snprintf (salidaArduino, 10, "+000,+%d\n", SAL2);
        }
    }
}
void UARTIntHandler (void)
{
uint32_t ui32Status;
ui32Status = UARTIntStatus (UART3_BASE, true);  //get interrupt status
UARTIntClear (UART3_BASE, ui32Status);    //clear the asserted interrupts
}

void ConfigureUART0 ()
{
MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOA);
SysCtlDelay (3);
MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_UART0);
SysCtlDelay (3);
MAP_GPIOPinConfigure (GPIO_PA0_U0RX);
MAP_GPIOPinConfigure (GPIO_PA1_U0TX);
MAP_GPIOPinTypeUART (GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
UARTClockSourceSet (UART0_BASE, UART_CLOCK_PIOSC);
UARTStdioConfig (0, 115200, 16000000);
}

void ConfigureUART3 (void)
{
MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_UART3);
SysCtlDelay (3);
MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC);
SysCtlDelay (3);
MAP_GPIOPinConfigure (GPIO_PC7_U3TX);
MAP_GPIOPinConfigure (GPIO_PC6_U3RX);
MAP_GPIOPinTypeUART (GPIO_PORTC_BASE, GPIO_PIN_7 | GPIO_PIN_6);
MAP_UARTConfigSetExpClk (UART3_BASE, MAP_SysCtlClockGet (), 9600,
                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                 UART_CONFIG_PAR_NONE));
}

void ConfigureUART7 (void)
{
MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_UART7);
SysCtlDelay (3);
MAP_SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOE);
SysCtlDelay (3);
MAP_GPIOPinConfigure (GPIO_PE0_U7RX);
MAP_GPIOPinConfigure (GPIO_PE1_U7TX);
MAP_GPIOPinTypeUART (GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
MAP_UARTConfigSetExpClk (UART7_BASE, MAP_SysCtlClockGet (), 115200,
                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                 UART_CONFIG_PAR_NONE));
}

void ConfigureInitial (void)
{
MAP_FPULazyStackingEnable ();
SysCtlDelay (3);
MAP_SysCtlClockSet (SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
SYSCTL_OSC_MAIN);   //50mhz
}


void UART3Salida (void){
    for (H = 0; H < 1; H++)
    {
        for (j = 0; j < 10; j++)
        {       // ENVIA CON CHARPUT UART3
            UARTCharPut (UART3_BASE, salidaArduino[j]);
            SysCtlDelay (3);
        }
    }
    SysCtlDelay (833333 * 2);   //50milisegundos ASEGURA 20 Hz
}
void RECEPCIONJetsontx2(void){
    if (UARTCharsAvail (UART3_BASE) > 0)
    {
        while (UARTCharsAvail (UART3_BASE)) //loop while there are chars
        {
            SysCtlDelay (3);
            while (NUMERO2 < SIZE2)
            {
                DATARX2[NUMERO2] = UARTCharGet (UART3_BASE);
                SysCtlDelay (3);
                NUMERO2++;
            }
        }
        NUMERO2 = 0;
    }
    IN2 = DATARX2[0];
}
void STOPtheBOAT(void){
    fact=5;
    do
    {
        STOP=1;
        if (SAL1 < 0  && SAL2< 0)
        {
            if (SAL1 > fact){
                SAL1=0;
            }else
            {
                SAL1+=fact;
            }
            if (SAL2 > fact)
            {
                SAL2=0;
            }else
            {
                SAL2+=fact;
            }
        }

        if (SAL1 > 0  && SAL2> 0)
        {
            if (SAL1 < fact){
                SAL1=0;
            }else
            {
                SAL1-=fact;
            }
            if (SAL2 < fact)
            {
                SAL2=0;
            }else
            {
                SAL2-=fact;
            }
        }

        if (SAL1<0 && SAL2==0)
        {
            if (SAL1 > fact){
                SAL1=0;
            }else
            {
                SAL1+=fact;
            }
        }

        if (SAL1>0 && SAL2==0)
        {
            if (SAL1 < fact){
                SAL1=0;
            }else
            {
                SAL1-=fact;
            }
        }

        if (SAL1==0 && SAL2<0)
        {
            if (SAL2 > fact)
            {
                SAL2=0;
            }else
            {
                SAL2+=fact;
            }
        }
        if (SAL1==0 && SAL2>0)
        {
            if (SAL2 < fact)
            {
                SAL2=0;
            }else
            {
                SAL2-=fact;
            }
        }
        if (SAL1<0 && SAL2>0)
        {
            if (SAL1 > fact){
                SAL1=0;
            }else
            {
                SAL1+=fact;
            }
            if (SAL2 < fact)
            {
                SAL2=0;
            }else
            {
                SAL2-=fact;
            }
        }
        if (SAL1>0 && SAL2<0)
        {
            if (SAL1 < fact){
                SAL1=0;
            }else
            {
                SAL1-=fact;
            }
            if (SAL2 > fact)
            {
                SAL2=0;
            }else
            {
                SAL2+=fact;
            }
        }

        if (SAL1==0 && SAL2==0)
        {
            STOP=0;
        }

    // EXPORT VALUE RAMP
        //INICIO DE TRANSICION//
        INTtoSTRING ();
        //ENVIO DE COMANDOS A ARDUINO//
        UART3Salida ();

    }while(STOP);

    if (SAL1 != 0 && SAL2 != 0){
        SAL1=0;
        SAL2=0;
        //INICIO DE TRANSICION//
        INTtoSTRING ();
        //ENVIO DE COMANDOS A ARDUINO//
        UART3Salida ();
    }
}

void RAMPAgiro(int A)
{
    if (A==0)  { //partir de 0 a izquierda
        for (ii=0; ii<vE;ii++)
        {
            RECEPCIONVN300(); // LECTURA DEL YAW
            RECEPCIONJetsontx2();
            SAL1+=1;
            SAL2+=2;
            //INICIO DE TRANSICION//
            INTtoSTRING ();

            //ENVIO DE COMANDOS A ARDUINO//
            UART3Salida ();
            //printf("%d,%d\n",sal1,sal2);
            if (IN2=='B'){
                STOPtheBOAT();
                ii=vE;
                CASO=0;
                SALIR=0;
            }
        }
    }
    if (A==1)  { // izquierda a derecha
        for (ii=0; ii<vE;ii++)
        {
            RECEPCIONVN300(); // LECTURA DEL YAW
            RECEPCIONJetsontx2();
            SAL1+=1;
            SAL2-=1;

            //INICIO DE TRANSICION//
            INTtoSTRING ();

            //ENVIO DE COMANDOS A ARDUINO//
            UART3Salida ();

            //printf("%d,%d\n",sal1,sal2);
            if (IN2=='B'){
                            STOPtheBOAT();
                            ii=vE;
                            CASO=0;
                            SALIR=0;
                        }
        }
    }
    if (A==2)   { //derecha a izquierda
        for (ii=0; ii<vE;ii++)
        {
            RECEPCIONVN300(); // LECTURA DEL YAW
            RECEPCIONJetsontx2();
            SAL1-=1;
            SAL2+=1;

            //INICIO DE TRANSICION//
            INTtoSTRING ();

            //ENVIO DE COMANDOS A ARDUINO//
            UART3Salida ();

            //printf("%d,%d\n",sal1,sal2);
            if (IN2=='B'){
                            STOPtheBOAT();
                            ii=vE;
                            CASO=0;
                            SALIR=0;
                        }
        }


    }

}

void mediavuelta(int A)
{

    if (A==0){
        for (ii=0;ii<vF/fact2;ii++)
        {

            SAL1+=fact2;
            SAL2-=fact2;

            //INICIO DE TRANSICION//
            INTtoSTRING ();

            //ENVIO DE COMANDOS A ARDUINO//
            UART3Salida ();


        }
        if (SAL1 != vF && SAL2 != vF )
        {
            SAL1 =vF;
            SAL2 =-vF;
            //INICIO DE TRANSICION//
            INTtoSTRING ();

            //ENVIO DE COMANDOS A ARDUINO//
            UART3Salida ();
        }

    }

    if (A==1){
        for (ii=0;ii<vF/fact2;ii++)
        {
            SAL1 -=fact2;
            SAL2 +=fact2;
            //INICIO DE TRANSICION//
            INTtoSTRING ();

            //ENVIO DE COMANDOS A ARDUINO//
            UART3Salida ();

        }
        if (SAL1 != vF && SAL2 != vF )
        {
            SAL1 =-vF;
            SAL2 =+vF;
            //INICIO DE TRANSICION//
            INTtoSTRING ();

            //ENVIO DE COMANDOS A ARDUINO//
            UART3Salida ();
        }

    }


}
