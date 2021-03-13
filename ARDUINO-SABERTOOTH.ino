// TAMAÑO DE PALABRA ENTRANTE
#define NUM 10   //tamaño de la fraze + 1 = NUM
                // ejemplo:
                // Hola  --> en realidad seria: Hola\n
                //  4                             5
#define BAUDIOS 9600
// LIBRERIA DRIVER
#include <SabertoothSimplified.h>

// TERMINAL VIRTUAL TX EN PIN 3
#include <SoftwareSerial.h>
SoftwareSerial SWSerial_1(NOT_A_PIN, 3);   // TX ES  PIN 3
SabertoothSimplified ST1(SWSerial_1); 

// TERMINAL ORIGINAL TX EN 11
SoftwareSerial SWSerial_2(NOT_A_PIN, 11);  // TX ES  PIN 11
SabertoothSimplified ST2(SWSerial_2); 

// TERMINAL RECEPTOR DE DATOS DE TIVA C
SoftwareSerial portOne(10, NOT_A_PIN);     // RX 10

// VARIABLES GLOBALES
// int power=0; 
char incomingChar=""; 
char palabra[NUM]=""; 
int i,J1;


int probe=0;
int probe2=0;

char delimitador[] = ",";
char *token;
int VAL1=0;
int VAL2=0;
int ACT=1;
int IN1=0;
int IN2=0;
int KEY = 1;

void RECEPCION(int *IN1,int *IN2);
  
void setup()
{
  Serial.begin(BAUDIOS);             //  CONSOLA
  pinMode(LED_BUILTIN, OUTPUT);   // INDICADOR
  SWSerial_1.begin(BAUDIOS);         //  MOTOR 1
  SWSerial_2.begin(BAUDIOS);         //  MOTOR 2
  portOne.begin(BAUDIOS);            //  RECEPCION TIVA
  //delay(3000);                  
  Serial.println("Configurado");
}


void loop()
{

    RECEPCION();
          
  
   
  
  
  
}

void RECEPCION(){
      
        
        if (portOne.available() > 0) {
          
              while (portOne.available() > 0) { // La redundancia evita error en recepcion
                digitalWrite(LED_BUILTIN, HIGH);
                      for (i=0;i<=NUM;i++){ 
                      incomingChar = portOne.read();
                        if (i<(NUM-1)){   
                        palabra[i]=incomingChar;}
                      delay(5);   // retraso necesario...
                      }
                  }    
              digitalWrite(LED_BUILTIN, LOW);
             // Serial.print("RECIBI: ");
            // Serial.println(palabra);
  
                // ASCII to INT
                
          token = strtok(palabra,delimitador);
          IN1=atoi(token);

          token = strtok(NULL, delimitador);
          IN2=atoi(token);
          
          ST1.motor(1, IN1);
          ST2.motor(1, IN2);
          
Serial.println(IN1);     
Serial.println(IN2);     

      }

}

void PRINTF (int VAL1, int VAL2){
  
    //Serial.print("VAL1: ");
    Serial.println(VAL1);
    //Serial.print("VAL2: ");
    //Serial.println(VAL2);

    //Serial.println("=======================");

    //delay(20);

}
  
