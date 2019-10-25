#include <Shifter.h>
#include <EEPROM.h>   
#define SER_Pin 2 
#define RCLK_Pin 3 
#define SRCLK_Pin 4 
#define NUM_REGISTERS 3 
Shifter shifter(SER_Pin, RCLK_Pin, SRCLK_Pin, NUM_REGISTERS); 


 
  void gravar_memoria(byte memoria, int valor_para_gravar){EEPROM.write(memoria, highByte(valor_para_gravar)); 
  EEPROM.write(memoria + 256, lowByte(valor_para_gravar));}
  int memoria_lida[255];        

  void le_memoria(int endereco_memoria){ byte high = EEPROM.read(endereco_memoria);          
  byte low = EEPROM.read(endereco_memoria + 256);  memoria_lida[endereco_memoria] = (high << 8) + low; }
  
  

boolean bot1ok=0,bot2ok=0,bot3ok=0, soma=0, dim=0;
byte rele1=5, rele2=6, somabot1=0, somabot2=0, somabot3=0;
byte pinSensor = A2, pinos[8]={7, 10, 1, 2, 3, 6, 15, 5},  leds[10]={8, 7, 9, 10, 11, 12, 13, A3, A4, A5}, dig[8] = {4,12,11,14,0,0,0,0};   
byte digito_set_seg[19][8] = { 
{ 0,0,0,0,0,0,1,1 }, // = 0
{ 1,0,0,1,1,1,1,1 }, // = 1
{ 0,0,1,0,0,1,0,1 }, // = 2
{ 0,0,0,0,1,1,0,1 }, // = 3
{ 1,0,0,1,1,0,0,1 }, // = 4
{ 0,1,0,0,1,0,0,1 }, // = 5
{ 0,1,0,0,0,0,0,1 }, // = 6
{ 0,0,0,1,1,1,1,1 }, // = 7
{ 0,0,0,0,0,0,0,1 }, // = 8
{ 0,0,0,0,1,0,0,1 }, // = 9
{ 1,1,1,1,1,1,0,1 }, // = -
{ 1,1,1,0,1,1,1,1 }, // = _   11
{ 1,1,1,0,1,1,0,1 }, // = =   12
{ 1,1,1,0,0,0,1,1 }, // = L
{ 0,0,1,1,0,0,0,0 }, // = P   14
{ 0,0,0,1,0,0,0,0 }, // = A   15
{ 0,1,1,0,0,0,0,0 }, // = E   16
{ 0,1,1,1,0,0,0,0 }, //=F 17
{ 0,1,1,1,0,0,0,0 }};
char lido='Z', lid[3]={'0', '0', '0'};
byte leituras=0, x=0, espera=0;
int valorSensor = 0, valorSensorcont=0, valorPor=0, ma1=0, ma2=0, mp1=0, mp2=0;
int  digito2=0, digito3=0, digito4=0, valormax=0, tempMax=0, segundo=0;




void tresDig(int valorSens){ digito2 = valorSens / 100;  digito3 = (valorSens - digito2 * 100) / 10; digito4 = valorSens - digito2 * 100 - digito3 * 10;
                escreve_set_seg(2, digito2);     escreve_set_seg(3, digito3); escreve_set_seg(4, digito4); 
                
                
             
}
void escreve_set_seg(byte x, byte digit) { shifter.clear();  shifter.write();   byte pin=0; for (byte segCount = 0; segCount < 8; ++segCount) { 
                                           shifter.setPin(pinos[pin], digito_set_seg[digit][segCount]); ++pin;} shifter.setPin(dig[x-1], HIGH); shifter.write(); 
                                           valorSensorcont = valorSensorcont + analogRead(pinSensor);  leituras++;
                                           if(leituras>=30){ valorSensor = valorSensorcont/30; valorSensorcont=0;
                                           valorSensor = map(valorSensor, mp1, 1023, 0, mp2); leituras=0;}}

void setup(){ Serial.begin(9600); //  gravar_memoria(12,0); gravar_memoria(13,400);  gravar_memoria(10,100); gravar_memoria(11,200);
shifter.clear();  shifter.write();
le_memoria(10);le_memoria(11);le_memoria(12);le_memoria(13);le_memoria(20);le_memoria(21);

ma1=memoria_lida[10];ma2=memoria_lida[11];mp1=memoria_lida[12];mp2=memoria_lida[13];
valormax=memoria_lida[20];
for (int x = 0; x < 10; x++)  {pinMode(leds[x], OUTPUT); digitalWrite(leds[x], LOW); }

Serial.print("pico maximo de pressao: "); Serial.print(memoria_lida[20]); Serial.print(", tempo com presao acima do valor definido: ");Serial.print(memoria_lida[21]);
Serial.print(" minutos...");
pinMode(rele1,OUTPUT); pinMode(3,OUTPUT); pinMode(2,OUTPUT); pinMode(4,OUTPUT); 
pinMode(rele2,OUTPUT);
digitalWrite(rele1,LOW);
digitalWrite(rele2,LOW); 


}
void loop(){ 

if (Serial.available() > 0) { lido = Serial.read();}
if (lido=='A'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[0], Serial.read()); lido = ' ';} 
if (lido=='B'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[1], Serial.read()); lido = ' ';}
if (lido=='C'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[2], Serial.read()); lido = ' ';}
if (lido=='D'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[3], Serial.read()); lido = ' ';}
if (lido=='E'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[4], Serial.read()); lido = ' ';}
if (lido=='F'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[5], Serial.read()); lido = ' ';}
if (lido=='G'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[6], Serial.read()); lido = ' ';}
if (lido=='H'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[7], Serial.read()); lido = ' ';}
if (lido=='I'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[8], Serial.read()); lido = ' ';}
if (lido=='J'){while(Serial.available() == 0){delay(1);} digitalWrite(leds[9], Serial.read()); lido = ' ';}
                
if (lido=='t'){              
for (int x = 0; x < 10; x++)  { digitalWrite(leds[x], 1); delay(100);}
for (int x = 0; x < 10; x++)  {digitalWrite(leds[x], LOW);delay(100); }}

if (lido=='T'){              
for (int x = 0; x < 10; x++)  { digitalWrite(leds[x], 1); delay(100);}
for (int x = 11; x > 0; x--)  {digitalWrite(leds[x-1], LOW);delay(100);}delay(300);}


if (lido=='s'){              
for (int x = 0; x < 10; x++)  { digitalWrite(leds[x], 1); }
for (int x = 11; x > 0; x--)  {digitalWrite(leds[x-1], LOW);delay(100);}}

if (lido=='S'){              
for (int x = 0; x < 10; x++)  { digitalWrite(leds[x], 1);delay(100); }
for (int x = 11; x > 0; x--)  {digitalWrite(leds[x-1], LOW);}}

if (lido=='P'){              
for (int x = 0; x < 10; x++)  { digitalWrite(leds[x], 1); }delay(100);
for (int x = 11; x > 0; x--)  {digitalWrite(leds[x-1], LOW);}delay(100);}

if (lido=='p'){
  
digitalWrite(leds[0], 1); 
digitalWrite(leds[2], 1);
digitalWrite(leds[4], 1);
digitalWrite(leds[6], 1);
digitalWrite(leds[8], 1);
delay(100);
for (int x = 11; x > 0; x--)  {digitalWrite(leds[x-1], LOW);}
delay(100);
digitalWrite(leds[1], 1); 
digitalWrite(leds[3], 1);
digitalWrite(leds[5], 1);
digitalWrite(leds[7], 1);
digitalWrite(leds[9], 1);
delay(100);
for (int x = 11; x > 0; x--)  {digitalWrite(leds[x-1], LOW);}
delay(100);
}


if (lido=='W'){while(Serial.available() == 0){delayMicroseconds(1);}  lido = Serial.read();

}



if (lido=='Z'){

valorPor = map(valorSensor, 0, ma2, 0, 10);
for (int x = 0; x < 10; x++)  {digitalWrite(leds[x], LOW); }
for (int x = 0; x < valorPor; x++)  { digitalWrite(leds[x], 1); }

if (valorSensor < 0 ) {valorSensor=0;}tresDig(valorSensor);

if(valorSensor >= ma2){escreve_set_seg(1, 16);}else{ if(valorSensor >= ma1){escreve_set_seg(1, 15);}else{escreve_set_seg(1, 13);}}


if(valorSensor >= ma1) {digitalWrite(rele1, HIGH);} if(valorSensor < ma1 - 5){digitalWrite(rele1, LOW);}

if (valorSensor >= ma2) {digitalWrite(rele2, HIGH); tempMax++;}    if (valorSensor < ma2 - 5){digitalWrite(rele2, LOW);}
if(tempMax>1000){tempMax=0; segundo++;}
if(segundo>50){segundo=0; int w = memoria_lida[21]++; gravar_memoria(21, w);}


if(valormax < valorSensor){valormax = valorSensor; gravar_memoria(20, valormax);}




}




}

 
