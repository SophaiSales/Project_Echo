#include <IRremote.h>

#define portInfra 11    //definiçao da porta do infracermelho
//Motor1
#define velMotor1 3    //pino de velocidade da ponteH do motor1 
#define ent1Motor1 4   //pino1 de entrada da ponteH do motor1
#define ent2Motor1 5   //pino2 de entrada da ponteH do motor1 
//Motor2
#define velMotor2 6    //pino de velocidade da ponteH do motor2 
#define ent1Motor2 7   //pino1 de entrada da ponteH do motor2
#define ent2Motor2 8   //pino2 de entrada da ponteH do motor2 
#define tmp(3000);
#define tmp2(200);    //tempo de parar pra virar  


int velo=255;

IRrecv recInfra(portInfra);    //classe para localizar o pino do arduino
decode_results sinal;    //valor do sinal que ele recebe do infravermelho

void frente(){
  analogWrite(velMotor1,velo); //definiçao da velocidade
  digitalWrite(ent1Motor1,LOW); //definiçao para virar par 
  digitalWrite(ent2Motor1,HIGH);
  delay(tmp); //definiçao de tempo

  analogWrite(velMotor2,velo); //definiçao da velocidade
  digitalWrite(ent1Motor2,LOW); //definiçao para virar par 
  digitalWrite(ent2Motor2,HIGH);
  delay(tmp); //definiçao de tempo
  }
void pratras(){
  analogWrite(velMotor2,velo); //definiçao da velocidade
  digitalWrite(ent1Motor1,HIGH); //definiçao para virar pra 
  digitalWrite(ent2Motor1,LOW);
  delay(tmp);
  
  analogWrite(velMotor2,velo); //definiçao da velocidade  
  digitalWrite(ent1Motor2,HIGH); //definiçao para virar pra 
  digitalWrite(ent2Motor2,LOW);
  delay(tmp);
  }
void parar(){
  digitalWrite(ent1Motor1,LOW); //definiçao par parar 
  digitalWrite(ent2Motor1,LOW);
  delay(tmp2);

  digitalWrite(ent1Motor2,LOW); //definiçao par parar 
  digitalWrite(ent2Motor2,LOW);
  delay(tmp2);
  
  }
void setup() { 
  pinMode(velMotor1,OUTPUT);   //Motro1 definiçao de entrada
  pinMode(ent1Motor1,OUTPUT);
  pinMode(ent2Motor1,OUTPUT);
  analogWrite(velMotor1,velo);   //definiçao da velocidade
  pinMode(velMotor2,OUTPUT);     //Motro2 definiçao de entrada
  pinMode(ent1Motor2,OUTPUT);
  pinMode(ent2Motor2,OUTPUT);
  analogWrite(velMotor2,velo);   //definiçao da velocidade
  Serial.begin(9600);   //ver valor lido
  recInfra.enableIRIn();    //iniciar recepitor infravermelho
}

void loop() {
//se indentificar o valor lido reservar o valor
  if(recInfra.decode(&sinal)){
    switch(sinal.value){
      case----://valor do botao precionado em hex para frente 
      break;
      case----://valor do botao precionado em hex pratras 
      break;
      case----://valor do botao precionado em hex para parar
      break;
      case----://valor do botao precionado em hex para esquerda
      break;
      case----://valor do botao precionado em hex para direita
      break;
      }
    recInfra.resume();    // continuar lendo
    
    }
  
}
