#include <IRremote.h>
#define portInfra   7    //definiçao da porta do infracermelho no arduino
#define gndInfra    8     // definiçao de porta 0
#define vccInfra    9    // definiçao de porta 1

#define  ent1Motor1  3   //pino1 de entrada da ponteH do motor1
#define  ent2Motor1  4   //pino2 de entrada da ponteH do motor1 
#define  ent1Motor2  5   //pino1 de entrada da ponteH do motor2
#define  ent2Motor2  6   //pino2 de entrada da ponteH do motor2

IRrecv recInfra(portInfra );    //classe para localizar o pino do arduino
decode_results sinal ;    //valor do sinal que ele recebe do infravermelho

void setup () {
  pinMode(ent1Motor1, OUTPUT);
  pinMode(ent2Motor1, OUTPUT);
  pinMode(ent1Motor2, OUTPUT);
  pinMode(ent2Motor2, OUTPUT);
  pinMode(gndInfra, OUTPUT );
  pinMode(vccInfra, OUTPUT );
  Serial.begin(9600 );   //ver valor lido
  recInfra.enableIRIn();    //iniciar recepitor infravermelho
}
void frente (){
  digitalWrite(ent1Motor1,LOW); //definiçao para virar par 
  digitalWrite(ent2Motor1,HIGH);
  digitalWrite(ent1Motor2,LOW); //definiçao para virar par 
  digitalWrite(ent2Motor2,HIGH);
  delay(2000);    //definiçao de tempo
  }
void pratras (){
  digitalWrite(ent1Motor1, HIGH); //definiçao para virar pra 
  digitalWrite(ent2Motor1, LOW );
  digitalWrite(ent1Motor2, HIGH ); //definiçao para virar pra 
  digitalWrite(ent2Motor2, LOW );
  delay (2000);    //definiçao de tempo
  }
void parar (){
  digitalWrite(ent1Motor1, LOW); //definiçao par parar 
  digitalWrite(ent2Motor1, LOW);
  digitalWrite(ent1Motor2, LOW); //definiçao par parar 
  digitalWrite(ent2Motor2, LOW);
  delay(2000);    //definiçao de tempo
}
void result (){
//se indentificar o valor lido reservar o valor
  if(recInfra.decode(&sinal)){
    switch(sinal.value){
      case 0xE0E006F9://valor do botao precionado em hex para frente 
        frente();
      break;
      case 0xE0E08679://valor do botao precionado em hex pratras 
        pratras();
      break;
      case 0xE0E016E9://valor do botao precionado em hex para parar
        parar();
      }
    recInfra.resume();    // continuar lendo
    }
}    
void loop() {
  digitalWrite(gndInfra, 0);
  digitalWrite(vccInfra, 1);
  result();
}
