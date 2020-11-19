#include <IRremote.h>
#define portInfra   6     //definiçao da porta do infracermelho no arduino
#define gndInfra    7     // definiçao de porta 0
#define vccInfra    8     // definiçao de porta 1

#define  ent1Motor1  3    //pino1 de entrada da ponteH do motor1
#define  ent2Motor1  9    //pino2 de entrada da ponteH do motor1 
#define  ent1Motor2  10   //pino1 de entrada da ponteH do motor2
#define  ent2Motor2  11   //pino2 de entrada da ponteH do motor2

IRrecv recInfra(portInfra );    //classe para localizar o pino do arduino
decode_results sinal ;          //valor do sinal que ele recebe do infravermelho

void init_inframotores() {
  pinMode(ent1Motor1, OUTPUT);
  pinMode(ent2Motor1, OUTPUT);
  pinMode(ent1Motor2, OUTPUT);
  pinMode(ent2Motor2, OUTPUT);
  pinMode(gndInfra, OUTPUT );
  pinMode(vccInfra, OUTPUT );
  pinMode(portInfra,INPUT);
  Serial.begin(9600 );   //ver valor lido
  recInfra.enableIRIn();    //iniciar recepitor infravermelho
}
void derecMotores(double comando){
  if(comando > 0){                               // se o comando for maior que 0 o robo anda pra frente 
   analogWrite(ent2Motor2, abs(comando));        //motor2 andando para frente 
   analogWrite(ent2Motor1, 0);                   //motor1 andando pra tras
   analogWrite(ent1Motor2, 0);                   //motor2 andando para tras
   analogWrite(ent1Motor1, abs(comando));        //motor1 andando pra frente
    }else{                                       // se o comando for manor que 0 o robo anda pra tras
   analogWrite(ent2Motor2, 0);                   //motor2 andando para frente 
   analogWrite(ent2Motor1, abs(comando));        //motor1 andando pra tras
   analogWrite(ent2Motor1, abs(comando));        //motor2 andando para tras
   analogWrite(ent1Motor1, 0);                   //motor1 andando pra frente
      }
 
}

void result (){
//se indentificar o valor lido reservar o valor
  if(recInfra.decode(&sinal)){
    switch(sinal.value){
      case 0xE0E006F9://valor do botao precionado em hex para frente 
        ent2Motor2, ent1Motor1;
      break;
      case 0xE0E08679://valor do botao precionado em hex pratras 
        ent2Motor1,ent2Motor1 ;
      break;
      }
    recInfra.resume();    // continuar lendo
    }
}    
void loop() {
  digitalWrite(gndInfra, 0);
  digitalWrite(vccInfra, 1);
  result();
}
