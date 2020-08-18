#include <IRremote.h>
#define portInfra 11    //definiçao da porta do infracermelho no arduino

IRrecv recInfra(portInfra);    //classe para localizar o pino do arduino
decode_results sinal;    //valor do sinal que ele recebe do infravermelho

void result(){
//se indentificar o valor lido reservar o valor
  if(recInfra.decode(&sinal)){
    switch(sinal.value){
      case 0xFF629D://valor do botao precionado em hex para frente 
        frente()
      break;
      case 0xFF629D://valor do botao precionado em hex pratras 
        pratras()
      break;
      case 0xFF629D://valor do botao precionado em hex para parar
        parar()
      break;
      case 0xFF629D://valor do botao precionado em hex para esquerda
      break;
      case 0xFF629D://valor do botao precionado em hex para direita
      break;
      }
    recInfra.resume();    // continuar lendo
    }
void setup() {
  Serial.begin(9600);   //ver valor lido
  recInfra.enableIRIn();    //iniciar recepitor infravermelho
}

}
void loop() {
  result()
  
  //teste do infravermelho if(recInfra.decode(&sinal)){
    Serial.print("VALOR");
    Serial.println(sinal.value,HEX);
    recInfra.resume(); 
    }
}
