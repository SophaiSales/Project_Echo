//Motor1
#define velMotor1 3    //pino de velocidade da ponteH do motor1 
#define ent1Motor1 4   //pino1 de entrada da ponteH do motor1
#define ent2Motor1 5   //pino2 de entrada da ponteH do motor1 
//Motor2
#define velMotor2 6    //pino de velocidade da ponteH do motor2 
#define ent1Motor2 7   //pino1 de entrada da ponteH do motor2
#define ent2Motor2 8   //pino2 de entrada da ponteH do motor2 
//#define tmp1(3000)
//#define tmp2(200)    //tempo de parar pra virar  

int velo=255;

void frente(){
  analogWrite(velMotor1,velo); //definiçao da velocidade
  digitalWrite(ent1Motor1,LOW); //definiçao para virar par 
  digitalWrite(ent2Motor1,HIGH);
  //delay(tmp1); //definiçao de tempo

  analogWrite(velMotor2,velo); //definiçao da velocidade
  digitalWrite(ent1Motor2,LOW); //definiçao para virar par 
  digitalWrite(ent2Motor2,HIGH);
  //delay(tmp1); //definiçao de tempo
  }
void pratras(){
  analogWrite(velMotor2,velo); //definiçao da velocidade
  digitalWrite(ent1Motor1,HIGH); //definiçao para virar pra 
  digitalWrite(ent2Motor1,LOW);
  //delay(tmp1);
  
  analogWrite(velMotor2,velo); //definiçao da velocidade  
  digitalWrite(ent1Motor2,HIGH); //definiçao para virar pra 
  digitalWrite(ent2Motor2,LOW);
  //delay(tmp1);
  }
void parar(){
  digitalWrite(ent1Motor1,LOW); //definiçao par parar 
  digitalWrite(ent2Motor1,LOW);
  //delay(tmp2);

  digitalWrite(ent1Motor2,LOW); //definiçao par parar 
  digitalWrite(ent2Motor2,LOW);
  //delay(tmp2);
  
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
}

void loop() {
  frente()
  pratras()
  parar()
}
