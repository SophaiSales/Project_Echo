#include <Wire.h>                             /*Essa biblioteca serve para eu me comunicar via i2C*/
#include <Kalman.h>                           /*Essa biblioteca serve para eu adicionar o filtro de kalman que nos ajuda a dar uma estimativa melhor dos ângulos,para termos melhor precisão*/

uint8_t i2c_data[14];                         /*criei uma variavel com buffer de 14 posições, ai temos 14 posições de uint8_t*/

double accX, accY, accZ;                      /*Essas variaveis servem para guardar os valores da MPU*/
double gyroX, gyroY, gyroZ;

#define delay_sensor 100

uint32_t timer;

Kalman KalmanX;                               /*3 Instâncias  que servem para calcular os valores dos ângulos de x,y e z*/
Kalman KalmanY;
Kalman KalmanZ;

double KalAngleX;                             /*Essas variaveis servem para receber os valores dos ângulo que foi calculado anteriormente ou seja irar receber os ângulos estimados em x,y e z*/
double KalAngleY;
double KalAngleZ;

double gyroXangle;                            /*Para termos uma estimativa de ângulos utilizando os valores de pitch e de roll,juntando os valores dos ângulos que o acelerômetro calcula para ter melhor precisão na estimativa*/
double gyroYangle;

void setup(){

 Serial.begin(115200);                          /*para verificar os valores no monitor serial e com velocidade de 9600 que é a velocidade que o arduino traalha melhor*/
 Wire.begin();

 #if ARDUINO >= 157
   Wire.setClock(400000UL);                  /* frequencia de 400KHZ*/
 #else
   TWBR = ((F_CPU/400000UL) - 16 / 2;        /*frequencia do i2 de 400kHz,fiz de novo porque dependendo da versão o wire.setClock da um erro*/
 #endif
                                              /*nessas posições da i2c_data eu carreguei com esses seguintes valores para configurar o giroscopio*/
  i2c_data[0] = 7;                            /*para essa configuração utilezei o datasheet do MPU de forma que colocando esses valores na memoria 0x19*/
  i2c_data[1] = 0x00;                         /*com esses valores configurei,fundo de escala do gyro e do acelerômetro,configurar taxa de amostragem e etc*/
  i2c_data[2] = 0x00;
  i2c_data[3] = 0x00;

  while(i2cWrite(0x19, i2c_data, 4, false));  /*com isso movi os 4 valores que colocquei no i2c a fim de configurar o giroscopio e acelerômetro  no endereço de memoria 0x19 da MPU,e adicionei uma flag*/

  while(i2cWrite(0x6B, 0x01, true));          /*com isso movi para o endereço 0x6B configigurações para que o PLL tenha com referêrencia o gyro do eixo x e desabelitei o sleep mode*/

  while(i2cRead(0x75, i2c_data, 1));          /*o while ta ai somente para garantir que essa transição de dados ocorra com sucesso,pois quando da sucesso e adicionado 0,e quando esse valor e diferente de 0 o programa trava aqui*/

  if(i2c_data[0] != 0x68){                    /*isso é para saber se estou trabalhando com a MPU6050 ou não*/
    Serial.print("Erro. Placa desconhecida\n");
    while(1){
      Serial.print("Erro. Conecte a MPU6050 no barramento i2c\n");
     }
    }

   delay(delay_sensor);                       /*esse delay serve parao sensor estabilizar os valores lidos*/

                                                
  while(i2cRead(0x3B, i2c_data, 14));         /*Leitura dos dados de Acc XYZ */

                                              /* Organizar os dados de Acc XYZ */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])      
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])

                                               
  double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;        /* Calculo de Pitch e Roll,esse calculo são os valores de ângulos que o proprio acelerômetro calcula,pitch é a inclinacão do eixo Y e roll do X */ 
  double roll  =  atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

                                              /*Inicialização do Filtro de Kalman XY */
  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);

  gyroXangle = roll;
  gyroYangle = pitch;
  
  timer = micros();
  init_inframotores();
  
}
void loop(){
  
 while(i2cRead(0X3B, i2c_data, 14));
 
  /*Aceleração*/
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])   /*Variavel que vai guardar os valores de aceleração e de giro nos 3 eixos:x,y e z*/
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])   

  /*Giroscópio*/
  gyroX = (int16_t)((i2c_data[8] << 8) | i2c_data[9]); // ([ MSB ] [ LSB ])
  gyroY = (int16_t)((i2c_data[10] << 8) | i2c_data[11]); // ([ MSB ] [ LSB ])
  gyroZ = (int16_t)((i2c_data[12] << 8) | i2c_data[13]); // ([ MSB ] [ LSB ])
  
  /*
  Serial.print("AccXYZ"); Serial.print("\t");
  Serial.print(accX); Serial.print("\n");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\n");                              /*Isso serve somente para observar graficamente que o giroscopio esta recebendo os valores conforme ele gira e acelera
  Serial.print("GiroXYZ"); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\n");
  */
  
  /********************** Filtro de Kalman *************************/
  
  /* Calculo do Delta Time */
  double dt = (double)(micros() - timer)/1000000;         /*Tempo que demorou para executarmos o loop inteiro e voltar no mesmo ponto*/
  timer = micros();
   
  double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

  /* Convertendo de Rad/Segundo para Graus/Segundo Calculo da Taxa angular baseado no Giroscópio */
  gyroXangle = gyroX / 131.0; //deg/s
  gyroYangle = gyroY / 131.0; //deg/s

  /* Estimativa de Ângulo nos Eixos X e Y usando Filtro de Kalman */
  KalAngleX = KalmanX.getAngle(roll, gyroXangle, dt);
  KalAngleY = KalmanY.getAngle(pitch, gyroYangle, dt);

  Serial.print(KalAngleY); Serial.print("\n");
  Serial.print(pitch); Serial.print("\t");
  double  res = Compute(KalAngleY);
  derecMotores (-100);
  
  /*Feito todo o processo de estimativa de valores com o filtro de kalman é esperado que tenha melhor linearidade nos valores de inclinaçao e de giro,para que isso facilite na calibração para o equlibrio do Echo*/
}
//==============================================================================
const uint8_t IMUAddress = 0x68; 
const uint16_t I2C_TIMEOUT = 1000; 

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {                   /*quando é chamado a função i2cWrite com 3 argumentos somente essa parte vai ser chamada*/
  return i2cWrite(registerAddress, &data, 1, sendStop); // Retorna 0 com sucesso
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {  /*quando é passado 4 argumenos é chamado essa parte*/
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); 
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; 
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; 
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; 
      }
    }
  }
  return 0; // FOI POHAA
}
