#include <Wire.h> /*Essa bilioteca serve para eu me comunicar via i2C*/
#include <Kalman.h>

uint8_t i2c_data[14];                         /*criei uma variavel com buffer de 14 posições, ai temos 14 posições de uint8_t*/

double accX, accY, accZ;                      /*Essas variaveis servem para guardar os valores da MPU*/
double gyroX, gyroY, gyroZ;

#define delay_sensor 100

uint32_t timer;

Kalman KalmanX;
Kalman KalmanY;
Kalman KalmanZ;

double KalAngleX;
double KalAngleY;
double KalAngleZ;

double gyroXangle;
double gyroYangle;

void setup(){

 Serial.begin(9600);                          /*para verificar os valores no monitor serial e com velocidade de 9600 que é a velocidade que o arduino traalha melhor*/
 Wire.begin();

 #if ARDUINO >= 157
   Wire.setClock(4000000UL);                  /* frequencia de 400KHZ*/
 #else
   TWBR = ((F_CPU/4000000UL) - 16 / 2;        /*frequencia do i2 de 400kHz,fiz de novo porque dependendo da versão o wire.setClock da um erro*/
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

   delay(delay_sensor);                        /*esse delay serve parao sensor estabilizar os valores lidos*/

    /* 1 - Leitura dos dados de Acc XYZ */
  while(i2cRead(0x3B, i2c_data, 14));

  /* 2 - Organizar os dados de Acc XYZ */
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])
  accY = (int16_t)((i2c_data[2] << 8) | i2c_data[3]); // ([ MSB ] [ LSB ])
  accZ = (int16_t)((i2c_data[4] << 8) | i2c_data[5]); // ([ MSB ] [ LSB ])

  /* 3 - Calculo de Pitch e Roll */  
  double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll  =  atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

  /* 4 - Inicialização do Filtro de Kalman XY */
  KalmanX.setAngle(roll);
  KalmanY.setAngle(pitch);

  gyroXangle = roll;
  gyroYangle = pitch;
  
  timer = micros();

}
void loop(){
  
 while(i2cRead(0X3B, i2c_data, 14));
  /*Aceleração*/
  accX = (int16_t)((i2c_data[0] << 8) | i2c_data[1]); // ([ MSB ] [ LSB ])
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
  Serial.print(accZ); Serial.print("\n");
  Serial.print("GiroXYZ"); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\n");
  */
  
  /********************** Filtro de Kalman *************************/
  
  /* Calculo do Delta Time */
  double dt = (double)(micros() - timer)/1000000;
  timer = micros();

  double pitch = atan(accX/sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double roll = atan(accY/sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

  /* Convertendo de Rad/Segundo para Graus/Segundo Calculo da Taxa angular baseado no Giroscópio */
  gyroXangle = gyroX / 131.0; //deg/s
  gyroYangle = gyroY / 131.0; //deg/s

  /* Estimativa de Ângulo nos Eixos X e Y usando Filtro de Kalman */
  KalAngleX = KalmanX.getAngle(roll, gyroXangle, dt);
  KalAngleY = KalmanY.getAngle(pitch, gyroYangle, dt);

  /* Mensagens de Debug para verificação dos resultados obtidos com Filtro de Kalman e Calculos dos Angulos com os Acelerômetros */
  Serial.print(KalAngleY); Serial.print("\n"); //Angulo estimado com o filtro de Kalman
  Serial.print(pitch); Serial.print("\t"); //Angulo Calculado com os dados de aceleração da MPU6050
}
//==============================================================================
const uint8_t IMUAddress = 0x68; 
const uint16_t I2C_TIMEOUT = 1000; 

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
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
