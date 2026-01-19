#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Endereço padrão do PCA9685
#define PCA9685_ADDR 0x40

// Frequência para servos
#define SERVO_FREQ 50  // 50 Hz

// Limites típicos de servo (ajusta depois!)
#define SERVO_MIN  150  // pulso mínimo
#define SERVO_MAX  600  // pulso máximo
#define posmin 40
#define posmax 140

#define zerope 45
#define umpe 120
#define doispe 55
#define trespe 90
#define quatrope 65
#define cincope 65
#define oitope 25
#define novepe 115
#define dezpe 50
#define onzepe 80
#define dozepe 60
#define trezepe 90

int pos;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Forward declaration
void moveServo(uint8_t channel, uint8_t angle);
void Levanta_Aranha();
void Deita_Aranha();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inicializando PCA9685...");

  // Iniciar I2C
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();  // GP4 = SDA, GP5 = SCL

  // Iniciar PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("PCA9685 pronto!");
  Serial.println("Movendo servo para posição central...");

  // Teste: mover servo do canal 0 para posição central
  pos=0;
  moveServo(0,zerope);     // CALIBRADA EM PÉ -> 45      Levantada para mover -> 90
  moveServo(1,umpe);       // CALIBRADA EM PÉ -> 120     
  moveServo(2,doispe);        // CALIBRADA EM PÉ -> 55      Levantada para mover -> 100
  moveServo(3,trespe);        // CALIBRADA EM PÉ -> 90      
  moveServo(4,quatrope);        // CALIBRADA EM PÉ -> 65      Levantada para mover -> 110
  moveServo(5,cincope);        // CALIBRADA EM PÉ -> 65      
  //moveServo(6,90);
  //moveServo(7,90);
  moveServo(8,oitope);        // CALIBRADA EM PÉ -> 25      Levantada para mover -> 70
  moveServo(9,novepe);       // CALIBRADA EM PÉ -> 115     
  moveServo(10,dezpe);       // CALIBRADA EM PÉ -> 50      Levantada para mover -> 95
  moveServo(11,onzepe);       // CALIBRADA EM PÉ -> 80      
  moveServo(12,dozepe);        // CALIBRADA EM PÉ -> 60      Levantada para mover -> 75
  moveServo(13,trezepe);       // CALIBRADA EM PÉ -> 90      
  //moveServo(14,90); 
  moveServo(15,90);
  delay(3000);
}

void loop() {
/* 
  
  // Passo para a frente
  //Levanta primeiro as patas 0 4 e 10 amplitude 45º
  for (pos=0;pos<45;pos++){
    moveServo(0,zerope+pos);
    moveServo(4,quatrope+pos);
    moveServo(10,dezpe+pos);
    delay(5);
  }
  delay(3000);

  /*moveServo(0,zerope);
  moveServo(4,quatrope);
  moveServo(10,dezpe);
  delay(5000);
  
  //Arrasta as patas do chao para a frente (servos de cima), mover 3 9 13 para a frente/trás amplitude 30º
  for (pos=0;pos<30;pos++){
    moveServo(3,90+pos);
    moveServo(9,115-pos);
    moveServo(13,90-pos);
    delay(5);
  }
  //coloca patas do ar no chao () logo a seguir a arrastar as patas
  for (pos=0;pos<45;pos++){
    moveServo(0,zerope+45-pos);
    moveServo(4,quatrope+45-pos);
    moveServo(10,dezpe+45-pos);
    delay(1);
  }
  delay(5000);

  // posiçao inicial
  moveServo(3,trespe);
  moveServo(9,novepe);
  moveServo(13,trezepe);
  moveServo(0,zerope);
  moveServo(4,quatrope);
  moveServo(10,dezpe);
  delay(5000);
    */


   /* ROTINA DEITA E LEVANTA
  Deita_Aranha();
  delay(2000);
  
  Levanta_Aranha();
  delay(2000);
           */

}





// Função para mover servo por ângulo (0–180)
void moveServo(uint8_t channel, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);

  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println(" graus");
}

void Levanta_Aranha(){
  for (pos=0; pos<=50; pos=pos+2){
    moveServo(0,zerope+50-pos);
    moveServo(2,doispe+50-pos);
    moveServo(4,quatrope+50-pos);
    moveServo(8,oitope+50-pos);
    moveServo(10,dezpe+50-pos);
    moveServo(12,dozepe+50-pos);
    delay(50);
  }
}

void Deita_Aranha(){
  for (pos=0; pos<=50; pos=pos+2){
    moveServo(0,zerope+pos);
    moveServo(2,doispe+pos);
    moveServo(4,quatrope+pos);
    moveServo(8,oitope+pos);
    moveServo(10,dezpe+pos);
    moveServo(12,dozepe+pos);
    delay(50);
  }
}
