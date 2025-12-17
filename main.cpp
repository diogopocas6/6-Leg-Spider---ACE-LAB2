#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Endereço padrão do PCA9685
#define PCA9685_ADDR 0x40

// Frequência para servos
#define SERVO_FREQ 50  // 50 Hz

// Limites típicos de servo (ajusta depois!)
#define SERVO_MIN  150  // pulso mínimo
#define SERVO_MAX  600  // pulso máximo
int pos;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Forward declaration
void moveServo(uint8_t channel, uint8_t angle);

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
  moveServo(0,90);
  moveServo(1,90);
  moveServo(2,90);
  moveServo(3,90);
  moveServo(4,90);
  moveServo(5,90);
  moveServo(6,90);
  moveServo(7,90);
  moveServo(8,90);
  moveServo(9,90);
  moveServo(10,90);
  moveServo(11,90);
  moveServo(12,90);
  delay(1000);
}

void loop() {
  for (pos=90; pos<=120; pos++){
    moveServo(7, pos);
    moveServo(12, pos);
    moveServo(0,pos);
    moveServo(1,pos);
    delay(5);
  }
  delay(2000);
  
  for (pos=120; pos>=90; pos--){
    moveServo(7, pos);
    moveServo(12, pos);
    moveServo(0,pos);
    moveServo(1,pos);
    delay(5);
  }
  delay(2000);
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
