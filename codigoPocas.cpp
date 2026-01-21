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

#define zerope 40
#define umpe 120
#define doispe 35
#define trespe 90
#define quatrope 65
#define cincope 65
#define oitope 70
#define novepe 115
#define dezpe 70
#define onzepe 80
#define dozepe 50
#define trezepe 90

int pos;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

//Classe máquina de estados vinda do código do display do trabalho 1
typedef struct {
  int state;
  unsigned long tes;   // time entering state
  unsigned long tis;   // time in state
} fsm_t;

fsm_t fsm;

//Estados fsm
enum {
  Aranha_deitada = 0,           // estado inicial
  Aranha_a_levantar,            // executa Levanta_Aranha()
  Aranha_em_pe,                 // estado estável
  Aranha_a_deitar,              // executa Deita_Aranha()
  Aranha_a_andar,               // Executa Walk()
  Aranha_velocidade_levantar,   // estado para escolher velocidade da rotina Levanta_Aranha()
  Aranha_velocidade_deitar,      // estado para escolher velocidade da rotina Deita_Aranha()
  Aranha_velocidade_andar       // estado para escolher velocidade da rotina Walk()
};

// Forward declaration
void moveServo(uint8_t channel, uint8_t angle);
void Levanta_Aranha(int velocidade);
void Deita_Aranha(int velocidade);
void Walk(int velocidade);
void fsm_update();
void set_state(fsm_t &fsm, int new_state);

// Bloco para utilizarmos o Serial para transição de estados
char cmd = 0;
int speed_l = 1;
int speed_d = 1;
int speed_w = 1;
void read_serial() {
  if (Serial.available()) {
    cmd = Serial.read();
  } else {
    cmd = 0;
  }
}
//

void setup() {
  Serial.begin(115200);
  delay(1000);
  set_state(fsm, Aranha_em_pe);

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

  read_serial();
  fsm_update();
}





// Função para mover servo por ângulo (0–180)
void moveServo(uint8_t channel, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);

  /*
  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println(" graus");
  */
}

void Levanta_Aranha(int velocidade){
  int delay_number, incremento;
  if (velocidade == 1){
    delay_number=50;
    incremento = 1;
  }
  if (velocidade == 2){
    delay_number = 25;
    incremento = 2;
  }
  if (velocidade == 3){
    delay_number = 25;
    incremento = 5;
  }

  for (pos=0; pos<=50; pos=pos+incremento){
    moveServo(0,zerope+50-pos);
    moveServo(2,doispe+50-pos);
    moveServo(4,quatrope+50-pos);
    moveServo(8,oitope+50-pos);
    moveServo(10,dezpe+50-pos);
    moveServo(12,dozepe+50-pos);
    delay(delay_number);
  }
}

void Deita_Aranha(int velocidade){
  int delay_number, incremento;
  if (velocidade == 1){
    delay_number=50;
    incremento = 1;
  }
  if (velocidade == 2){
    delay_number = 25;
    incremento = 2;
  }
  if (velocidade == 3){
    delay_number = 25;
    incremento = 5;
  }
  for (pos=0; pos<=50; pos=pos+incremento){
    moveServo(0,zerope+pos);
    moveServo(2,doispe+pos);
    moveServo(4,quatrope+pos);
    moveServo(8,oitope+pos);
    moveServo(10,dezpe+pos);
    moveServo(12,dozepe+pos);
    delay(delay_number);
  }

}
//Função que faz robô andar
void Walk(int velocidade){
int intervalo, delay_servos,delay_passos,incremento,pos_interval;
if (velocidade == 1){
    delay_servos = 15;
    incremento = 1;
    pos_interval = 30;
    delay_passos = 200;
  }
  if (velocidade == 2){
    delay_servos = 15;
    incremento = 1;
    pos_interval = 30;
    delay_passos = 200;
  }
  if (velocidade == 3){
    delay_servos = 15;
    incremento = 1;
    pos_interval = 30;
    delay_passos = 200;
  }

  //Serial.println("A - Passo 1");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){    //sobem A
    moveServo(0,zerope+pos);
    moveServo(4,quatrope+pos);
    moveServo(10,dezpe+pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("A - Passo 2");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){    //mexem A1
    moveServo(1,umpe-pos);
    moveServo(5,cincope-pos);
    moveServo(11,onzepe+pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("A - Passo 3");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//descem A
    moveServo(0,zerope+pos_interval-pos);
    moveServo(4,quatrope+pos_interval-pos);
    moveServo(10,dezpe+pos_interval-pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("A - Passo 4 ");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//sobem B
    moveServo(2,doispe+pos);
    moveServo(8,oitope+pos);
    moveServo(12,dozepe+pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("A - Passo 5");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//andam A1
    moveServo(1,umpe-pos_interval+pos);
    moveServo(5,cincope-pos_interval+pos);
    moveServo(11,onzepe+pos_interval-pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("A - Passo 6");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//descem B
    moveServo(2,doispe+pos_interval-pos);
    moveServo(8,oitope+pos_interval-pos);
    moveServo(12,dozepe+pos_interval-pos);
    delay(delay_servos);
  } 

  delay(delay_passos); 
  
  //------------------------------------------------//
  // SEGUNDA PARTE DO MOVIMENTO PARA SER DOIS PASSOS//
  //------------------------------------------------//
  //Serial.println("B - Passo 1");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//sobem B
    moveServo(2,doispe+pos);
    moveServo(8,oitope+pos);
    moveServo(12,dozepe+pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("B - Passo 2");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//mexem B1
    moveServo(3,trespe-pos);
    moveServo(9,novepe-pos);
    moveServo(13,trezepe+pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("B - Passo 3");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//descem B
    moveServo(2,doispe+pos_interval-pos);
    moveServo(8,oitope+pos_interval-pos);
    moveServo(12,dozepe+pos_interval-pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("B - Passo 4");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//sobem A
    moveServo(0,zerope+pos);
    moveServo(4,quatrope+pos);
    moveServo(10,dezpe+pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("B - Passo 5");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//andam B1
    moveServo(3,trespe-pos_interval+pos);
    moveServo(9,novepe-pos_interval+pos);
    moveServo(13,trezepe+pos_interval-pos);
    delay(delay_servos);
  }
  delay(delay_passos);

  //Serial.println("B - Passo 6");
  for (pos=0; pos<=pos_interval; pos=pos+incremento){//descem A
    moveServo(0,zerope+pos_interval-pos);
    moveServo(4,quatrope+pos_interval-pos);
    moveServo(10,dezpe+pos_interval-pos);
    delay(delay_servos);
  } 
  delay(delay_passos);

}

// Função que chama set_state e cumpre a lógica da máquina de estaddos com transições a utilizar o serial monitor
void fsm_update() {
  fsm.tis = millis() - fsm.tes;

  switch (fsm.state) {

    case Aranha_deitada:
      if (cmd == 'l') {     // l no serial monitor faz com que a aranha levante
        set_state(fsm,Aranha_velocidade_levantar);
        Serial.println("Escolha a velocidade para levantar entre 1 e 3.");
        //set_state(fsm, Aranha_a_levantar);
      }
      break;

    case Aranha_velocidade_levantar:
      if (cmd != 0) {
        if (cmd == '1' || cmd == '2' || cmd == '3') {
          speed_l = cmd - '0';    // se fizessemos simplesmente (int)cmd , o valor não seria 1, por causa da conversão hexadecimal!
          set_state(fsm,Aranha_a_levantar);
        }
        else {
          Serial.println("Velocidade inválida!Escolha uma velocidade entre 1 e 3.");
        }
      }
      break;

    case Aranha_a_levantar:
      Levanta_Aranha(speed_l);
      speed_l=1;
      set_state(fsm, Aranha_em_pe);
      break;

    case Aranha_velocidade_deitar:
      if (cmd != 0) {
        if (cmd == '1' || cmd == '2' || cmd == '3') {
          speed_d = cmd - '0';    // se fizessemos simplesmente (int)cmd , o valor não seria 1, por causa da conversão hexadecimal!
          set_state(fsm,Aranha_a_deitar);
        }
        else {
          Serial.println("Velocidade inválida!Escolha uma velocidade entre 1 e 3.");
        }
      }
      break;

    case Aranha_velocidade_andar:
      if (cmd != 0) {
        if (cmd == '1' || cmd == '2' || cmd == '3') {
          speed_d = cmd - '0';    // se fizessemos simplesmente (int)cmd , o valor não seria 1, por causa da conversão hexadecimal!
          set_state(fsm,Aranha_a_andar);
          Serial.println("Para parar o robô, utilize a tecla 's'.");
        }
        else {
          Serial.println("Velocidade inválida!Escolha uma velocidade entre 1 e 3.");
        }
      }
      break;

    case Aranha_em_pe:
      if (cmd == 'd') {     // d no serial monitor faz com que a aranha deite
        set_state(fsm, Aranha_velocidade_deitar);
        Serial.println("Escolha a velocidade para levantar entre 1 e 3.");
      }
      if (cmd == 'w') {     // w no serial monitor faz com que a aranha ande
        set_state(fsm,Aranha_velocidade_andar);
        Serial.println("Escolha a velocidade para andar entre 1 e 3.");
      }
      break;
    
    case Aranha_a_deitar:
      Deita_Aranha(speed_d);
      speed_d=1;
      set_state(fsm, Aranha_deitada);
      break;

    case Aranha_a_andar:
    if (cmd == 's') {
      set_state(fsm,Aranha_em_pe);
      Serial.println("Aranha em pé. Comandos disponíveis:");
      Serial.println("Andar - 'w'");
      Serial.println("Deitar - 'd'");
    }
    else {
      Walk(speed_w);
      break;
    }
  }
}

//função para transição de estados
void set_state(fsm_t &fsm, int new_state) {
  if (fsm.state != new_state) {
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}
