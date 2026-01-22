  #include <WiFi.h>
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

  const uint8_t ELBOW_CH[6] = {0, 2, 4, 8, 10, 12};
  const uint8_t SHOULDER_CH[6]= {1, 3, 5, 9, 11, 13};
  const int ELBOW_BASE[6] = { zerope, doispe, quatrope, oitope, dezpe, dozepe };
  const int SHOULDER_BASE[6] = { umpe, trespe, cincope, novepe, onzepe, trezepe };



  //Configurações de Wifi
  const char* ssid = "Wifi_aranha";
  const char* password = "Senha123";
  WiFiServer server(1234);   // 1234 é o número da porta
  WiFiClient client;



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
    Aranha_velocidade_andar,      // estado para escolher velocidade da rotina Walk()
    Aranha_a_virar
  };

  // Forward declaration
  void moveServo(uint8_t channel, uint8_t angle);
  void Levanta_Aranha(int velocidade);
  void Deita_Aranha(int velocidade);
  void WalkStep(int velocidade);
  void fsm_update();
  void set_state(fsm_t &fsm, int new_state);
  void ResetToNeutral();
  void Turn();
  

  // Bloco para utilizarmos o Serial para transição de estados
  char cmd = 0;
  int speed_l = 1;
  int speed_d = 1;
  int speed_w = 1;
  void read_serial() {
    if (Serial.available()) {
      cmd = Serial.read();
    }
  }

  //Função para utilizar comunicação wifi
void read_wifi() {
  if (!client || !client.connected()) {
    client = server.available();
    return;
  }

  if (client.available()) {
    char c = client.read();

    if (c == '\n' || c == '\r') return;

    cmd = c;

    Serial.print("CMD (WiFi): ");
    Serial.println(cmd);

    client.print("CMD: ");
    client.println(cmd);
  }
}



void logMsg(const char* msg) {
  Serial.println(msg);          // debug 
  if (client && client.connected()) {
    client.println(msg);        // Wi-Fi
  }
}

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

  Serial.println("Ligando ao WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi ligado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Servidor TCP iniciado na porta 1234");

  delay(3000);
  }

  void loop() {
    read_wifi();
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
  void Turn(){//Função que faz o robô virar
    int delay_servos, incremento;
    delay_servos = 30; incremento = 2;
    const int AMP_ELBOW = 30;
    const int AMP_SHOULDER = 25;
    static int fase = 0;
    static int stepPos = 0;  
    static unsigned long lastTime = 0;
    if (cmd == 's') { fase = 0; stepPos = 0; ResetToNeutral(); set_state(fsm, Aranha_a_andar); return; }
    if (millis() - lastTime < (unsigned long)delay_servos) return;
    lastTime = millis();
  switch(fase)
    {
        case 0:{//levantar A
        moveServo(0,zerope +stepPos);
        moveServo(4,quatrope +stepPos);
        moveServo(10,dezpe +stepPos);
        stepPos += incremento;
        if(stepPos>=AMP_ELBOW)
        {
        stepPos = 0;
        fase++;
        }
        break;
        }
        case 1:{//Turn A 
            moveServo(1,umpe-stepPos);
            moveServo(5,cincope-stepPos);
            moveServo(11,onzepe-stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_SHOULDER)
            {
                stepPos = 0;
                fase++;
                break;
            }
            break;
        }
        case 2:{//Baixar A
            moveServo(0,zerope +AMP_ELBOW -stepPos);
            moveServo(4,quatrope +AMP_ELBOW -stepPos);
            moveServo(10,dezpe +AMP_ELBOW -stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_ELBOW)
            {
                stepPos = 0;
                fase++;
                break;
            }
            break;
        }
        case 3:{//levantar B
            moveServo(2,doispe +stepPos);
            moveServo(8,oitope +stepPos);
            moveServo(12,dozepe +stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_ELBOW)
            {
            stepPos = 0;
            fase++;
            break;
            }
            break;
        }
        case 4:{//A back to the normal position
            moveServo(1,umpe-AMP_SHOULDER +stepPos);
            moveServo(5,cincope-AMP_SHOULDER +stepPos);
            moveServo(11,onzepe-AMP_SHOULDER +stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_SHOULDER)
            {
                stepPos = 0;
                fase++;
                break;
            }
            break;
        }
        case 5:{//Andar B a frente
            moveServo(3,trespe+stepPos);
            moveServo(9,novepe+stepPos);
            moveServo(13,trezepe+stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_SHOULDER)
            {
                stepPos = 0;
                fase++;
                break;
            }
            break;
        }
        case 6:{//Baixar B
            moveServo(2,doispe +AMP_ELBOW -stepPos);
            moveServo(8,oitope +AMP_ELBOW -stepPos);
            moveServo(12,dozepe +AMP_ELBOW -stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_ELBOW)
            {
                stepPos = 0;
                fase++;
                break;
            }
            break;
        }
        case 7:{//levantar A
            moveServo(0,zerope +stepPos);
            moveServo(4,quatrope +stepPos);
            moveServo(10,dezpe +stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_ELBOW)
            {
                stepPos = 0;
                fase++;
                break;
            }
            break;
        }
        case 8:{//Andar B pra trás
            moveServo(3,trespe+AMP_SHOULDER -stepPos);
            moveServo(9,novepe+AMP_SHOULDER -stepPos);
            moveServo(13,trezepe+AMP_SHOULDER -stepPos);
            stepPos += incremento;
            if(stepPos>=AMP_SHOULDER)
            {
                stepPos = 0;
                fase=1;
                break;
            }
            break;
        }
    }
}
  //Função que faz robô andar
void WalkStep(int velocidade) {
  int delay_servos, incremento;
  const int AMP_ELBOW = 30;
  const int AMP_SHOULDER = 25;
  static int fase = 0;
  static int stepPos = 0;  
  static unsigned long lastTime = 0;
  if (velocidade == 1) { delay_servos = 50; incremento = 1; }
  else if (velocidade ==2)  { delay_servos = 30; incremento = 2; }
  else if (velocidade == 3) { delay_servos = 20; incremento = 5; }
  if (cmd == 's') { fase = 0; stepPos = 0; ResetToNeutral(); set_state(fsm, Aranha_em_pe); return; }
  if (cmd == 't') { fase = 0; stepPos = 0; ResetToNeutral(); set_state(fsm,Aranha_a_virar); return; }
  if (millis() - lastTime < (unsigned long)delay_servos) return;
  lastTime = millis();
  switch(fase){
    case 0:{//Levantar A
    moveServo(0,zerope +stepPos);
    moveServo(4,quatrope +stepPos);
    moveServo(10,dezpe +stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_ELBOW)
    {
      stepPos = 0;
      fase++;
    }
    break;
  }
  case 1:{//Andar A a frente
    moveServo(1,umpe-stepPos);
    moveServo(5,cincope-stepPos);
    moveServo(11,onzepe+stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_SHOULDER)
    {
      stepPos = 0;
      fase++;
      break;
    }
    break;
  }
  case 2:{//Baixar A
    moveServo(0,zerope +AMP_ELBOW -stepPos);
    moveServo(4,quatrope +AMP_ELBOW -stepPos);
    moveServo(10,dezpe +AMP_ELBOW -stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_ELBOW)
    {
      stepPos = 0;
      fase++;
      break;
    }
    break;
  }
  case 3:{//Levantar B
    moveServo(2,doispe +stepPos);
    moveServo(8,oitope +stepPos);
    moveServo(12,dozepe +stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_ELBOW)
    {
      stepPos = 0;
      fase++;
      break;
    }
    break;
  }
  case 4:{//Andar A pra trás
    moveServo(1,umpe-AMP_SHOULDER +stepPos);
    moveServo(5,cincope-AMP_SHOULDER +stepPos);
    moveServo(11,onzepe+AMP_SHOULDER -stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_SHOULDER)
    {
      stepPos = 0;
      fase++;
      break;
    }
    break;
  }
  case 5:{//Andar B a frente
    moveServo(3,trespe-stepPos);
    moveServo(9,novepe+stepPos);
    moveServo(13,trezepe+stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_SHOULDER)
    {
      stepPos = 0;
      fase++;
      break;
    }
    break;
  }
  case 6:{//Baixar B
    moveServo(2,doispe +AMP_ELBOW -stepPos);
    moveServo(8,oitope +AMP_ELBOW -stepPos);
    moveServo(12,dozepe +AMP_ELBOW -stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_ELBOW)
    {
      stepPos = 0;
      fase++;
      break;
    }
    break;
  }
  case 7:{//levantar A
    moveServo(0,zerope +stepPos);
    moveServo(4,quatrope +stepPos);
    moveServo(10,dezpe +stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_ELBOW)
    {
      stepPos = 0;
      fase++;
      break;
    }
    break;
  }
  case 8:{//Andar B pra trás
    moveServo(3,trespe-AMP_SHOULDER +stepPos);
    moveServo(9,novepe+AMP_SHOULDER -stepPos);
    moveServo(13,trezepe+AMP_SHOULDER -stepPos);
    stepPos += incremento;
    if(stepPos>=AMP_SHOULDER)
    {
      stepPos = 0;
      fase=1;
      break;
    }
    break;
  }
}
}
// Add this helper to reset all to base on stop
void ResetToNeutral() {
  for (int i = 0; i < 6; i++) {
    moveServo(ELBOW_CH[i], ELBOW_BASE[i]);
    moveServo(SHOULDER_CH[i], SHOULDER_BASE[i]);
  }
  delay(500); // Settle
}
  // Função que chama set_state e cumpre a lógica da máquina de estaddos com transições a utilizar o serial monitor
  void fsm_update() {
    fsm.tis = millis() - fsm.tes;

    switch (fsm.state) {

      case Aranha_deitada:
        if (cmd == 'l') {     // l no serial monitor faz com que a aranha levante
          set_state(fsm,Aranha_velocidade_levantar);
          logMsg("Escolha a velocidade para levantar entre 1 e 3.");
          cmd = 0;
        }
        break;

      case Aranha_velocidade_levantar:
        if (cmd != 0) {
          if (cmd == '1' || cmd == '2' || cmd == '3') {
            speed_l = cmd - '0';    // se fizessemos simplesmente (int)cmd , o valor não seria 1, por causa da conversão hexadecimal!
            set_state(fsm,Aranha_a_levantar);
            cmd=0;
          }
          else {
            logMsg("Velocidade inválida!Escolha uma velocidade entre 1 e 3.");
            cmd=0;
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
            cmd=0;
          }
          else {
            logMsg("Velocidade inválida!Escolha uma velocidade entre 1 e 3.");
            cmd=0;
          }
        }
        break;

      case Aranha_velocidade_andar:
        if (cmd != 0) {
          if (cmd == '1' || cmd == '2' || cmd == '3') {
            speed_w = cmd - '0';    // se fizessemos simplesmente (int)cmd , o valor não seria 1, por causa da conversão hexadecimal!
            set_state(fsm,Aranha_a_andar);
            logMsg("Para parar o robô, utilize a tecla 's'.");
            cmd=0;
          }
          else {
            logMsg("Velocidade inválida!Escolha uma velocidade entre 1 e 3.");
            cmd=0;
          }
        }
        break;

      case Aranha_em_pe:
        if (cmd == 'd') {     // cmd = 'd' faz com que a aranha deite
          set_state(fsm, Aranha_velocidade_deitar);
          logMsg("FSM: em_pe -> velocidade_deitar");
          logMsg("Escolha a velocidade para deitar entre 1 e 3.");
          cmd=0;
        }
        if (cmd == 'w') {     // w no serial monitor faz com que a aranha ande
          set_state(fsm,Aranha_velocidade_andar);
          logMsg("Escolha a velocidade para andar entre 1 e 3.");
          cmd=0;
        }
        break;
      
      case Aranha_a_deitar:
        Deita_Aranha(speed_d);
        speed_d=1;
        set_state(fsm, Aranha_deitada);
        break;

      case Aranha_a_andar:
        WalkStep(speed_w);
        break;
      case Aranha_a_virar:
        Turn();
        break;
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

