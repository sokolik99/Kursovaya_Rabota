#include <Ultrasonic.h>
Ultrasonic sonarL(37, 39);
Ultrasonic sonarC(41, 43);
Ultrasonic sonarR(40, 42);
Ultrasonic sonarSideL(31, 33);
Ultrasonic sonarSideR(50, 52);
Ultrasonic sonarB(30, 32);

#include <AFMotor.h>
AF_DCMotor m_FL(4, MOTOR34_64KHZ);
AF_DCMotor m_FR(3, MOTOR34_64KHZ);
AF_DCMotor m_BL(1, MOTOR12_64KHZ);
AF_DCMotor m_BR(2, MOTOR12_64KHZ);

enum Command {
  NONE_CMD = 0,
  STOP_CMD = 1,
  BACKWARD_CMD = 2,
  FORWARD_CMD = 3,
  LEFT_TURN_CMD = 4,
  RIGHT_TURN_CMD = 5,
  AUTOPILOT_CMD = 6
};

Command currentCommand; //команда, которая выполняется в данный момент
bool isAutopilot = false; //текущий режим: автопилот/ручное управление

float distL, distC, distR, distSideR, distSideL, distB;
int sFL, sFR, sBL, sBR, pulsR, pulsL, direction = 0;
int encoder1_pin = 2; // правый энкодер
int encoder2_pin = 3; // левый энкодер

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(115200);
  currentCommand = STOP_CMD;
  pinMode(encoder1_pin, INPUT);
  pinMode(encoder2_pin, INPUT);
  attachInterrupt(0, counter, FALLING);
  pulsR = 0;
  pulsL = 0;
}

void ultrazvuk() { // считывание расстояния датчиками ультразвука
  distL = sonarL.Ranging(CM);
  distR = sonarR.Ranging(CM);
  distC = sonarC.Ranging(CM);
  distSideL = sonarSideL.Ranging(CM);
  distSideR = sonarSideR.Ranging(CM);
  distB = sonarB.Ranging(CM);
}

void counter() {
  pulsR++;
  pulsL++;
}

void dvig (int sFL, int sFR, int sBL, int sBR) { // установка ориентации направления моторов
  if (sFL > 0) {
    m_FL.run(FORWARD);
  }
  else {
    m_FL.run(BACKWARD);
    sFL = -sFL;
  }
  if (sFR > 0) {
    m_FR.run(BACKWARD);
  }
  else {
    m_FR.run(FORWARD);
    sFR = -sFR;
  }
  if (sBL > 0) {
    m_BL.run(BACKWARD);
  }
  else {
    m_BL.run(FORWARD);
    sBL = -sBL;
  }
  if (sBR > 0) {
    m_BR.run(FORWARD);
  }
  else {
    m_BR.run(BACKWARD);
    sBR = -sBR;
  }
  else if (sFL < 0 && sFR > 0 && sBL < 0 && sBR > 0) { // влево
    direction -= 1;
  }
  else if (sFL > 0 && sFR < 0 && sBL > 0 && sBR < 0) { // вправо
    direction += 1;
  }
  m_FL.setSpeed(sFL);
  m_FR.setSpeed(sFR);
  m_BL.setSpeed(sBL);
  m_BR.setSpeed(sBR);
}

void vdolSR (int D) { // едем на расстоянии 6 см от стены
  int x = 35;
  int Kr = (distSideR - D) * x;
  sBL -= Kr;
  sFL -= Kr;
  sFL += Kr;
  sBR += Kr;
}
void vdolSL (int D) { // едем на расстоянии 6 см от стены
  int x = 35;
  int Kl = (distSideL - D) * x;
  sBR -= Kl;
  sFR -= Kl;
  sFL += Kl;
  sBL += Kl;
}

void gogo(int FL, int FR, int BL, int BR, int ticks) { // робот едет указанное количество меток (1 метка = 1 см)

  dvig(FL, FR, BL, BR);
  int x = pulsR + ticks;
  int y = pulsL + ticks;
  while (pulsR <= x || pulsL <= y) {
  }
  dvig(0, 0, 0, 0);
}

void turnLEFT() { // поворот на 90 гр влево
  sFR = 250;
  sFL = -250;
  sBR = 250;
  sBL = -250;
  dvig(sFL, sFR, sBL, sBR);
  int x = pulsR + 55;
  while (pulsR <= x) {
  }
  dvig(0, 0, 0, 0);
}
void turnRIGHT() { // поворот на 90 гр вправо
  sFR = -250;
  sFL = 250;
  sBR = -250;
  sBL = 250;
  dvig(sFL, sFR, sBL, sBR);
  int x = pulsR + 55;
  while (pulsR <= x) {
  }
  dvig(0, 0, 0, 0);
}

void m_speed () { // основная программа движения с нечеткой логикой

  int sFL, sFR, sBL, sBR;
  float nL, nR, nC, nSR, nSL;

  if (distC <= 15 || distL <= 8 || distR <= 8) { // объезд препятствия и движение вдоль преграды
    int nL = sonarL.Ranging(CM);
    int nR = sonarR.Ranging(CM);
    int nC = sonarC.Ranging(CM);
    int nSR = sonarSideR.Ranging(CM);
    int nSL = sonarSideL.Ranging(CM);
    delay(1000);
    if (nL >= 8 && nSL >= 30 && nSR >= 15) { // поворот на 90гр влево
      turnLEFT();
      gogo(200, 200, 200, 200, 15);

      nL = sonarL.Ranging(CM);
      nR = sonarR.Ranging(CM);
      nC = sonarC.Ranging(CM);
      nSR = sonarSideR.Ranging(CM);
      nSL = sonarSideL.Ranging(CM);

      if (nR >= 8 && nSR >= 30 && nSL >= 15) { // поворот на 90гр вправо
        turnRIGHT();
      }
      else if (sonarSideR.Ranging(CM) <= 15) {
        vdolSR(10);
        if (sonarC.Ranging(CM) < 10) {
          gogo(0, 250, 0, 250, 110);
        }
        vdolSR(10);
      };
      else {
        while (SonarC.Ranging(CM) > 10) {
          dvig(200, 200, 200, 200);
        } ;
      }
    }

    else if (nL < 8 && (nR >= 8 && nSR >
                        30 && nSL > 15)) { // поворот на 90гр вправо
      turnRIGHT();
      gogo(200, 200, 200, 200, 15);

      nL = sonarL.Ranging(CM);
      nR = sonarR.Ranging(CM);
      nC = sonarC.Ranging(CM);
      nSR = sonarSideR.Ranging(CM);
      nSL = sonarSideL.Ranging(CM);

      if (nL >= 8 && nSL > 30 && nSR > 15) { // поворот на 90гр влево
        turnLEFT();
      }
      else if (sonarSideR.Ranging(CM) <= 15) {
        vdolSL(10);
        if (sonarC.Ranging(CM) < 15) {
          gogo(250, 0, 250, 0, 110);

        }
        vdolSL(10);
      };
      else {
        while (SonarC.Ranging(CM) > 10) {
          dvig(200, 200, 200, 200);
        };
      }
    }
  }
}


if (distC < 15 && distR < 15) { // поворот влево 2 случай
  nL = sonarL.Ranging(CM);
  nR = sonarR.Ranging(CM);
  nC = sonarC.Ranging(CM);
  nSR = sonarSideR.Ranging(CM);
  nSL = sonarSideL.Ranging(CM);
  delay(1000);
  if (nL >= 8 && nSL > 30 && nSR > 15) {
    turnLEFT();
    if (sonarC.Ranging(CM) > 15) {
      gogo(200, 200, 200, 200, 15);
    }
    else {}
  }
  else {
    if (distB > 15) {
      gogo(-150, -150, -150, -150, 15);
    }
    else {}
  }
}
else if (distC < 15 && distL < 15) { // повоорт вправо 3 случай
  nL = sonarL.Ranging(CM);
  nR = sonarR.Ranging(CM);
  nC = sonarC.Ranging(CM);
  nSR = sonarSideR.Ranging(CM);
  nSL = sonarSideL.Ranging(CM);
  delay(1000);
  if (nR >= 8 && nSR > 30 && nSL > 15) {
    turnRIGHT();
    if (sonarC.Ranging(CM) > 15) {
      gogo(200, 200, 200, 200, 15);
    }
    else {}

  }
  else {
    if (distB > 15) {
      gogo(-150, -150, -150, -150, 15);
    }
    else {}
  }
}


if ((distC < 8 && distR < 8) || (distL < 8 && distC < 8) || distC < 8 || distL < 4 || distR < 4) { // экстремальная остановка
  dvig(0, 0, 0, 0);
}
else if (distL > 8 && distC > 8 && distR > 8) { // динамическое изменение скорости (зависимость от расстояния до препятствия)
  int x = 150;
  int y = 750;
  float k = 0.9;
  float sp ;
  sp = x - k * (y / distC);
  sFL = sp;
  sFR = sp;
  sBL = sp;
  sBR = sp;
}


dvig (sFL, sFR, sBL, sBR);
}

bool tryApplyManualCommand(Command command) {
  bool isSuccess = false;
  switch (command) {
    case FORWARD_CMD:
      if (distC > 7 && distL > 10 && distR > 10) {
        dvig(130, 130, 130, 130);
        isSuccess = true;
      }
      break;

    case BACKWARD_CMD:
      if (distB > 10) {
        dvig(-100, -100, -100, -100);
        isSuccess = true;
      }
      break;

    case LEFT_TURN_CMD:
      if (distC > 5 && distL > 8) {
        dvig(-220, 220, -220, 220);
        isSuccess = true;
      }
      break;

    case RIGHT_TURN_CMD:
      if (distC > 5 && distR > 8) {
        dvig(220, -220, 220, -220);
        isSuccess = true;
      }
      break;

    default:
      //Команда "стоп" выполняется единожды, после остановки робот находится в ожидании
      dvig(0, 0, 0, 0);
      currentCommand = NONE_CMD;
      isSuccess = true;
      break;
  }
  return isSuccess;
}

void loop() {
  //Проверка на новые команды
  if (Serial1.available() > 0) {
    uint8_t data = Serial1.read();
    data -= 48;
    if (data < 7) {
      currentCommand = (Command)data;
    }
  }
  else if (Serial2.available() > 0) {
    uint8_t data = Serial2.read();
    data -= 48;
    if (data < 7) {
      currentCommand = (Command)data;
    }
  }

  //Команда "автопилот" выполняется единожды, её задача - переключение режима
  if (currentCommand == AUTOPILOT_CMD) {
    if (isAutopilot) {
      isAutopilot = false;
      currentCommand = STOP_CMD;
    }
    else {
      isAutopilot = true;
      currentCommand = NONE_CMD;
    }
  }

  if (currentCommand != NONE_CMD) {
    ultrazvuk();
    if (!tryApplyManualCommand(currentCommand)) {
      //Если путь по направлению загорожен, то остановить робота
      tryApplyManualCommand(STOP_CMD);
    }
    //При поступлении команды движения, режим автопилот сбрасывается
    isAutopilot = false;
  }
  else if (isAutopilot) {
    ultrazvuk();
    m_speed();
  }
  delay(50);
  ultrazvuk();
  Serial.print(distC);
  Serial.print("\n");
  Serial.print(distSideL);
  Serial.print("\n");
  Serial.print(distSideR);
  Serial.print("\n");
  Serial.print(distL);
  Serial.print("\n");
  Serial.print(distR);
  Serial.print("\n");


  Serial.print(distB);
  Serial.print("\n\r");
  Serial.print("\n\r");

  delay(1000);
}
