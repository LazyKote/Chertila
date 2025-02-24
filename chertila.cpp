#include <Arduino.h>
#include <Servo.h>
float path_length[5][5] = {{1,3,6,2,7}, {1,3,6,2,7}, {1,3,6,2,7}, {1,3,6,2,7}, {1,3,6,2,7},};
int dot_chain[] = {1,2,3,4,5}; //порядок точек
float path_angle[5][5]; //матрица с углами поворота
Servo servo1;
int encoder = 0;
int start_button = 0;
int motor1Pin1 = 14; 
int motor1Pin2 = 27;
int enable1Pin = 12;
int motor2Pin1 = 26; 
int motor2Pin2 = 25; 
int enable2Pin = 16;
int val = 150;

#define CLK_pin 26      // Сигнальный пин энкодера
#define DT_pin 25       // Сигнальный пин энкодера
#define discrets 1600.0 // Число импульсов на полный оборот
#define d 0.093         // Диаметр колеса
#define PI 3.1415926535 // PI

// Переменная - счетчик энкодера
volatile long long int counter = 0;

// Функция, обновляющая счетчик энкодера
void updateEncoder() {
  static int currentStateCLK;
  static int lastStateCLK;
  currentStateCLK = digitalRead(CLK_pin);
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
    if (digitalRead(DT_pin) != currentStateCLK) {
      counter--;
    } else {
      counter++;
    }
  }
  lastStateCLK = currentStateCLK;
}

// Функция настройки пинов для энкодера
void setup_encoder(int clk, int dt) {
  pinMode(clk, INPUT);
  pinMode(dt, INPUT);
  attachInterrupt(clk, updateEncoder, CHANGE);
  attachInterrupt(dt, updateEncoder, CHANGE);
}

/*
Получение линейной скорости с энкодера

delay_ms - период вызова функции в мс
*/
float get_speed(int delay_ms) {
  float linear = 0;
  /*
  Используем пропорцию

  d * PI (длина окружности) - discrets (число импульсов на оборот)
  пройденное расстояние     - counter (число имульсов за период)
  */

  linear = (counter * d * PI / discrets) * (1000.0 / delay_ms);
  counter = 0;
  return linear;
}

int dot_by_dot(int dot_chain[])
{
    for (int i=0; i<5; i++)
    {
        int dot1 = dot_chain[i];
        int dot2 = dot_chain[i+1];
        memory(path_length, path_angle ,dot1,dot2);
    }
}

int memory(float path_length[5][5], float path_angle[5][5], int dot1, int dot2)
{
    float length = path_length[dot1][dot2];
    float angle = path_angle[dot1][dot2];
    go_distance(length);
    return length, angle;
}

// Функция для перемещения вперед на определенное расстояние
void go_distance(float distance) {
  float cur_dist = 0;
  long tmr1 = 0;
  while (cur_dist < distance) {
    if (millis() - tmr1 > 10) {
      tmr1 = millis();
      float linear = get_speed(10);
      cur_dist += linear / 100.0;
    }
    move(false, cur_dist);
    
  }
  cur_dist = 0;
  stop();
}

void move(bool motor, int val)
 {
  if (motor = 1)
  {
    if (val > 149)
    {
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      analogWrite(enable1Pin,abs(val));
      delay(2);
    }
    else if (val<149)
    {
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH); 
      analogWrite(enable1Pin,abs(val));
      delay(2);
    }
  }
  else
  {
    if (val > 149)
    {
      analogWrite(enable2Pin,abs(val));
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH);
      delay(2);
    }
    else if (val<149)
    {
      analogWrite(enable2Pin,abs(val));
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW); 
      delay(2);
    }
  } 
 }
 void stop()
 {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
 }

void setup()
{
servo1.attach(0); //привязываем к пину
pinMode(encoder, INPUT);
servo1.write(90); //опускаем маркер
setup_encoder(CLK_pin, DT_pin);
}

void loop()
{

}

