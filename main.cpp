#include <Arduino.h>
#include <Servo.h>
float path_length[7][7] = {
{0, 26.5, 45.5, 49.5, 68.0, 0, 54.0}, 
{26.5, 0, 26.5, 24.5, 56.5, 0, 46.0}, 
{49.5, 26.5, 0, 35.5, 34.5, 64.0, 69.1}, 
{49.5, 24.5, 35.5, 0, 51.0, 33.5, 38.5}, 
{68.0, 56.5, 34.5, 51.0, 0, 61.0, 89.0},
{0, 0, 64.0, 33.5, 61.0, 0, 51.0},
{54.0, 46.0, 69.1, 38.5, 89.0, 51.0, 0}};
int dot_chain[] = {1,2,3,4,5}; //порядок точек
float path_angle[7][7] = {
  {90, 90, 90, 90, 90, 90, 90}, 
  {90, 90, 90, 90, 90, 90, 90}, 
  {90, 90, 90, 90, 90, 90, 90}, 
  {90, 90, 90, 90, 90, 90, 90}, 
  {90, 90, 90, 90, 90, 90, 90}, 
  {90, 90, 90, 90, 90, 90, 90}, 
  {90, 90, 90, 90, 90, 90, 90}}; //матрица с углами поворота
Servo servo1;
int start_button = 0;
int motor1Pin1 = 14; 
int motor1Pin2 = 27;
int speed1Pin = 12;
int motor2Pin1 = 26; 
int motor2Pin2 = 25; 
int speed2Pin = 16;

#define CLK_pin 26      // Сигнальный пин энкодера
#define DT_pin 25       // Сигнальный пин энкодера
#define discrets 1600.0 // Число импульсов на полный оборот
#define d 0.095         // Диаметр колеса
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
// Функция для поворота на заданный угол
void turn_angle(float target_angle) {
  float current_angle = 0;
  long tmr1 = millis();
  float initial_counter = counter;

  // Коэффициент для перевода импульсов энкодера в градусы
  float angle_per_pulse = 360.0 / discrets;

  while (abs(current_angle) < abs(target_angle)) {
    if (millis() - tmr1 > 10) {
      tmr1 = millis();
      current_angle = (counter - initial_counter) * angle_per_pulse;

      if (target_angle > 0) {
        // Поворот вправо
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW);
        digitalWrite(motor2Pin1, LOW);
        digitalWrite(motor2Pin2, HIGH);
      } else {
        // Поворот влево
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, HIGH);
        digitalWrite(motor2Pin1, HIGH);
        digitalWrite(motor2Pin2, LOW);
      }
      analogWrite(speed1Pin, 150);
      analogWrite(speed2Pin, 150);
    }
  }
  stop(); // Остановка моторов после поворота
}

//движение моторов вперёд
void move(float cur_dist)
 {
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      analogWrite(speed1Pin,abs(cur_dist));
      analogWrite(speed2Pin,abs(cur_dist));
      delay(2);
}

 //остановка моторов
 void stop()
 {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
 }
 
// Функция для перемещения вперед на определенное расстояние
void go_distance(float distance, float angle) {
  float cur_dist = 0;
  long tmr1 = 0;
  while (cur_dist < distance) {
    if (millis() - tmr1 > 10) {
      tmr1 = millis();
      float linear = get_speed(10);
      cur_dist += linear / 100.0;
    }
    if (angle != 90) {
      turn_angle(angle);
    }
    move(cur_dist);
    move(cur_dist);
  }
  cur_dist = 0;
  stop();
}
 
//Берём точку начала и конечную точку
int dot_by_dot(int dot_chain[])
{
    for (int i=0; i<5; i++)
    {
        int dot1 = dot_chain[i];
        int dot2 = dot_chain[i+1];
        go_distance(path_length[dot1][dot2], path_angle[dot1][dot2]);
    }
}

void setup()
{
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(speed1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(speed2Pin, OUTPUT);
  pinMode(start_button, OUTPUT);
  servo1.attach(0); //привязываем свервопривод к пину
  servo1.write(90); //опускаем маркер
  setup_encoder(CLK_pin, DT_pin);
}

void loop()
{
 dot_by_dot(dot_chain);
}