#include <Arduino.h>
#include <Servo.h>
//матрица "расстояний"
/*float path_length[7][7] = {
  {0.0, 26.5, 45.5, 49.5, 68.0, 0.0, 54.0}, 
  {26.5, 0.0, 26.5, 24.5, 56.5, 0.0, 46.0}, 
  {49.5, 26.5, 0.0, 35.5, 34.5, 64.0, 69.1}, 
  {49.5, 24.5, 35.5, 0.0, 51.0, 33.5, 38.5}, 
  {68.0, 56.5, 34.5, 51.0, 0.0, 61.0, 89.0},
  {0.0, 0.0, 64.0, 33.5, 61.0, 0.0, 51.0},
  {54.0, 46.0, 69.1, 38.5, 89.0, 51.0, 0.0}};*/
int dot_chain[] = {1,2,3,4,5}; //порядок точек
/*float path_angle[7][7] = {
  {0, 90.0, 100.0, 48.0, 90.0, 50.0, 30.0}, 
  {180.0, 0.0, 90.0, 90.0, 70.0, 30.0, 90.0}, 
  {100.0, 90.0, 0.0, 10.0, 20.0, 90.0, 90.0}, 
  {90.0, 120.0, 90.0, 0.0, 100.0, 90.0, 90.0}, 
  {60.0, 70.0, 80.0, 100.0, 0.0, 200.0, 90.0}, 
  {90.0, 70.0, 160.0, 90.0, 90.0, 0.0, 90.0}, 
  {90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 0.0}};*/ //матрица с углами поворота
  float path_length[] = {0.56, 0.30, 0.20, 0.50, 0.40};
  float path_angle[] = {0, 90.0, 100.0, 48.0, 90.0};
Servo servo1;
int start_button = 0;
int motor1Pin1 = 9; 
int motor1Pin2 = 8;
int speed1Pin = 7;
int motor2Pin1 = 11; 
int motor2Pin2 = 10; 
int speed2Pin = 5;

#define CLK_pin 2      // Сигнальный пин энкодера
#define DT_pin 3       // Сигнальный пин энкодера
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

 //остановка моторов
 void stop()
 {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
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
  unsigned long tmr1 = millis();
  long initial_counter = counter; // Начальное значение счетчика энкодера
  float angle_per_pulse = 360.0 / discrets; // Градусов на один импульс энкодера

  while (abs(current_angle) < abs(target_angle)) {
      if (millis() - tmr1 > 10) { // Обновляем каждые 10 мс
          tmr1 = millis();
          current_angle = (counter - initial_counter) * angle_per_pulse;

          if (target_angle > 90) {
              // Поворот вправо
              digitalWrite(motor1Pin1, HIGH);
              digitalWrite(motor1Pin2, LOW);
              digitalWrite(motor2Pin1, LOW);
              digitalWrite(motor2Pin2, HIGH);
          } 
          if (target_angle < 90) {
              // Поворот влево
              digitalWrite(motor1Pin1, LOW);
              digitalWrite(motor1Pin2, HIGH);
              digitalWrite(motor2Pin1, HIGH);
              digitalWrite(motor2Pin2, LOW);
          }
          analogWrite(speed1Pin, 130); // Скорость моторов
          analogWrite(speed2Pin, 250);
    }
  }
  stop();
}

//движение моторов вперёд
void move()
 {
      Serial.print("Полный вперёд");
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW);
      analogWrite(speed1Pin, abs(130));
      analogWrite(speed2Pin, abs(250));
      delay(2);
}

// Функция для перемещения вперед на определенное расстояние
void go_distance(float distance, float angle) {
  float cur_dist = 0;
  long tmr1 = 0;
  turn_angle(angle);
  while (cur_dist < distance) {
    if (millis() - tmr1 > 10) {
      tmr1 = millis();
      float linear = get_speed(10);
      cur_dist += linear / 100.0;
    }

    move();
  }
  cur_dist = 0;
  stop();
}
 
//Берём точку начала и конечную точку
void dot_by_dot(int dot_chain[])
{
    for (int i = 0; i < 5; i++)
    {
        int dot1 = dot_chain[i];
        go_distance(path_length[dot1], path_angle[dot1]);
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
  servo1.attach(13); //привязываем свервопривод к пину
  servo1.write(150); //опускаем маркер
  setup_encoder(CLK_pin, DT_pin);
  Serial.begin(9600);
}

void loop()
{
  /*digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(speed1Pin,150);
  analogWrite(speed2Pin,150);
  delay(2);*/
  dot_by_dot(dot_chain);
}