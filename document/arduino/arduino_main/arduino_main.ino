// Параметры отладки программы
//#define DEBUG_ENCODER
#include <Wire.h>
#include <Servo.h>

// Пины подключения
#define MOTOR_PWM         5
#define MOTOR_A1          7
#define MOTOR_B1          8
#define SERVO_PIN         10
// Параметры машинки
#define TICK_REVOLUTION   120.0

#ifdef DEBUG_ENCODER
  #define WAIT_CMD        15000
#else
  #define WAIT_CMD        2000
#endif

class Motor {
private:
  int pwm, 
    a1, 
    b1;

  bool direction_;
public:
  Motor(int pwm_, int a1_, int b1_):
    pwm(pwm_),
    a1(a1_),
    b1(b1_)
  {
    pinMode(pwm, OUTPUT);
    pinMode(a1_, OUTPUT);
    pinMode(b1_, OUTPUT);

    speed(0);
    stop(false);
    direction_ = true;
  }

  /*
   * flag = false, BREAKEGND
   * flag = true, BREAKEVCC
   */
  void stop(bool flag) {
    if(flag) {
      digitalWrite(a1, HIGH);
      digitalWrite(b1, HIGH);
      speed(255);
    }else{
      digitalWrite(a1, LOW);
      digitalWrite(b1, LOW);
      speed(0);
    }
  }

  /*
   * flag = false, назад
   * flag - true, вперёд
   */
  void direction(bool flag) {
    direction_ = flag;
    if(flag) {
      digitalWrite(a1, LOW);
      digitalWrite(b1, HIGH);
    }else{
      digitalWrite(a1, HIGH);
      digitalWrite(b1, LOW);
    }
  }

  void speed(int speed_) {
    analogWrite(pwm, speed_);
  }

  bool get_direction() { return direction_; }
};

struct Regulator {
  int output = 0;
  int input = 0, set_point = 0;
  float error = 0.0, 
        integral = 0.0;
  float kp = 0.1, 
        ki = 0.1;
};

void receive_data(int bytes);
void request_data();
void encoder_update();

#ifdef DEBUG_ENCODER
void parsing();

#endif

Servo servo;
Motor motor(MOTOR_PWM, MOTOR_A1, MOTOR_B1);
int turn = 90, old_turn = 90;
Regulator reg;
unsigned long time = 0,
              time_wire = 0;
volatile bool encoder_increase = true;
volatile long encoder_pulse = 0,
              encoder_pulse_old = 0,
              encoder_pusle_old_wire = 0;

void setup() {
#ifdef DEBUG_ENCODER
  // Для плоттера
  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println("setpoint, input, output");
#endif

  Wire.begin(8);
  // Регистрирует функцию, вызываемую, когда ведомое устройство получает передачу от ведущего
  Wire.onReceive(receive_data);
  // Регистрирует функцию, вызываемую, когда ведущее устройство получает передачу от ведомого
  Wire.onRequest(request_data);

  attachInterrupt(1, encoder_update, FALLING);

  motor.direction(true);

  servo.attach(SERVO_PIN);
  servo.write(turn);

  time = millis();
  time_wire = millis();
}

void loop() {
  // Угол поворота передних колёс
  if(turn != old_turn) {
    servo.write(turn);
    old_turn = turn;
  }

  if((millis() - time_wire) >= WAIT_CMD) 
    reg.set_point = 0;

  if((millis() - time) >= 300) {
    unsigned long now_time = (millis() - time);
    // Расчёт ШИМ для регулятора мотора
    reg.input = abs(360.0 * (encoder_pulse - encoder_pulse_old)) / (TICK_REVOLUTION * (now_time / 1000.0));
    encoder_pulse_old = encoder_pulse;
      
    // ПИ регулятор
    reg.error = abs(reg.set_point) - reg.input;
    reg.integral = constrain(reg.integral + reg.error * reg.ki * (now_time / 1000.0), 0, 255);
    reg.output = (abs(reg.set_point) > 0 ? constrain(reg.error * reg.kp + reg.integral, 0, 255) : 0);

    // Установить направление движения
    bool new_direction = (reg.set_point >= 0);
    if(motor.get_direction() != new_direction) {
      encoder_increase = new_direction;
      motor.direction(new_direction);
    }

    motor.speed(reg.output);
    
    time = millis();
  }

#ifdef DEBUG_ENCODER
  // Для плоттера
  Serial.print(reg.set_point);
  Serial.print(',');
  Serial.print(reg.input);
  Serial.print(',');
  Serial.println(reg.output);

  parsing();
#endif

  delay(10);
}

#ifdef DEBUG_ENCODER
void parsing() {
  if(Serial.available() > 1) {
    char argument = Serial.read();
    float value = Serial.parseInt();
    
    switch(argument) {
      case 'd':
        reg.set_point = value;
         time_wire = millis();
      break;
    }
  }
}
#endif

void receive_data(int bytes) {
  uint8_t a = Wire.read(),
          b = Wire.read();
          
  reg.set_point = ((b << 8) | a);
  turn = (uint8_t)Wire.read();
  time_wire = millis();
}

void request_data() {
  union {
    int16_t data;
    byte b_data[sizeof(data)];
  } td;
  
  td.data = (360 * (encoder_pulse - encoder_pusle_old_wire)) / TICK_REVOLUTION;
  encoder_pusle_old_wire = encoder_pulse;

  Wire.write(td.b_data, 2);
}

void encoder_update() {
  encoder_pulse += (encoder_increase ? 1 : -1);
}
