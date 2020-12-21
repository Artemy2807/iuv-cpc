// Параметры отладки программы
#define ENABLE_ROS
#define DEBUG_ENCODER

#ifdef ENABLE_ROS
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#endif
#include <Servo.h>

// Параметры ROS
#define SPEED_TOPIC "mobile/speed"
#define STEERING_TOPIC "mobile/cmd_vel"
// Пины подключения
#define MOTOR_PWM   5
#define MOTOR_A1    7
#define MOTOR_B1    8
#define SERVO_PIN   10
// Параметры машинки
#define TICK_FOR_M  55.0

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

void encoder_update();

#ifdef ENABLE_ROS
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64MultiArray> steering_sub(STEERING_TOPIC, feedback);
std_msgs::Float64 speed_msg;
ros::Publisher speed_pub(SPEED_TOPIC, &speed_msg);

void feedback(const std_msgs::Float64MultiArray& cmd);
#endif

#ifdef DEBUG_ENCODER
void parsing();
#endif

Servo servo;
Motor motor(MOTOR_PWM, MOTOR_A1, MOTOR_B1);
int old_turn = 90;
int regulator = 0, regulator_old = 0;
float speed_set = 0.0, speed_real = 0;
float kp = 60.0, 
      ki = 120.0;
float integral = 0;
unsigned long time = 0;
volatile unsigned long encoder_pulse = 0,
                      encoder_pulse_old = 0;

void setup() {
#ifdef ENABLE_ROS
  nh.initNode();
  nh.advertise(speed_pub);
  nh.subscribe(steering_sub);
#endif

#ifdef DEBUG_ENCODER
  // Для плоттера
  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println("setpoint, real, regulator");
#endif

  attachInterrupt(1, encoder_update, FALLING);

  motor.direction(true);

  servo.attach(SERVO_PIN);
  servo.write(old_turn);

  time = millis();
}

void loop() {
  if((millis() - time) >= 300) {
    // Расчёт скорости движения модели
    float denc = encoder_pulse - encoder_pulse_old;
    encoder_pulse_old = encoder_pulse;

    speed_real = (denc / TICK_FOR_M) / 0.1;

#ifdef ENABLE_ROS
    speed_msg.data = speed_real;
#endif
    
    // Установить направление движения
    bool new_direction = (speed_set >= 0.0);
    if(motor.get_direction() != new_direction)
      motor.direction(new_direction);

    // Расчёт ШИМ для регулятора мотора
    regulator_old = regulator;
    if(speed_set != 0.0) {
      // ПИ регулятор
      float error = abs(speed_set) - speed_real;
      integral += error * ki;
      integral = (integral > 200 ? 200 : integral);
      regulator = constrain((error * kp + integral), 0, 255);
      regulator = (regulator + regulator_old) * 0.5;
    }else regulator = 0;

    motor.speed(regulator);
    
    time = millis();
  }

#ifdef ENABLE_ROS
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
#endif

#ifdef DEBUG_ENCODER
  // Для плоттера
  Serial.print(speed_set);
  Serial.print(',');
  Serial.print(speed_real);
  Serial.print(',');
  Serial.println(regulator);

  parsing();
#endif

  delay(10);
}

#ifdef ENABLE_ROS
void feedback(const std_msgs::Float64MultiArray& cmd) {
  speed = cmd.data[0];
  
  int int_turn = (int)cmd.data[1];
  if(old_turn != int_turn) {
    servo.write(int_turn);
    old_turn = int_turn;
  }
}
#endif

#ifdef DEBUG_ENCODER
void parsing() {
  if(Serial.available() > 1) {
    char argument = Serial.read();
    float value = Serial.parseInt();
    
    switch(argument) {
      case 's':
        speed_set = value;
      break;
    }
  }
}
#endif

void encoder_update() {
  encoder_pulse++;
}
