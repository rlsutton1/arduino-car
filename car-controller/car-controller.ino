#include <Encoder.h>
#include <Servo.h>

Servo myservo;

class SpeedMonitor {
  private:
    double speed = 0.0;
    double position = 0.0;
    Encoder *encoder;


  public:
    SpeedMonitor(Encoder *encoder)
    {

      this->encoder = encoder;
    }

    void readEncoder()
    {
      double newPosition = encoder->read();

      speed = ( newPosition - position);
      position = newPosition;
    }

    int getSpeed()
    {
      return speed;
    }

    int getPosition()
    {
      return position;
    }
};


class MotorController {
  private:
    int directionPin;
    int pwmPin;
  public:
    MotorController(int directionPin, int pwmPin)
    {
      this->directionPin = directionPin;
      this->pwmPin = pwmPin;
      pinMode(directionPin, OUTPUT);
      pinMode(pwmPin, OUTPUT);
    }

    void setSpeed( int speed)
    {
      if (speed < 0)
      {
        digitalWrite(this->directionPin, HIGH);
      } else
      {
        digitalWrite(this->directionPin, LOW);
      }
      analogWrite(this->pwmPin, abs(speed));
    }
};



double   motor2target = 50;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(9);
}

long ticks = 0;

class MPid {

    double lastMeasuredSpeed = 0.0;
    double throttle = 0.0;
    // 1 < throttleResponse < 256
    double throttleResponse = 4;

  public:

    double pid(double targetSpeed, double measuredSpeed)
    {
      // calculate current accelleration
      double currentAccel = measuredSpeed - lastMeasuredSpeed;

      // note the speed for next time
      lastMeasuredSpeed = measuredSpeed;

      // calculate the accelleration required in one step.
      double delta = targetSpeed - measuredSpeed;

      // only allow half the accelleration to prevent overshoot.
      double desiredAccel = abs(delta) / 2.0;

      // scale the throttle response by the ratio of the desired change in
      // accelleration compared to the current accelleration.
      double throttleScaling = throttleResponse * (abs( desiredAccel -
                               currentAccel) / max(1, abs(currentAccel)));

      // ensure throttleScalling stays within the bounds.
      double throttleResponseLimitScaler = 2.0;
      if (signum(currentAccel) != signum(delta))
      {
        //unlimited if we are accellerating in the wrong direction
        throttleResponseLimitScaler = 10000.0;
      }

      // apply throttleResponseLimitScaler
      throttleScaling = min(throttleResponse * throttleResponseLimitScaler, throttleScaling);

      // never more than a quarter throttle adjustment otherwise we will overshoot
      throttleScaling = min(255.0 / 4.0, throttleScaling);

      // always atleast 1 so we always converge
      throttleScaling = max(1, throttleScaling);

      // calculate the throttle adjustment.
      double throttleStep = signum(delta) * throttleScaling;

      // adjust the throttle
      throttle += throttleStep ;

      // ensure the thorttle stays within bounds.
      if (throttle > 255)
        throttle = 255;
      if (throttle < -255)
        throttle = -255;

      return throttle;

    }

};

MotorController motor2 = MotorController(4, 5);
MotorController motor1 = MotorController(7, 6);
Encoder encoder1(0, 1);
SpeedMonitor motor1SpeedMonitor(&encoder1);
Encoder encoder2(2, 3);
SpeedMonitor motor2SpeedMonitor(&encoder2);
MPid mpid = MPid();

double m1tcs = 1.0;
double m2tcs = 1.0;
boolean slip = false;

int loopCount = 0;

void loop() {
  // put your main code here, to run repeatedly:

  loopCount++;

  if (loopCount > 200)
  {
    // range is 0 to 180 degrees
    myservo.write(45);
  } else  if (loopCount  > 100)
  {
    myservo.write(90);
  } else
  {
    myservo.write(135);
  }
  if (loopCount > 300)
  {
    loopCount = 0;
  }

  motor1SpeedMonitor.readEncoder();
  motor2SpeedMonitor.readEncoder();

  double sp1 = motor1SpeedMonitor.getSpeed();
  double sp2 = motor2SpeedMonitor.getSpeed();

  double speed = (sp1 + sp2) / 2.0;
  double throttle = mpid.pid(motor2target, speed);

  // traction control logic

  // progressively increase throttle limit back to 1
  m1tcs = min(1, m1tcs + 0.05);
  m2tcs = min(1, m2tcs + 0.05);

  double absSp1 = abs(sp1);
  double absSp2 = abs(sp2);

  double maxSlip = 1.5;

  // check both wheels are turning fast enought to be able to detect slip
  if (absSp1 + absSp2 > 10)
  {
    // check if motor 1 is slipping
    if (absSp1 > maxSlip * absSp2)
    {
      // reduce power of motor 1
      m1tcs = m1tcs * 0.5;
      slip = true;
    }
    // check if motor 2 is slipping
    if (absSp2 > maxSlip * absSp1)
    {
      //reduce power of motor2
      m2tcs = m2tcs * 0.5;
      slip = true;
    }
  }

  // normalize tcs - either m1 or m2 should always = 1
  double tcsNormalizer = 1.0 / max(max(m1tcs, m2tcs), 0.01);
  m1tcs = m1tcs * tcsNormalizer;
  m2tcs = m2tcs * tcsNormalizer;

  motor1.setSpeed(-1 * throttle * m1tcs);
  motor2.setSpeed(throttle * m2tcs);

  ticks++;
  if (ticks % 10 == 0)
  {
    Serial.print(slip);
    Serial.print(' ');
    Serial.print((int)sp1);
    Serial.print(" ");
    Serial.print((int)sp2);
    Serial.print(' ');
    Serial.print((int)speed);
    Serial.print(' ');
    Serial.print((int)motor2target);
    Serial.print(' ');
    Serial.print(throttle);

    Serial.println();
    slip = false;
  }
  if (Serial.available() > 2) {
    motor2target = Serial.parseInt();
  }
  delay(50);
}

double signum(double value)
{
  if (value > 0)
    return 1;
  if (value < 0)
    return -1;
  return 0;
}
