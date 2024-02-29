#include <Arduino.h>
#include <Wire.h>

int sensorValues[] = {0, 0, 0, 0, 0, 0, 0, 0};

class motor {
   private:
    int id;                         // motor id
    int pwmPort, in1Port, in2Port;  // ports of the h bridge
    float Kp, Ki, Kd;               // PID constants
    int sensorPort;                 // angle sensor
    float angle;                    // angle value
    float angleRef;                 // target value for angle
    float error, lastError;         // error
    float sumError = 0;             // error integral
    float minAngle, maxAngle;       // joint minimum and maximum angles used to
                               // calculate the current angle
    void move(int pwmValue) {
        /*this function receives the value that we want to write on the motor,
        if the value is positiv move to a certain direction, if the value is
        negative move to the oposite direction
        */
        if (pwmValue > 0) {
            analogWrite(in1Port, HIGH);
            analogWrite(in2Port, LOW);
            analogWrite(pwmPort, pwmValue);
        } else {
            digitalWrite(in1Port, HIGH);
            digitalWrite(in2Port, LOW);
            analogWrite(-pwmPort, pwmValue);
        }
    }

   public:
    motor(int iid, int ipwmPort, int iin1Port, int iin2Port, float iKp,
          float iKi, float iKd, float isensorPort, float iminAngle,
          float imaxAngle) {
        /*
        motor constructor, get motor values and stores it
        */
        id = iid;
        pwmPort = ipwmPort;    // motor h bridge pwmPort
        in1Port = iin1Port;    // motor h bridge in1 port
        in2Port = iin2Port;    // motor h bridge in2 port
        Kp = iKp;              // PID constant
        Ki = iKi;              // PID constant
        Kd = iKd;              // PID constant
        minAngle = iminAngle;  // angle that corresponds to the 0 value of the
                               // potentiometer
        maxAngle = imaxAngle;  // angle that corresponds to the max value of the
                               // potentiometer
        sensorPort = isensorPort;  // port of the potentiometer
    }
    void init() {
        /*
        initiates the motor, basically sets the mode of the pins
        */

        pinMode(pwmPort, OUTPUT);
        pinMode(in1Port, OUTPUT);
        pinMode(in2Port, OUTPUT);
        pinMode(sensorPort, INPUT);
    }
    float control(float iangRef) {
        /*
        runs the PID based on the desired angle iangRef
        */
        angleRef = iangRef;        // stores angRef
        angle = readAngle();       // gets the angle from the sensor
        error = angle - angleRef;  // calculate error
        sumError += error;         // calculate the intgral of the error
        float pwmValue = Kp * error + Ki * sumError +
                         Kd * (error - lastError);  // PID calculus
        move(pwmValue);     // write the PID to the motor
        lastError = error;  // updates last error
    }
    float readAngle() {
        //! TODO: implement comunication with slave to ask angle
        return map(analogRead(sensorPort), 0, 255, minAngle, maxAngle);
    }
};

motor myMotor(0, 1, 2, 3, 10, 10, 10, 5, 0, 90);

void setup() {
    Wire.begin();
    myMotor.init();
}

int chartoint(char a,char b){
    return a<<4 + b;
}

void loop() {
    while (1) {
        // begin i2c
        Wire.requestFrom(1, 2 * 8);
        int n = 0;
        while (Wire.available()) {  // peripheral may send less than requested
            char a = Wire.read();   // receive a byte as character
            char b = Wire.read();
            sensorValues[n] = chartoint(a,b);
            n ++;
        }

        // end i2c
        int dt = 10;
        int now = millis();

        //! TODO: receber informação das posições desejadas dos motores

        myMotor.control(30);

        while (millis() - now < dt) {
            now = millis();
        }
    }
}