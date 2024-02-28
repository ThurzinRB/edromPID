#include <Arduino.h>

// Added comment

class motor {
   private:
    int id;
    int pwmPort, in1Port, in2Port;  // ports of the h bridge
    float Kp, Ki, Kd;               // PID constants
    int sensorPort;                 // angle sensor
    float angle;                    // angle value
    float angleRef;                 // target value for angle
    float error, lastError;         // error
    float minAngle, maxAngle;
    float sumError = 0;
    void move(int pwmValue) {
        if (pwmValue > 0) {
            analogWrite(in1Port, HIGH);
            analogWrite(in2Port, LOW);
        } else {
            digitalWrite(in1Port, HIGH);
            digitalWrite(in2Port, LOW);
        }
        analogWrite(pwmPort, pwmValue);
    }

   public:
    motor(int iid, int ipwmPort, int iin1Port, int iin2Port, float iKp, float iKi,
          float iKd, float isensorPort, float iminAngle, float imaxAngle) {
        id = iid;
        pwmPort = ipwmPort;
        in1Port = iin1Port;
        in2Port = iin2Port;
        Kp = iKp;
        Ki = iKi;
        Kd = iKd;
        minAngle = iminAngle;
        maxAngle = imaxAngle;
        sensorPort = isensorPort;
        
    }
    void init(){
        pinMode(pwmPort, OUTPUT);
        pinMode(in1Port, OUTPUT);
        pinMode(in2Port, OUTPUT);
        pinMode(sensorPort, INPUT);
    }
    float control(float iangRef){
        angleRef = iangRef;
        angle = readAngle();
        error = angle - angleRef;
        sumError += error;
        float pwmValue = Kp * error + Ki * sumError + Kd * (error - lastError);
        move(pwmValue);
        lastError = error;
    }
    float readAngle(){

        //!TODO: implement comunication with slave to ask angle
        return map(analogRead(sensorPort), 0, 255, minAngle, maxAngle);
    }
};

motor myMotor(0,1,2,3,10,10,10,5,0,90);


void setup() {
    myMotor.init();
}

void loop() {
    while (1) {
        int dt = 10;
        int now = millis();
        
        myMotor.control(30);

        while (millis() - now < dt) {
            now = millis();
        }
    }
}