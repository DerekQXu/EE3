/*
 * Team Name: Race Car
 * Members:
 *  Derek Xu
 *  Kevin Wu
 */

//Pins for H-bridge
// L and R denote which motor
// 0 - forward
// 1 - backward
// S - speed
#define L_MOTOR_0 7
#define L_MOTOR_1 8
#define L_MOTOR_S 5

#define R_MOTOR_0 9
#define R_MOTOR_1 10
#define R_MOTOR_S 3

//Pins for sensors
#define REC_LEFT 15
#define REC_CENTER 16
#define REC_RIGHT 14
#define REC_MAG 17 //for hall-effect sensor
#define LED 11 //for hall-effect sensor

//NOTE: in our case both motors had the same coefficient of friction
//  In more often cases, one would need to test to see if driving 180
//  will make the car veer left or right due to differences in the motors' 
//  coefficient of friction. In this case, replace FORWARD_SPEED with 
//  FOWARD_SPEED_L and FORWARD_SPEED_R, tuned to your motors.
#define FORWARD_SPEED 180

#define nTimes 10

#define nTimes 10

//note 32 bit baselines needed to store sensor values
uint32_t baseLine_l, baseLine_c, baseLine_r, baseLine_m, baseLine_total;
uint16_t threshold1, threshold2, threshold, threshold_m;

float correction;

unsigned long init_time;

//PID variables
// adjust the weights to optimal values
// NOTE: these weights work, to tune to fit speed, alter correctionFactor
float weightP = 1.1;
float weightI = 0;
float weightD = 0.8;

//correctionFactor variable
// this value dictates HOW MUCH turning should be implemented based on PID
// adjust this value based on speed of vehical
float correctionFactor = 0.2;

//temporary variables
int16_t err;
int16_t derivative;
int16_t pError;
int32_t integral;
unsigned long dt;

//sets up all input and output pins
void setup()
{
    pinMode(REC_LEFT, INPUT);
    pinMode(REC_CENTER, INPUT);
    pinMode(REC_RIGHT, INPUT);
    
    pinMode(L_MOTOR_0, OUTPUT);
    pinMode(R_MOTOR_0, OUTPUT);
    pinMode(L_MOTOR_1, OUTPUT);
    pinMode(R_MOTOR_1, OUTPUT);
    pinMode(L_MOTOR_S, OUTPUT);
    pinMode(R_MOTOR_S, OUTPUT);
    pinMode(LED, OUTPUT);

    digitalWrite(L_MOTOR_1, LOW);
    digitalWrite(R_MOTOR_1, LOW);
    digitalWrite(L_MOTOR_0, HIGH);
    digitalWrite(R_MOTOR_0, HIGH);
    pinMode(LED_BUILTIN, OUTPUT);

    uint8_t count = 0;
    baseLine_l = baseLine_c = baseLine_r = 0;

    //sampling the average sensor values to create baseline (removes noise)
    while (count < nTimes)
    {
        baseLine_c += analogRead(REC_CENTER);
        delay(15);
        baseLine_l += analogRead(REC_LEFT);
        delay(15);
        baseLine_r += analogRead(REC_RIGHT);
        delay(15);
        baseLine_m += analogRead(REC_MAG);
        delay(15);
        ++count;
        delay(10);
    }
    baseLine_c /= count;
    baseLine_l /= count;
    baseLine_r /= count;
    baseLine_m /= count;

    //calculating baseline (ensures a good error curve)
    baseLine_total = baseLine_c * (((float)baseLine_l)/100 - ((float)baseLine_r)/100);

    //setting thresholds
    threshold1 = 0;
    threshold2 = 0;

    //Ease into start running (break static friction)
    // This ensures that both motors break static friction before reaching
    // respective FORWARD_SPEED's
    int counter = 0;
    while (counter < FORWARD_SPEED)
    {
      counter++;
      analogWrite(L_MOTOR_S, counter);
      analogWrite(R_MOTOR_S, counter);
    }

    threshold_m = 100;
    Serial.begin(9600);
}

void loop()
{
    // for now pError at t = 0 is error at t = 0
    pError = (analogRead(REC_CENTER)*((analogRead(REC_LEFT))/100-(analogRead(REC_RIGHT))/100))-baseLine_total;
    integral = 0;

    //calculate dt
    dt = millis()-init_time;
    init_time = millis();

    //calculate error
    err = (analogRead(REC_CENTER)*((analogRead(REC_LEFT))/100-(analogRead(REC_RIGHT))/100))-baseLine_total;

    //calculate PID variables
    derivative = pError - err;
    integral += 2 * dt * err;
    pError = err;
    correction = weightP * err + weightI * integral + weightD * derivative;

    //this turns the motors (NOTE: set a good correctionFactor)
    analogWrite(L_MOTOR_S, FORWARD_SPEED + correction * correctionFactor);
    analogWrite(R_MOTOR_S, FORWARD_SPEED - correction * correctionFactor);

    //for magnet sensing
    if (analogRead(REC_MAG) - baseLine_m > threshold_m)
      digitalWrite(LED, HIGH);
    else
      digitalWrite(LED, LOW);
}
