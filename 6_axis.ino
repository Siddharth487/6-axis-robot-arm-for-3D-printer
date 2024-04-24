#include <SoftwareSerial.h>

// Arduino pin numbers
const int left_SW_pin = 7;
const int left_x_pin = A0;
const int left_y_pin = A1;
const int right_SW_pin = 8;
const int right_x_pin = A5;
const int right_y_pin = A4;
//const int end_effector = A2;
const int end_switch = 9; 
const int wrist = A2;
const int end_effector = A3;



unsigned long lastButtonPress = 0;

// Offsets for joystick deadzone - get's setup at startup
int left_base_y, left_base_x;
int right_base_y, right_base_x;
int wrist_base, end_effector_base;
//int end_effector_base;

// software serial 
SoftwareSerial ssc32u(12, 13);

void setup()
{
    // Pin declarations
    pinMode(left_SW_pin, INPUT);
    pinMode(right_SW_pin, INPUT);
    pinMode(end_switch, INPUT);
    digitalWrite(left_SW_pin, HIGH);
    digitalWrite(right_SW_pin, HIGH);
    digitalWrite(end_switch, HIGH);

    // Serial Setup
    Serial.begin(9600);
    Serial.println("===> System Init <===\n");
    Serial.print("Connecting to SSC-32U...");
    // ssc32u Controller Setup
    ssc32u.begin(9600);
    ssc32u.listen();
    Serial.println(" Done");

    // Center servos
    Serial.print("Centering Servos...");
    ssc32u.write("#0P1500 #1P1500 #2P1500 #3P1500 #4P1500 #5P1500 #6P1500\r");
    Serial.println(" Done");
    Serial.print("Calibrating Joysticks & Enocoder...");
    left_base_y = analogRead(left_y_pin);
    left_base_x = analogRead(left_x_pin);
    right_base_y = analogRead(right_y_pin);
    right_base_x = analogRead(right_x_pin);
    //end_effector_base = analogRead(end_effector);
    wrist_base = analogRead(wrist);
    end_effector_base = analogRead(end_effector);

    Serial.println(" Done\n");
    Serial.println("===> Starting Main Loop <===");
}

// Define arrays for servo values, maxes, and minimums
uint16_t pulses[6] = {1500, 1500, 1500, 1500, 1500, 1500};
uint16_t max[6] = {2500, 2000, 1800, 2500, 2500, 2500};
uint16_t min[6] = {500, 800, 800, 500, 500, 500};

void loop()
{
    // Reset Servos to Base values when Motor-Encoder is pressed
    int btnState = digitalRead(left_SW_pin);
    // If we detect LOW signal, button is pressed
    if (btnState == LOW)
    {
        // if 50 ms since LOW pulse, button has been pressed
        
        if (millis() - lastButtonPress > 50)
        {
            for (uint8_t i = 0; i < 6; i++)
            {
                pulses[i] = 1500;
            }
            // Make the reset motion smoother
            ssc32u.write("#0P1500 #1P1500 #2P1500 #3P1500 #4P1500 #5P1500 #6P1500 T1000 \r");
            delay(1000);
        }
        // store last button state
        lastButtonPress = millis();
    }

    // Formula to refine motor movement for the 6 axes
    pulses[0] -= (analogRead(left_y_pin) - left_base_y) / 25;   // Base
    pulses[1] += (analogRead(left_x_pin) - left_base_x) / 25;   // Shoulder
    pulses[2] += (analogRead(right_x_pin) - right_base_x) / 25; // Elbow
    pulses[3] += (analogRead(right_y_pin) - right_base_y) / 25; // Wrist-vert
    pulses[4] += (analogRead(wrist) - wrist_base) / 25; // 360 degree wrist rotation
    pulses[5] += (analogRead(end_effector) - end_effector_base) / 25; // gripper

    // Prevent servos from over-extending
    for (uint8_t i = 0; i < 6; i++)
    {
        if (pulses[i] >= max[i])
        {
            pulses[i] = max[i];
        }
        else if (pulses[i] <= min[i])
        {
            pulses[i] = min[i];
        }
    }

    // Motor instructions are as follows, note: S and T are optional
    // #<servo_num>P<pulse>S<speed>T<time>\r
    // note: if time is at the end of the statement then it will effect all servos
    // #0P1500 #1P1500 #2P1500 T100
    char output[80];
    sprintf(output, "#0P%u #1P%u #2P%u #3P%uT100 #4P%u #5P%u\r", pulses[0], pulses[1], pulses[2], pulses[3], pulses[4], pulses[5]);
    ssc32u.write(output);
}

