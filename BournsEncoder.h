#pragma once
#ifndef BournsEncoder_h
#define BournsEncoder_h

class BournsEncoder
{
public:
    const int INPUTS[254] = {56, 40, 55, 24, -1, 39, 52, 8, 57, -1, -1, 23, -1, 36, 13, 120, -1, 41, 54, -1, -1, -1, 53, 7, -1, -1, -1, 20, 19, 125, 18, 104, 105, -1, -1, 25, 106, 38, -1, -1, 58, -1, -1, -1, -1, 37, 14, 119, 118, -1, -1, -1, 107, -1, -1, 4, -1, 3, -1, 109, 108, 2, 1, 88, -1, 89, -1, -1, -1, -1, 51, 9, 10, 90, -1, 22, 11, -1, 12, -1, -1, 42, 43, -1, -1, -1, -1, -1, -1, -1, -1, 21, -1, 126, 127, 103, -1, 102, -1, -1, -1, -1, -1, -1, -1, 91, -1, -1, -1, -1, -1, 116, 117, -1, -1, 115, -1, -1, -1, 93, 94, 92, -1, 114, 95, 113, 0, 72, 71, -1, 68, 73, -1, -1, 29, -1, 70, -1, 69, -1, -1, 35, 34, 121, -1, 122, -1, 74, -1, -1, 30, 6, -1, 123, -1, -1, -1, 124, 17, -1, -1, -1, 67, 26, -1, 27, 28, -1, 59, -1, -1, -1, -1, -1, 15, -1, -1, -1, -1, -1, -1, -1, -1, 5, -1, -1, -1, 110, -1, 111, 16, 87, 84, -1, 45, 86, 85, -1, 50, -1, -1, -1, 46, -1, -1, -1, 33, -1, 83, -1, 44, 75, -1, -1, 31, -1, -1, -1, -1, -1, -1, -1, 32, 100, 61, 101, 66, -1, 62, -1, 49, 99, 60, -1, 47, -1, -1, -1, 48, 77, 82, 78, 65, 76, 63, -1, 64, 98, 81, 79, 80, 97, 96, 112};
    int pins[8];
    int previousOutput = -1;

    BournsEncoder(int pin1, int pin2, int pin3, int pin4, int pin5, int pin6, int pin7, int pin8)
    {
        this->pins[0] = pin1;
        this->pins[1] = pin2;
        this->pins[2] = pin3;
        this->pins[3] = pin4;
        this->pins[4] = pin5;
        this->pins[5] = pin6;
        this->pins[6] = pin7;
        this->pins[7] = pin8;

        pinMode(pin1, INPUT);
        pinMode(pin2, INPUT);
        pinMode(pin3, INPUT);
        pinMode(pin4, INPUT);
        pinMode(pin5, INPUT);
        pinMode(pin6, INPUT);
        pinMode(pin7, INPUT);
        pinMode(pin8, INPUT);
    }
    int getPosition()
    {
        int output = 0;
        for (int i = 7; i >= 0; i--)
        {
            output += digitalRead(this->pins[i]) * pow(2, i) + .5;
        }
        //for (int i = 7; i >= 0; i--)
        //{
        //    Serial.print(i);
        //    Serial.print(": ");
        //    Serial.println(digitalRead(this->pins[i]));
        //}
        //Serial.println("output: " + String(output));
        return INPUTS[output - 1];
    }
    void recordPositions()
    {
        int output = 0;
        for (int i = 7; i >= 0; i--)
        {
            output += digitalRead(this->pins[i]) * pow(2, i) + .5;
        }
        if (output != this->previousOutput)
        {
            this->previousOutput = output;
            for (int i = 7; i >= 0; i--)
            {
                Serial.print(digitalRead(this->pins[i]));
                Serial.print(" ");
            }
            Serial.print(String(output));
            Serial.print(" ");
            Serial.println(INPUTS[output - 1]);
        }
    }
};

#endif
