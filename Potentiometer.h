#pragma once
#ifndef Potentiometer_h
#define Potentiometer_h

class Potentiometer
{
public:
    int zeroPosition = 0;
    int threshold = 0;

    Potentiometer()
    {
    }

    int position()
    {
        return 0;
    }
};

#endif
