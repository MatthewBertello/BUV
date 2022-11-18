#pragma once
#ifndef PID_h
#define PID_h

class PID
{
public:
    float p;
    float i;
    float d;
    float c;
    float lastError;
    float deltaTime;
    float integral;
    float derivative;
    float output;
    float target;
    float lastTime;

    PID(float p, float i, float d, float c)
    {
        this->p = p;
        this->i = i;
        this->d = d;
        this->c = c;
        this->lastError = 0;
        this->deltaTime = 0;
        this->integral = 0;
        this->derivative = 0;
        this->output = 0;
        this->target = 0;
        this->lastTime = 0;
    }

    void setTarget(float target)
    {
        this->target = target;
    }

    void reset()
    {
        lastError = 0;
        integral = 0;
        derivative = 0;
        output = 0;
        lastTime = millis();
    }

    float update(float input)
    {
        float error = this->target - input;
        float currentTime = millis();
        this->deltaTime = currentTime - this->lastTime;
        this->lastTime = currentTime;
        this->integral += error * this->deltaTime;
        this->derivative = (error - this->lastError) / this->deltaTime;
        this->lastError = error;
        this->output = this->p * error + this->i * this->integral + this->d * this->derivative + this->c;
        return this->output;
    }
};

#endif