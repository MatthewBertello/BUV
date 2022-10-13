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
    }

    void setTarget(float target)
    {
        this->target = target;
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