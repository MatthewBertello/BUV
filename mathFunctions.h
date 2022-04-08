#ifndef mathFunctions_h
#define mathFunctions_h

namespace mathFunctions
{
    double map(double x, double minimumInput, double maximumInput, double minimumOutput, double maximumOutput)
    {
        return (x - minimumInput) * (maximumOutput - minimumOutput) / (maximumInput - minimumInput) + minimumOutput;
    }
}

#endif