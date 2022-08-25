#ifndef utilities_h
#define utilities_h

namespace utilities
{
    /**
     * Maps a value from one range to another.
     *
     * @param value The value to map
     * @param minimumInput The low value of the range to map from
     * @param maximumInput The high value of the range to map from
     * @param minimumOutput The low value of the range to map to
     * @param maximumOutput The high value of the range to map to
     * @return The mapped value
     */
    double map(double x, double minimumInput, double maximumInput, double minimumOutput, double maximumOutput)
    {
        return (x - minimumInput) * (maximumOutput - minimumOutput) / (maximumInput - minimumInput) + minimumOutput;
    }
}

#endif