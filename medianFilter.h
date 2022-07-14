#ifndef medianFilter_h
#define medianFilter_h

class medianFilter
{

public:
    int size = 1;
    int startPosition = 0;
    // create an array of ints
    int *values;

    medianFilter()
    {
        this->size = 1;
        this->values = new int[size];
    }

    medianFilter(int size, int initialValue)
    {
        this->size = size;
        this->values = new int[size];
        for (int i = 0; i < size; i++)
        {
            values[i] = initialValue;
        }
    }

    // Add a value to the filter
    void add(int value)
    {
        values[startPosition] = value;
        startPosition++;
        if (startPosition == size)
        {
            startPosition = 0;
        }
    }

    // Get the median value of the filter
    int getMedian()
    {
        int tempValues[size];
        for (int i = 0; i < size; i++)
        {
            tempValues[i] = values[i];
        }

        sort(tempValues, size);
        return tempValues[size / 2];
    }

    void sort(int *array, int size)
    {
        int temp;
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size - 1; j++)
            {
                if (array[j] > array[j + 1])
                {
                    temp = array[j];
                    array[j] = array[j + 1];
                    array[j + 1] = temp;
                }
            }
        }
    }

    void print()
    {
        Serial.print("[");
        for (int i = 0; i < size; i++)
        {
            Serial.print(values[i]);
            if (i != size - 1)
            {
                Serial.print(", ");
            }
        }
        Serial.print("]");
    }
};

#endif
