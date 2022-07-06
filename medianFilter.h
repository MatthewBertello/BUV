#ifndef medianFilter_h
#define medianFilter_h

class medianFilter
{

public:
    int size = 0;
    int *values = NULL;

    medianFilter(int size)
    {
        this->size = size;
        values = new int[size];
    }

    // Add a value to the filter
    void add(int value)
    {
        for (int i = 0; i < size - 1; i++)
        {
            values[i] = values[i + 1];
        }
        values[size - 1] = value;
    }

    // Get the median value of the filter
    int getMedian()
    {
        int median = 0;
        int *tempArray = new int[size];
        for (int i = 0; i < size; i++)
        {
            tempArray[i] = values[i];
        }
        sort(tempArray, size);
        if (size % 2 == 0)
        {
            median = (tempArray[size / 2] + tempArray[size / 2 - 1]) / 2;
        }
        else
        {
            median = tempArray[size / 2];
        }
        delete[] tempArray;
        return median;
    }

    // Sort an array
    void sort(int *array, int arraySize)
    {
        int temp;
        for (int i = 0; i < arraySize - 1; i++)
        {
            for (int j = 0; j < arraySize - 1; j++)
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
};

#endif