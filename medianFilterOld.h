#ifndef medianFilterOld_h
#define medianFilterOld_h

#include "LinkedList.h"

class medianFilter
{

public:
    int size = 1;
    LinkedList values = LinkedList();

    medianFilter()
    {
        this->size = 1;
        this->values = LinkedList();
    }

    medianFilter(int size, int initialValue)
    {
        this->size = size;
        for (int i = 0; i < size; i++)
        {
            values.add(initialValue);
        }
    }

    // Add a value to the filter
    void add(int value)
    {
        values.addStart(value);
        values.removeLast();
    }

    // Get the median value of the filter
    int getMedian()
    {
        LinkedList tempValues = LinkedList();
        for (int i = 0; i < values.getSize(); i++)
        {
            tempValues.add(values.get(i));
        }

        tempValues.sort();
        return tempValues.get(tempValues.getSize() / 2);
    }

    void print()
    {
        values.print();
    }
};

#endif
