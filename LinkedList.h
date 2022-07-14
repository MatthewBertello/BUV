#ifndef LinkedList_h
#define LinkedList_h

class LinkedList
{
public:
    LinkedList()
    {
        this->head = NULL;
        this->tail = NULL;
        this->size = 0;
    }
    int size;

    class Node
    {
    public:
        int value;
        Node *next;
        Node *prev;

        Node(int value)
        {
            this->value = value;
            this->next = NULL;
            this->prev = NULL;
        }
    };

    Node *head;
    Node *tail;

    void add(int value)
    {
      
        Node *newNode = new Node(value);
        if (this->head == NULL)
        {
            this->head = newNode;
            this->tail = newNode;
        }
        else
        {
            this->tail->next = newNode;
            newNode->prev = this->tail;
            this->tail = newNode;
        }
        this->size++;
        
    }

    void addStart(int value)
    {
      
        Node *newNode = new Node(value);
        if (this->head == NULL)
        {
            this->head = newNode;
            this->tail = newNode;
        }
        else
        {
            newNode->next = this->head;
            this->head->prev = newNode;
            this->head = newNode;
        }
        this->size++;
        
    }

    void add(int value, int index)
    {
      /**
        Node *newNode = new Node(value);
        if (index == 0)
        {
            newNode->next = this->head;
            this->head->prev = newNode;
            this->head = newNode;
        }
        else if (index == this->size)
        {
            this->tail->next = newNode;
            newNode->prev = this->tail;
            this->tail = newNode;
        }
        else
        {
            Node *current = this->head;
            for (int i = 0; i < index; i++)
            {
                current = current->next;
            }
            newNode->next = current;
            newNode->prev = current->prev;
            current->prev->next = newNode;
            current->prev = newNode;
        }
        this->size++;
        */
        
    }

    bool remove(int index)
    {
      /**
        if (index < 0 || index >= this->size)
        {
            return false;
        }
        if (index == 0)
        {
            this->head = this->head->next;
            this->head->prev = NULL;
        }
        else if (index == this->size - 1)
        {
            this->tail = this->tail->prev;
            this->tail->next = NULL;
        }
        else
        {
            Node *current = this->head;
            for (int i = 0; i < index; i++)
            {
                current = current->next;
            }
            current->prev->next = current->next;
            current->next->prev = current->prev;
        }
        this->size--;
        return true;
        */
        
    }

    bool removeLast()
    {
      
        if (this->size == 0)
        {
            return false;
        }
        this->tail = this->tail->prev;
        this->tail->next = NULL;
        this->size--;
        return true;
        
    }

    int get(int index)
    {
      Serial.println("s1");
      Serial.println(index);
      
        if (index < 0 || index >= this->size)
        {
            return -1;
        }
        
        Node *current = this->head;
        for (int i = 0; i < index; i++)
        {
          if(current->next != NULL)
            current = current->next;
        }
        int x = current->value;
        Serial.println(x);
        Serial.println("e1");
        return current->value;
        
        
    }

    int getSize()
    {
        return this->size;
    }

    void sort()
    {
      /**
        Node *current = this->head;
        while (current != NULL)
        {
            sortHelper(current);
            current = current->next;
        }
        */
        
    }

    void sortHelper(Node *current)
    {
        if (current->prev == NULL)
        {
            return;
        }
        if (current->value < current->prev->value)
        {
            int temp = current->value;
            current->value = current->prev->value;
            current->prev->value = temp;
            sortHelper(current->prev);
        }
    }

    void print()
    {
      /**
        Node *current = this->head;
        while (current != NULL)
        {
            Serial.print(current->value);
            if (current->next != NULL)
            {
                Serial.print(", ");
            }
            current = current->next;
        }
        Serial.println();
        */
    }
};

#endif
