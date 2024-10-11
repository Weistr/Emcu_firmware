#include "main.h"
#include "task.h"

void main()
{
    boardInit();

    while (1)
    {
        Task_Pro_Handler_Callback();
    }
}