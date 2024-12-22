#include "main.h"
#include "task.h"
#include "board.h"


void main()
{


    boardInit();


    while (1)
    {
        Task_Pro_Handler_Callback();
    }
}






