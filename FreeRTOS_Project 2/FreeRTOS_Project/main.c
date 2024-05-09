



/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"

/* MCAL includes. */
#include "gpio.h"
#include "uart0.h"
#include "ADC.h"
#include "eeprom.h"
#include "tm4c123gh6pm_registers.h"

#define AddressOfLastLevelD_Error    0
#define AddressOfLastLevelP_Error    4
#define AddressOfTimeStampsD_Error   8
#define AddressOfTimeStampsP_Error   12

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 364

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}


/* Definitions for the event bits in the event group. */
#define mainSW2_INTERRUPT_BIT ( 1UL << 0UL )  /* Event bit 0, which is set by a SW2 Interrupt. */
#define mainSW1_INTERRUPT_BIT ( 1UL << 1UL )  /* Event bit 0, which is set by a SW1 Interrupt. */
#define RUNTIME_MEASUREMENTS_TASK_PERIODICITY (1000U)

uint32 ullTasksOutTime[13];
uint32 ullTasksInTime[13];
uint32 ullTasksExecutionTime[13];

#define Enable_Exceptions()    __asm(" CPSIE I ")
#define Disable_Exceptions()   __asm(" CPSID I ")

uint8 SVDCommand = 0 ;
uint8 SVPCommand = 0 ;

/* FreeRTOS Queues */
QueueHandle_t Command_HC_D;
QueueHandle_t Command_HC_P;
QueueHandle_t Display_D;
QueueHandle_t Display_P;
/*Events*/
EventGroupHandle_t eventTempSet;
/*Temp Required State */
uint8 Dstate =0 ;
uint8 Pstate =0 ;
float32 CurrentTargetD ;
float32 CurrentTargetP ;

float32 SuprevisedTempD;
float32 SuprevisedTempP;

/* FreeRTOS Mutexes */
//xSemaphoreHandle SVDCommandMutex ;
//xSemaphoreHandle SVPCommandMutex ;
xSemaphoreHandle SuprevisedTempDMutex;
xSemaphoreHandle SuprevisedTempPMutex;
xSemaphoreHandle CurrentTargetDMutex;
xSemaphoreHandle CurrentTargetPMutex;
xSemaphoreHandle UARTMutex;

/* Resource lock time per task for each resource.*/
TickType_t  SuprevisedTempDReadingTaskLT ;
TickType_t  SuprevisedTempDSupervisionTaskLT ;
TickType_t  SuprevisedTempPReadingTaskLT ;
TickType_t  SuprevisedTempPSupervisionTaskLT ;


TickType_t  CurrentTargetDSupervisionTaskLT ;
TickType_t  CurrentTargetDHeating_CoolingTaskLT ;
TickType_t  CurrentTargetPSupervisionTaskLT ;
TickType_t  CurrentTargetPHeating_CoolingTaskLT ;


TickType_t UARTDisplayTaskDLT ;
TickType_t UARTDisplayTaskPLT ;

TickType_t UARTOneTimeRunTimeMeasurementsTaskLT ;
TickType_t UARTRunTimeMeasurementsTaskLT ;

/* The HW setup function */
static void prvSetupHardware( void );

/* FreeRTOS tasks */
void vTemperatureReadingTask(void *pvParameters);
/*Description: Reads temperature data from sensors placed inside the system.
 * Priority: Medium, as accurate temperature data is crucial for control decisions.
 * Dependencies: None.
 * Communication: Shares temperature data with the Control Task.
*/

void vTemperatureSettingTask(void *pvParameters);
/* Description: establishes the desired temperature within the system.
 * Priority: High, as the target value is the base for the system operation.
 * Dependencies: None.
 * Communication: Shares temperature target value with the Control Task.
 *
 */
void vSupervisorTask(void *pvParameters);
/*Description: Determines actions based on the target temperature and current temperature, controlling the heating or cooling systems.
 *Priority: Medium, as it directly influences the system environment.
 *Dependencies: Depends on Temperature Reading and Target Temperature Setting Tasks for input.
 *Communication: Sends commands to the Heating/Cooling Task. */

void vHeating_CoolingTask(void *pvParameters);
/*Heating/Cooling Task:
 *Description: Executes commands from the Control Task to activate heating or cooling systems.
 *Priority: Medium, as it directly impacts the system temperature.
 *Dependencies: Depends on the Control Task.
 *Communication: None.
 * */
void vDisplayTask(void *pvParameters);
/*Display Task:
 *Description: Manages the display interface showing current temperature and system status. Priority: Low, as it's not critical for real-time control.
 *Dependencies: Depends on Commands from ControlTask.
 *Communication: None.*/
void vRunTimeMeasurements (void *pvParameters) ;
/**/
TaskHandle_t vTemperatureSettingTaskForDSeatHandle ;
TaskHandle_t vTemperatureSettingTaskForPSeatHandle ;
TaskHandle_t vTemperatureReadingTaskForDSeatHandle ;
TaskHandle_t vTemperatureReadingTaskForPSeatHandle ;
TaskHandle_t vHeating_CoolingTaskForDSeatHandle    ;
TaskHandle_t vHeating_CoolingTaskForPSeatHandle    ;
TaskHandle_t vDisplayTaskDHandle                   ;
TaskHandle_t vDisplayTaskPHandle                   ;
TaskHandle_t vSupervisorTaskDHandle                ;
TaskHandle_t vSupervisorTaskPHandle                ;
TaskHandle_t vRunTimeMeasurementsTaskHandle        ;

/*Display Task Variables*/
    uint8 levelD;
    uint8 levelP;
    uint8 * heaterStateD ;
    uint8 * heaterStateP ;

int main(void)
{
    NVIC_EN1_R |= (1 << (19));
    NVIC_EN0_R |= (1 << 17);
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    xTaskCreate(vRunTimeMeasurements, "vRunTimeMeasurementsTask", 64, NULL, 1, &vRunTimeMeasurementsTaskHandle);
    vTaskSetApplicationTaskTag( vRunTimeMeasurementsTaskHandle, ( void * ) 1 );

   void* x= ((void*)'D') ;
    /* Create Tasks here */
    xTaskCreate(vTemperatureSettingTask, /* Pointer to the function that implements the task. */
                    "vTemperatureSettingTaskForDSeat",       /* Text name for the task.  This is to facilitate debugging only. */
                    64,            /* Stack depth - most small microcontrollers will use much less stack than this. */
                    x,           /* We are not passing a task parameter in this example. */
                    (configMAX_PRIORITIES-1),              /* This task will run at priority 3. */
                    &vTemperatureSettingTaskForDSeatHandle);          /* We are not using the task handle. */

    vTaskSetApplicationTaskTag( vTemperatureSettingTaskForDSeatHandle, ( void * ) 2 );


//    xTaskCreate(vTemperatureSettingTask,
//                "vTemperatureSettingTaskForPSeat"
//                , 64,
//                ((void*)'P'),
//                (configMAX_PRIORITIES-2),
//                &vTemperatureSettingTaskForPSeatHandle);
//    vTaskSetApplicationTaskTag( vTemperatureSettingTaskForPSeatHandle, ( void * ) 3 );



    xTaskCreate(vTemperatureReadingTask,
                "vTemperatureReadingTaskForDSeat"
                , 64,
                ((void*)'D'),
                (configMAX_PRIORITIES-2),
                &vTemperatureReadingTaskForDSeatHandle);

    vTaskSetApplicationTaskTag( vTemperatureReadingTaskForDSeatHandle, ( void * ) 4 );


    xTaskCreate(vTemperatureReadingTask,
                "vTemperatureReadingTaskForPSeat"
                , 64,
                ((void*)'P'),
                (configMAX_PRIORITIES-2),
                &vTemperatureReadingTaskForPSeatHandle);

    vTaskSetApplicationTaskTag( vTemperatureReadingTaskForPSeatHandle, ( void * ) 5 );


    xTaskCreate(vHeating_CoolingTask,
                "vHeating_CoolingTaskForDSeat"
                , 64,
                ((void*)'D'),
                (configMAX_PRIORITIES-2),
                &vHeating_CoolingTaskForDSeatHandle);

    vTaskSetApplicationTaskTag( vHeating_CoolingTaskForDSeatHandle, ( void * ) 6 );

    xTaskCreate(vHeating_CoolingTask,
                "vHeating_CoolingTaskForPSeat"
                , 64,
                ((void*)'P'),
                (configMAX_PRIORITIES-2),
                &vHeating_CoolingTaskForPSeatHandle);

    vTaskSetApplicationTaskTag( vHeating_CoolingTaskForPSeatHandle, ( void * ) 7 );


    xTaskCreate(vDisplayTask,
                "vDisplayTaskD"
                , 64,
                ((void*)'D'),
                1,
                &vDisplayTaskDHandle);

    vTaskSetApplicationTaskTag( vDisplayTaskDHandle, ( void * ) 8 );


    xTaskCreate(vDisplayTask,
                "vDisplayTaskP"
                , 64,
                ((void*)'P'),
                1,
                &vDisplayTaskPHandle);

    vTaskSetApplicationTaskTag( vDisplayTaskPHandle, ( void * ) 9 );

    xTaskCreate(vSupervisorTask,
                "vSupervisorTaskD"
                , 64,
                ((void*)'D'),
                1,
                &vSupervisorTaskDHandle);

    vTaskSetApplicationTaskTag( vSupervisorTaskDHandle, ( void * ) 10 );


    xTaskCreate(vSupervisorTask,
                "vSupervisorTaskP"
                , 64,
                ((void*)'P'),
                1,
                &vSupervisorTaskPHandle);

    vTaskSetApplicationTaskTag( vSupervisorTaskPHandle, ( void * ) 11 );



    SuprevisedTempDMutex=xSemaphoreCreateMutex();
    SuprevisedTempPMutex=xSemaphoreCreateMutex();
    CurrentTargetDMutex =xSemaphoreCreateMutex();
    CurrentTargetPMutex =xSemaphoreCreateMutex();
    UARTMutex           =xSemaphoreCreateMutex();
//    SVDCommandMutex     =xSemaphoreCreateMutex();
//    SVPCommandMutex     =xSemaphoreCreateMutex();


    Command_HC_D = xQueueCreate(1, sizeof(uint8));
    Command_HC_P = xQueueCreate(1, sizeof(uint8));
    Display_D    = xQueueCreate(1, sizeof(uint8));
    Display_P    = xQueueCreate(1, sizeof(uint8));


    eventTempSet=xEventGroupCreate();





    /* Now all the tasks have been started - start the scheduler.

    NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
    called.  The demo applications included in the FreeRTOS.org download switch
    to supervisor mode prior to main being called.  If you are not using one of
    these demo application projects then ensure Supervisor mode is used here. */
    Enable_Exceptions();
    vTaskStartScheduler();


    /* Should never reach here!  If you do then there was not enough heap
    available for the idle task to be created. */
    for (;;);

}


static void prvSetupHardware( void )
{

    /* Place here any needed HW initialization such as GPIO, UART, etc.  */
    GPIO_BuiltinButtonsLedsInit();/*->Note that for the purpose of testing only the heater level shall control the LED :
                                    i. Green color indicates low intensity.
                                    ii. Blue color indicates medium intensity.
                                    iii. Cyan color indicates high intensity.
                                    ->2 buttons in the car's middle console where each button is used to control one of the two seat
                                    heaters.*/
    GPIO_SW1EdgeTriggeredInterruptInit ();
    GPIO_SW2EdgeTriggeredInterruptInit ();


    ADC_INIT();
    /*The temperature sensor shall be connected to the ADC so that the current temperature is measured
                correctly*/
//    ADC_ModuleInitialize(0);
//    ADC_ModuleInitialize(1) ;
//    ADC_PinInitialize(0);
//    ADC_PinInitialize(1);

    GPTM_WTimer0Init();
    GPIO_ExtraExternalButtonAndLedsInit();
    GPIO_ExtraExternalButtonTriggeredInterruptInit();
    EEPROM_Init();
    UART0_Init();/*The current temperature, the heating level, and the heater state should be displayed on the screen by
  sending it through the UART.*/
}

void vTemperatureReadingTask(void *pvParameters)
{
    /*Description: Reads temperature data from sensors placed inside the system.
     * Priority: Medium, as accurate temperature data is crucial for control decisions.
     * Dependencies: None.
     * Communication: Shares temperature data with the Control Task.
     */
    uint8 DorP =((uint8)pvParameters) ; /*decides whether it is Driver's seat or passenger's seat*/
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    for (;;)
    {
        if(  DorP=='D' ){
            float32 Reading = ((float32)   ADC0_readChannel()    )/(4095);
            /*ADC0 is for DRIVER SEAT*/
            Reading = Reading *45 ;

            TickType_t xStartTime, xEndTime;
            if (xSemaphoreTake(SuprevisedTempDMutex, portMAX_DELAY) == pdTRUE) {
                xStartTime = xTaskGetTickCount();
                SuprevisedTempD=Reading;

                /* Release the peripheral */
                xSemaphoreGive(SuprevisedTempDMutex);
            }
            xEndTime = xTaskGetTickCount();
            SuprevisedTempDReadingTaskLT = xEndTime - xStartTime;/*Resource lock time*/




            xQueueSend(Display_D,&Reading, portMAX_DELAY) ;

        }else{
            float32 Reading = ((float32)   ADC1_readChannel()    )/(4095);
                        /*ADC0 is for DRIVER SEAT*/
            Reading = Reading *45 ;
            TickType_t xStartTime, xEndTime;
            if (xSemaphoreTake(SuprevisedTempPMutex, portMAX_DELAY) == pdTRUE) {
            xStartTime = xTaskGetTickCount();
                SuprevisedTempP=Reading;

                /* Release the peripheral */
                xSemaphoreGive(SuprevisedTempPMutex);
            }
            xEndTime = xTaskGetTickCount();
            SuprevisedTempPReadingTaskLT = xEndTime - xStartTime;/*Resource lock time*/

            xQueueSend(Display_P,&Reading, portMAX_DELAY) ;


        }
        vTaskDelay(xDelay);
    }
}

void vTemperatureSettingTask(void *pvParameters)
{
    uint32 debugdstate ;
    uint32 debugpstate ;
    EventBits_t xEventGroupValue;

    const EventBits_t xBitsToWaitFor = ( mainSW1_INTERRUPT_BIT | mainSW2_INTERRUPT_BIT);
    for (;;)
    {
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( eventTempSet,    /* The event group to read. */
                                                xBitsToWaitFor, /* Bits to test. */
                                                pdTRUE,         /* Clear bits on exit if the unblock condition is met. */
                                                pdFALSE,         /* Dont't Wait for all bits. */
                                                portMAX_DELAY); /* Don't time out. */
        debugdstate=Dstate;
        debugpstate=Pstate;
        if ((xEventGroupValue & mainSW2_INTERRUPT_BIT) != 0)
                {

//            if (xSemaphoreTake(CurrentTargetPMutex, portMAX_DELAY) == pdTRUE) {
            if(Pstate==0){


            }else if(Pstate==1){
                CurrentTargetP=25 ;

            }else if (Pstate==2){
                CurrentTargetP=30 ;

            }else if (Pstate==3){
                CurrentTargetP=35 ;

            }
//            xSemaphoreGive(CurrentTargetPMutex);
//        }

                    /*Passenger*/
//                uint8 DorP = 'P' ;
//                if (xSemaphoreTake(SVPCommandMutex, portMAX_DELAY) == pdTRUE) {
//                    SVPCommand=0;
//
//                    /* Release the peripheral */
//                    xSemaphoreGive(SVPCommandMutex);
//                }



           //     xQueueSend(Command_HC_P,&Pstate, portMAX_DELAY)  ;

                }
                /* In case PF4 edge triggered interrupt occurred, it will set event 1 bit */
        if ((xEventGroupValue & mainSW1_INTERRUPT_BIT) != 0)
        {
                /*Driver*/
//                    uint8 DorP = 'D' ;
//                    if (xSemaphoreTake(SVDCommandMutex, portMAX_DELAY) == pdTRUE) {
//                        SVDCommand=0;
//
//                        /* Release the peripheral */
//                        xSemaphoreGive(SVDCommandMutex);
//                    }

             //       xQueueSend(Command_HC_D,&Dstate, portMAX_DELAY)  ;

       //     if (xSemaphoreTake(CurrentTargetDMutex, portMAX_DELAY) == pdTRUE) {
            if(Dstate==0){

            }else if(Dstate==1){
                CurrentTargetD=25 ;
            }else if (Dstate==2){
                CurrentTargetD=30 ;
            }else if (Dstate==3){
                CurrentTargetD=35 ;
            }
         //   xSemaphoreGive(CurrentTargetDMutex);
           // }


        }

    }
}
void GPIOPortF_Handler(void)
{
    Disable_Exceptions();

    //GPIO_RedLedToggle();

    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    if(GPIO_PORTF_RIS_REG & (1<<0))           /* PF0 handler code for the DRIVER SEAT  */
    {
        Delay_MS(10);
        if(GPIO_PORTF_RIS_REG & (1<<0))     {
        xEventGroupSetBitsFromISR(eventTempSet, mainSW1_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        Dstate=(Dstate+1)%4;

        GPIO_PORTF_ICR_REG   |= (1<<0);
        }
        while(GPIO_PORTF_RIS_REG & (1<<0)) ;/* Clear Trigger flag for PF0 (Interrupt Flag) */
    }
    else if(GPIO_PORTF_RIS_REG & (1<<4))      /* PF4 handler code for the PASSENGER SEAT */
    {
        Delay_MS(10);
        if(GPIO_PORTF_RIS_REG & (1<<4)){
        xEventGroupSetBitsFromISR(eventTempSet, mainSW2_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        Pstate=(Pstate+1)%4;
        GPIO_PORTF_ICR_REG   |= (1<<4);

        }/* Clear Trigger flag for PF4 (Interrupt Flag) */
        while(GPIO_PORTF_RIS_REG & (1<<4)) ;
    }
    //Delay_MS(30) ;
    Enable_Exceptions() ;

}
void GPIOPortB_Handler(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    if(GPIO_PORTB_RIS_REG & (1<<0))           /* PF0 handler code for the DRIVER SEAT  */
    {
        xEventGroupSetBitsFromISR(eventTempSet, mainSW1_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        Dstate=(Dstate+1)%4;
        GPIO_PORTB_ICR_REG   |= (1<<0);       /* Clear Trigger flag for PF0 (Interrupt Flag) */
    }


}

void vSupervisorTask(void *pvParameters){

    for(;;){
    if( ((uint8)pvParameters) == 'D' ){
        float32 Reading , LevelRequired;
        TickType_t xStartTime, xEndTime;
        if (xSemaphoreTake(SuprevisedTempDMutex, portMAX_DELAY) == pdTRUE) {
        xStartTime = xTaskGetTickCount();
            Reading=SuprevisedTempD ;

            /* Release the peripheral */
            xSemaphoreGive(SuprevisedTempDMutex);
        }
        xEndTime = xTaskGetTickCount();
        SuprevisedTempDSupervisionTaskLT=xEndTime - xStartTime;

        if (xSemaphoreTake(CurrentTargetDMutex, portMAX_DELAY) == pdTRUE) {
        xStartTime = xTaskGetTickCount();
            LevelRequired=CurrentTargetD ;// LevelRequired here is in degrees

            /* Release the peripheral */
            xSemaphoreGive(CurrentTargetDMutex);
        }

        xEndTime                        = xTaskGetTickCount() ;
        CurrentTargetDSupervisionTaskLT =xEndTime - xStartTime;
        uint8 command ;
//        if (xSemaphoreTake(SVDCommandMutex, portMAX_DELAY) == pdTRUE) {
//            SVDCommand=1;
//
//            /* Release the peripheral */
//            xSemaphoreGive(SVDCommandMutex);
//        }

        if( (Reading<5 || Reading > 40 ) && LevelRequired!=0){
                    command='E';
                    EEPROM_Write(AddressOfLastLevelD_Error,(uint32)Dstate)        ;
                    EEPROM_Write(AddressOfTimeStampsD_Error, xTaskGetTickCount()) ;
                    uint32 rrrrrr=EEPROM_Read(AddressOfLastLevelD_Error);
                    if(rrrrrr==Dstate){
                        GPIO_RedLedOn();
                        GPIO_BlueLedOn();
                    }
                    xQueueSend(Command_HC_D,&command, portMAX_DELAY) ;
                }
        else if((LevelRequired-Reading)>=10    && LevelRequired!=0  ){
            command=3;
            xQueueSend(Command_HC_D,&command, portMAX_DELAY) ;

        }else if ((LevelRequired-Reading)>=5 && (LevelRequired-Reading)<10       && LevelRequired!=0   ){
            command=2;
            xQueueSend(Command_HC_D,&command, portMAX_DELAY) ;
        }else if((LevelRequired-Reading)>=2 && (LevelRequired-Reading)<5   && LevelRequired!=0 ){
            command=1;
            xQueueSend(Command_HC_D,&command, portMAX_DELAY) ;
        }else if ( (Reading-LevelRequired)>=0   ){
            command=0;
            xQueueSend(Command_HC_D,&command, portMAX_DELAY) ;
        }


    }else{
        float32 Reading,LevelRequired ;
        TickType_t xStartTime, xEndTime;
        if (xSemaphoreTake(SuprevisedTempPMutex, portMAX_DELAY) == pdTRUE) {
        xStartTime = xTaskGetTickCount();
            Reading=SuprevisedTempP ;

            /* Release the peripheral */
            xSemaphoreGive(SuprevisedTempPMutex);
        }

        xEndTime = xTaskGetTickCount();
        SuprevisedTempPSupervisionTaskLT=xEndTime - xStartTime;

        if (xSemaphoreTake(CurrentTargetPMutex, portMAX_DELAY) == pdTRUE) {
        xStartTime = xTaskGetTickCount();
            LevelRequired=CurrentTargetP ;

            /* Release the peripheral */
            xSemaphoreGive(CurrentTargetPMutex);
        }
        xEndTime                        = xTaskGetTickCount() ;
        CurrentTargetPSupervisionTaskLT =xEndTime - xStartTime;
        uint8 command ;
//        if (xSemaphoreTake(SVPCommandMutex, portMAX_DELAY) == pdTRUE) {
//            SVPCommand=1;
//
//            /* Release the peripheral */
//            xSemaphoreGive(SVPCommandMutex);
//        }


        if( (Reading<5 || Reading > 40 ) && LevelRequired!=0){
                    command='E';
                    EEPROM_Write(AddressOfLastLevelP_Error,(uint32)Pstate) ;
                    EEPROM_Write(AddressOfTimeStampsP_Error, xTaskGetTickCount()) ;
                    xQueueSend(Command_HC_P,&command, portMAX_DELAY) ;
                }else if((LevelRequired-Reading)>=10 && LevelRequired!=0  ){
            command=3;
            xQueueSend(Command_HC_P,&command, portMAX_DELAY) ;

        }else if ((LevelRequired-Reading)>=5 && (LevelRequired-Reading)<10 && LevelRequired!=0  ){
            command=2;
            xQueueSend(Command_HC_P,&command, portMAX_DELAY) ;
        }else if((LevelRequired-Reading)>=2 && (LevelRequired-Reading)<5     && LevelRequired!=0){
            command=1;
            xQueueSend(Command_HC_P,&command, portMAX_DELAY) ;
        }else if ( (Reading-LevelRequired)>=0 ){
            command=0;
            xQueueSend(Command_HC_P,&command, portMAX_DELAY) ;
        }



    }

    const TickType_t xDelay = pdMS_TO_TICKS(500);
    vTaskDelay(xDelay);
    }


}

void vHeating_CoolingTask(void *pvParameters){
            uint8 LevelRequired =0;
    for(;;){
        if( ((uint8)pvParameters) == 'D' ){

            if (xQueueReceive(Command_HC_D, &LevelRequired, portMAX_DELAY) == pdTRUE) {
//                TickType_t xStartTime, xEndTime;
//                if (xSemaphoreTake(CurrentTargetDMutex, portMAX_DELAY) == pdTRUE) {
//                xStartTime = xTaskGetTickCount();
//                    CurrentTargetD=LevelRequired ;



                    if(LevelRequired==3)
                    {
//                        if (xSemaphoreTake(SVDCommandMutex, portMAX_DELAY) == pdTRUE) {
//                           if(SVDCommand==0)
//                                CurrentTargetD=35 ;
//                            /* Release the peripheral */
//                            xSemaphoreGive(SVDCommandMutex);
//                        }

                        heaterStateD="HIGH" ;
                        GPIO_BlueLedOn();
                        GPIO_GreenLedOn();
                        GPIO_RedLedOff();
                    }else if(LevelRequired==2){
//                        if (xSemaphoreTake(SVDCommandMutex, portMAX_DELAY) == pdTRUE) {
//                            if(SVDCommand==0)
//                                CurrentTargetD=30 ;
//                            /* Release the peripheral */
//                            xSemaphoreGive(SVDCommandMutex);
//                        }


                        heaterStateD="Medium" ;
                        GPIO_RedLedOff();
                        GPIO_GreenLedOff();
                        GPIO_BlueLedOn();
                    }else if(LevelRequired==1){

//                        if (xSemaphoreTake(SVDCommandMutex, portMAX_DELAY) == pdTRUE) {
//                            if(SVDCommand==0)
//                                CurrentTargetD=25 ;
//                            /* Release the peripheral */
//                            xSemaphoreGive(SVDCommandMutex);
//                        }
                        heaterStateD="LOW" ;
                        GPIO_RedLedOff();
                        GPIO_GreenLedOn();
                        GPIO_BlueLedOff();

                    }
                    else if(LevelRequired=='E'){
                        heaterStateD="ERROR" ;
                        GPIO_GreenLedOff();
                        GPIO_BlueLedOff();
                        GPIO_RedLedOn();
                    }
                    else if (LevelRequired==0){
                        heaterStateD="OFF" ;
                        GPIO_GreenLedOff();
                        GPIO_BlueLedOff();
                        GPIO_RedLedOff();
                    }
                    /* Release the peripheral */
//                    xSemaphoreGive(CurrentTargetDMutex);
//                }
//                xEndTime = xTaskGetTickCount();
//                CurrentTargetDHeating_CoolingTaskLT=xStartTime-xEndTime ;

                }
        }else if(  ((uint8)pvParameters) == 'P'  ){



            if (xQueueReceive(Command_HC_P, &LevelRequired, portMAX_DELAY) == pdTRUE) {
//                TickType_t xStartTime, xEndTime;
//                if (xSemaphoreTake(CurrentTargetPMutex, portMAX_DELAY) == pdTRUE) {
//                xStartTime=xTaskGetTickCount();
//                    CurrentTargetP=LevelRequired ;

                    /* Release the peripheral */


                    if(LevelRequired==3){
//                        if (xSemaphoreTake(SVPCommandMutex, portMAX_DELAY) == pdTRUE) {
//                            if(SVPCommand==0)
//                                CurrentTargetP=35 ;
//                            /* Release the peripheral */
//                            xSemaphoreGive(SVPCommandMutex);
//                        }
                        heaterStateP="HIGH" ;
                        ExternalGPIO_BlueLedOn();
                        ExternalGPIO_GreenLedOn();
                        ExternalGPIO_RedLedOff();
                    }else if(LevelRequired==2){
//                        if (xSemaphoreTake(SVDCommandMutex, portMAX_DELAY) == pdTRUE) {
//                            if(SVDCommand==0)
//                                CurrentTargetD=30 ;
//                            /* Release the peripheral */
//                            xSemaphoreGive(SVDCommandMutex);
//                        }
                        heaterStateP="MEDIUM" ;
                        ExternalGPIO_RedLedOff();
                        ExternalGPIO_GreenLedOff();
                        ExternalGPIO_BlueLedOn();
                    }else if(LevelRequired==1){
//                        if (xSemaphoreTake(SVDCommandMutex, portMAX_DELAY) == pdTRUE) {
//                            if(SVDCommand==0)
//                                CurrentTargetD=25 ;
//                            /* Release the peripheral */
//                            xSemaphoreGive(SVDCommandMutex);
//                        }
                        heaterStateP="LOW" ;
                        ExternalGPIO_RedLedOff();
                        ExternalGPIO_GreenLedOn();
                        ExternalGPIO_BlueLedOff();

             }
                        else if(LevelRequired=='E'){
                        heaterStateP="ERROR" ;
                        ExternalGPIO_GreenLedOff();
                        ExternalGPIO_BlueLedOff();
                        ExternalGPIO_RedLedOn();
                    }
                    else if (LevelRequired==0){
                        heaterStateP="OFF" ;
                        ExternalGPIO_GreenLedOff();
                        ExternalGPIO_BlueLedOff();
                        ExternalGPIO_RedLedOff();
                    }
                    /* Release the peripheral */
//                    xSemaphoreGive(CurrentTargetPMutex);
//             }
//                xEndTime = xTaskGetTickCount();
//                CurrentTargetPHeating_CoolingTaskLT=xStartTime-xEndTime ;

           }

        }
    }
}
void vDisplayTask(void *pvParameters){
    for(;;){
        const TickType_t xDelay = pdMS_TO_TICKS(1000);
        vTaskDelay(xDelay);
        if( ((uint8)pvParameters) == 'D' ){
        if(xQueueReceive(Display_D, &levelD, portMAX_DELAY) == pdTRUE ){



            TickType_t xStartTime, xEndTime;
            if (xSemaphoreTake(UARTMutex, portMAX_DELAY) == pdTRUE) {
            xStartTime = xTaskGetTickCount();
                UART0_SendString("\nTemp of Driver Seat is : ") ;
                UART0_SendByte(levelD);
                UART0_SendString("\nTemp of Passenger Seat is : ") ;
                UART0_SendByte(levelP);
                UART0_SendString("\nHeating level is ");
                UART0_SendByte((uint32)CurrentTargetD);
                UART0_SendString("\nHeating level is ");
                UART0_SendByte((uint32)CurrentTargetP);
                UART0_SendString("\nHeating State ");
                UART0_SendString(heaterStateD);
                UART0_SendString("\nHeating State ");
                UART0_SendString(heaterStateP);
                /* Release the peripheral */
                xSemaphoreGive(UARTMutex);
            }
            xEndTime = xTaskGetTickCount();
            UARTDisplayTaskDLT=xEndTime - xStartTime;

        }

        }else{
            if(xQueueReceive(Display_P, &levelP, portMAX_DELAY) == pdTRUE ){
                TickType_t xStartTime, xEndTime;

                if (xSemaphoreTake(UARTMutex, portMAX_DELAY) == pdTRUE) {
                    xStartTime = xTaskGetTickCount();
                    UART0_SendString("\nTemp of Driver Seat is : ") ;
                    UART0_SendByte(levelD);
                    UART0_SendString("\nTemp of Passenger Seat is : ") ;
                    UART0_SendByte(levelP);
                    UART0_SendString("\nHeating level is ");
                    UART0_SendByte((uint32)CurrentTargetD);
                    UART0_SendString("\nHeating level is ");
                    UART0_SendByte((uint32)CurrentTargetP);
                    UART0_SendString("\nHeating State ");
                    UART0_SendString(heaterStateD);
                    UART0_SendString("\nHeating State ");
                    UART0_SendString(heaterStateP);
                    /* Release the peripheral */
                    xSemaphoreGive(UARTMutex);
                }
                xEndTime = xTaskGetTickCount();
                UARTDisplayTaskPLT=xEndTime - xStartTime;

            }


            }
}

    }

/*
TaskHandle_t vSupervisorTaskDHandle                ;
TaskHandle_t vSupervisorTaskPHandle                ;*/
void vRunTimeMeasurements (void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000);
    vTaskDelay(xDelay);
    TickType_t xStartTime, xEndTime;
    if (xSemaphoreTake(UARTMutex, portMAX_DELAY) == pdTRUE) {
    xStartTime = xTaskGetTickCount();

//        UART0_SendString("\nTemperatureSettingTaskForDSeat execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[2] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nTemperatureSettingTaskForPSeat execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[3] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nTemperatureReadingTaskForDSeat execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[4] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nTemperatureReadingTaskForPSeat execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[5] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nHeating_CoolingTaskForDSeat execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[6] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nHeating_CoolingTaskForPSeat execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[7] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nDisplayTaskD execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[8] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nDisplayTaskP execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[9] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nSupervisorTaskD execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[10] / 10);
//        UART0_SendString(" msec \r\n");
//
//        UART0_SendString("\nSupervisorTaskP execution time is ");
//        UART0_SendInteger(ullTasksExecutionTime[11] / 10);
//        UART0_SendString(" msec \r\n");
//
//

        /* Release the peripheral */
        xSemaphoreGive(UARTMutex);


    }
    xEndTime = xTaskGetTickCount();
    UARTOneTimeRunTimeMeasurementsTaskLT=xEndTime - xStartTime;


    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
        {
            uint8 ucCounter, ucCPU_Load;
            uint32 ullTotalTasksTime = 0;
            vTaskDelayUntil(&xLastWakeTime, RUNTIME_MEASUREMENTS_TASK_PERIODICITY);
            for(ucCounter = 1; ucCounter <= 11; ucCounter++)
            {
                ullTotalTasksTime += ullTasksExecutionTime[ucCounter];
            }
            ucCPU_Load = (ullTotalTasksTime * 100) /  GPTM_WTimer0Read();
            TickType_t xStartTime, xEndTime;
            if (xSemaphoreTake(UARTMutex, portMAX_DELAY) == pdTRUE) {
            xStartTime = xTaskGetTickCount();
//                UART0_SendString("CPU Load is ");
//                UART0_SendInteger(ucCPU_Load);
//                UART0_SendString("% \r\n");
                /* Release the peripheral */
                xSemaphoreGive(UARTMutex);
            }
            xEndTime = xTaskGetTickCount();
            UARTRunTimeMeasurementsTaskLT=xEndTime - xStartTime;
        }

}
/*-----------------------------------------------------------*/



