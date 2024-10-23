#include "msp.h"
#include "driverlib.h"

#include <stdint.h>

#include <stdbool.h>
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        19,                                     // BRDIV = 78
        8,                                       // UCxBRF = 2
        85,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

volatile uint8_t receivedBuffer = "";
volatile uint8_t request[2] = "";
volatile int entTog = 0;
volatile int numInt = 0;

int toTurn = 0;
int prompted = 0;
int startPrompt = 0;


int phasecount = 0;    // counts motor phases 0-7
int timestep = 3200;  // time delay counter limit
int direction = 0;

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    unsigned int dcoFrequency = 3E+6;
    MAP_CS_setDCOFrequency(dcoFrequency);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P6,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN1);

    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();


    int i = 0;

    int tog = 0;

    while(1)
    {
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0 && tog == 0){
            tog = 1;
            printf("here 1 \n");
        }

        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 1 && tog == 1) {
            int k = 0;
            for(k = 0; k < (15 * 68); k++) {
                phasecount = backwardStep(phasecount, GPIO_PORT_P6);
                for(i=0;i<512;i++){};
            }
            printf("here \n");
            tog = 0;
        }
    }
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        //MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));

        if (prompted == 0) {
            prompted = 1;
            char printThis[35];
            sprintf(printThis, "Enter angle Between 0 and 90:");
            int j = 0;
            while(j < 35){
                UART_transmitData(EUSCI_A0_BASE, (uint_fast8_t) printThis[j]);
                j++;
            }
            UART_transmitData(EUSCI_A0_BASE, 13);
            UART_transmitData(EUSCI_A0_BASE, 10);
        }

        if(numInt < 2 && UART_receiveData(EUSCI_A0_BASE) != 13) {
            receivedBuffer = MAP_UART_receiveData(EUSCI_A0_BASE);
            UART_transmitData(EUSCI_A0_BASE, receivedBuffer);
            request[numInt] = UART_receiveData(EUSCI_A0_BASE);
            numInt++;
        }

        int entCheck = 13;

        if(UART_receiveData(EUSCI_A0_BASE) == 13) {
            UART_transmitData(EUSCI_A0_BASE, 13);
            UART_transmitData(EUSCI_A0_BASE, 10);
            if (numInt == 1) {
                toTurn = ((int) request[0]) - 48;
            }
            if (numInt == 2) {
                int tens = (((int) request[0]) - 48) * 10;
                toTurn = ((int) request[1]) - 48 + tens;
            }
            numInt = 0;
            request[0] = 0;
            request[1] = 0;
            int i = 0;
            int k = 0;
            for(k = 0; k < (toTurn * 68); k++) {
                phasecount = backwardStep(phasecount, GPIO_PORT_P2);
                for(i=0;i<512;i++){};
            }
            prompted = 0;
        }
    }
}

int forwardStep(int phasecount, uint8_t port){
    if(phasecount==0){
        GPIO_setOutputHighOnPin(port,GPIO_PIN4|GPIO_PIN5);
        GPIO_setOutputLowOnPin(port,GPIO_PIN6|GPIO_PIN7);
    }
    else if(phasecount==1){
        GPIO_setOutputHighOnPin(port,GPIO_PIN5);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN7|GPIO_PIN6);
    }
    else if(phasecount==2){
        GPIO_setOutputHighOnPin(port,GPIO_PIN5|GPIO_PIN6);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN7);
    }
    else if(phasecount==3){
        GPIO_setOutputHighOnPin(port,GPIO_PIN6);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN5|GPIO_PIN7);
    }
    else if(phasecount==4){
        GPIO_setOutputHighOnPin(port,GPIO_PIN7|GPIO_PIN6);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN5);
    }
    else if(phasecount==5){
        GPIO_setOutputHighOnPin(port,GPIO_PIN7);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6);
    }
    else if(phasecount==6){
        GPIO_setOutputHighOnPin(port,GPIO_PIN4|GPIO_PIN7);
        GPIO_setOutputLowOnPin(port,GPIO_PIN6|GPIO_PIN5);
    }
    else if(phasecount==7){
        GPIO_setOutputHighOnPin(port,GPIO_PIN4);
        GPIO_setOutputLowOnPin(port,GPIO_PIN6|GPIO_PIN5|GPIO_PIN7);
    }

    if(phasecount==0){
        phasecount = 7;
    }
    else{
        phasecount--;
    }
    return phasecount;
}


int backwardStep(int phasecount, uint8_t port){
    if(phasecount==0){
        GPIO_setOutputHighOnPin(port,GPIO_PIN4|GPIO_PIN5);
        GPIO_setOutputLowOnPin(port,GPIO_PIN6|GPIO_PIN7);
    }
    else if(phasecount==1){
        GPIO_setOutputHighOnPin(port,GPIO_PIN5);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN7|GPIO_PIN6);
    }
    else if(phasecount==2){
        GPIO_setOutputHighOnPin(port,GPIO_PIN5|GPIO_PIN6);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN7);
    }
    else if(phasecount==3){
        GPIO_setOutputHighOnPin(port,GPIO_PIN6);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN5|GPIO_PIN7);
    }
    else if(phasecount==4){
        GPIO_setOutputHighOnPin(port,GPIO_PIN7|GPIO_PIN6);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN5);
    }
    else if(phasecount==5){
        GPIO_setOutputHighOnPin(port,GPIO_PIN7);
        GPIO_setOutputLowOnPin(port,GPIO_PIN4|GPIO_PIN5|GPIO_PIN6);
    }
    else if(phasecount==6){
        GPIO_setOutputHighOnPin(port,GPIO_PIN4|GPIO_PIN7);
        GPIO_setOutputLowOnPin(port,GPIO_PIN6|GPIO_PIN5);
    }
    else if(phasecount==7){
        GPIO_setOutputHighOnPin(port,GPIO_PIN4);
        GPIO_setOutputLowOnPin(port,GPIO_PIN6|GPIO_PIN5|GPIO_PIN7);
    }
    if(phasecount==7){
        phasecount = 0;
    }
    else{
        phasecount++;
    }
    return phasecount;
}
