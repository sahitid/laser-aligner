#include <stdint.h>
#include <stdbool.h>

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"

#include "utils/uartstdio.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"

#define LDAC_PORT     GPIO_PORTE_BASE
#define LDAC_1        GPIO_PIN_5
#define LDAC_2        GPIO_PIN_4

#define DAC_MAX       ((1 << 12) - 1)
#define DAC_SELECT    (1 << 15)
#define DAC_LOW_GAIN  (1 << 13)
#define DAC_NO_SHUT   (1 << 12)

#define TIMER_FREQ    100000

#define TIMER_INCR    1

#define DAC_SHIFT     0 /* divide by 1 */
#define PWM_CLOCK_RATE 39000000


uint32_t g_ui32DAC_val = 0;
char key_pressed = 'b';
int start_max = 0;
int repeats_max = 0;
int servo_filter = 0;
int angle = 30;
int number = 0;
int scale_1 = 1;
int scale_2 = 1;
int scale = 0;
int corrected_sensor = 0;
int hits = 0;
int filter = 1;
int sensor_first_max = 0;
int repeats = 0;
int total_sensor = 0;
int average_sensor = 0;
int dir=1;
int waiting=20;
int steps;
uint8_t step_size = 1;
int galv_val[4] = {2400, 2235, 2400, 1947};
int galv_val_max[4] = {0x800,0x800,0x800,0x800};
int sensor;
int sensor_last=0;
int sensor_max;
int x=0;
int num;
int half;
int step=2;
int galv_temp[4] = {0, 0, 0, 0};
int i=0; int j=0; int k=0; int l=0;
int delay = 4000;
int theta;
int dx;
int dy;
int dtheta;
int dphi;
int m; int n;
int size;
int recent_max;
int thresh;
int nmax;
int delaymax;
bool initialscan;
bool maxfound;
int count;

void Start_ADC(void)
{
    ADCProcessorTrigger(ADC0_BASE, 0);
}

uint32_t Read_ADC(void)
{
    uint32_t ui32Return_val = 0;

    while (!(ADCIntStatus(ADC0_BASE, 0, false) & ADC_INT_SS0)) {}
    ADCSequenceDataGet(ADC0_BASE, 0, &ui32Return_val);
    ADCIntClear(ADC0_BASE, 0);

    return ui32Return_val;
}

void Write_to_DAC(uint32_t ui32Base, uint8_t ui8Channel, uint32_t ui32Val)
{
    uint32_t ui32SSI_cmd = DAC_NO_SHUT | (DAC_MAX & ui32Val);
    if (ui8Channel > 0) ui32SSI_cmd |= DAC_SELECT;
    SSIDataPut(ui32Base, ui32SSI_cmd);
}


void Write_all_DACs_and_update(uint32_t ui32DAC1A, uint32_t ui32DAC1B, uint32_t ui32DAC2A, uint32_t ui32DAC2B)
{
    /* Write values */
    Write_to_DAC(SSI0_BASE, 0, ui32DAC1A);
    Write_to_DAC(SSI2_BASE, 0, (0xFFF-ui32DAC2A));
    Write_to_DAC(SSI0_BASE, 1, ui32DAC1B);
    Write_to_DAC(SSI2_BASE, 1, ui32DAC2B);

    /* Wait for write to complete */
    while(SSIBusy(SSI0_BASE) || SSIBusy(SSI2_BASE)) {}

    /* Drive LDAC low and then high */
    GPIOPinWrite(LDAC_PORT, LDAC_1 | LDAC_2, 0);
    GPIOPinWrite(LDAC_PORT, LDAC_1 | LDAC_2, LDAC_1 | LDAC_2);
}

/* Currently unused - there if you need it though */
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    Write_all_DACs_and_update((g_ui32DAC_val >> DAC_SHIFT),(g_ui32DAC_val >> DAC_SHIFT),(g_ui32DAC_val >> DAC_SHIFT),(g_ui32DAC_val >> DAC_SHIFT));

    /* Update DAC val */
    /*g_ui32DAC_val += TIMER_INCR;*/
}


void set_galv_temps(void){
    //UARTprintf("set_galv_temps     %d     %d     %d   %d        \r", n, i, m, j);
    galv_temp[0]=galv_val[0]+dx; galv_temp[3]=galv_val[3]+dx+dtheta/2;
    galv_temp[1]=galv_val[1]+dy; galv_temp[2]=galv_val[2]+dy+dphi/2;
}


//Main function that scans... takes the x,y positions, and pointing and writes values to galvos, checks the sensor and
//updates galv_val if there is a larger sensor value found at the new position

void check(void){

    Write_all_DACs_and_update(galv_temp[0],galv_temp[1],galv_temp[2],galv_temp[3]);
    SysCtlDelay(delay);
    Start_ADC();
    sensor = Read_ADC();
    if (sensor>sensor_max+thresh){
        SysCtlDelay(delay);
        Start_ADC();
        sensor = Read_ADC();
        if (sensor>sensor_max+thresh){
            sensor_max=sensor;
            initialscan=false;
            galv_val[0] = galv_temp[0];     galv_val[1] = galv_temp[1];     galv_val[2] = galv_temp[2];     galv_val[3] = galv_temp[3];
            n=0; m=0; i=0; j=0;
            count=1;
            repeats_max = 0;
            repeats = 0;
            UARTprintf("  New Max %04i [%04d, %04d, %04d, %04d] at %03d %03d %03d %03d with size %03d \n", sensor_max, galv_val[0], galv_val[1], galv_val[2], galv_val[3], dx, dy, dtheta, dphi, size);
            if (sensor>(4000/filter)){ size=0;
            sensor_first_max = sensor;
            repeats = 0;
            galv_val_max[0] = galv_temp[0];     galv_val_max[1] = galv_temp[1];     galv_val_max[2] = galv_temp[2];     galv_val_max[3] = galv_temp[3];

            }
        }
    }
}

//steps through different pointing values for the given position value - calls check() to check sensor for each value
void angle_scan(){
    m=0;
    while (size>0){
        m++;
        j=0;
        while (j<8*m){
            if (j<2*m) {dtheta=(m-j%(2*m))*size; dphi=m*size; set_galv_temps(); check();j++;}
            else if (j<4*m){dtheta=(m-j%(2*m))*size; dphi=-m*size; set_galv_temps(); check();j++;}
            else if (j<6*m){dtheta=m*size; dphi=(m-j%(2*m))*size; set_galv_temps(); check();j++;}
            else {dtheta=-m*size; dphi=(m-j%(2*m))*size; set_galv_temps(); check();j++;}
        }
        if (m>=n){break;}
    }
}

unsigned char check_uart(){
    if (UARTCharsAvail(UART0_BASE)){
        unsigned char typed = UARTgetc();
        return typed;
    }
    else return 0;
}


//changes x and y position values to scan in a square of n*size length with steps equal to 'size'
//new linear scan
void linear_scan()
{
    n=1;
    dy = 0;
    dx = 0;

    while (size>0)
    {
        if (n%2 != 0)
        {
            //moves right x n
            for (i = 0; i < n; i++)
            {
                dx += size;
                angle_scan();
            }


            //moves up x n
            for (i = 0; i < n; i++)
            {
                dy += size;
                angle_scan();
            }


        }
        else
        {
            //moves left x n

            for (i = 0; i < n; i++)
            {
                //moves left
                dx -= size;
                angle_scan();


            }

            //moves down x n
            for (i = 0; i < n; i++)
            {
                //moves down
                dy -= size;
                angle_scan();

            }



        }

        n++;


    }

    if (initialscan && n>18)
    {
        size=size/2;
        n=0;
        initialscan=false;
    }

    if (!initialscan && n>4)
    {
        size=size/2;
        n=0;
    }

    Write_all_DACs_and_update(galv_val[0],galv_val[1],galv_val[2],galv_val[3]);
}

//can be called at beginning of main function after setup, to check if mirrors are all moving
void test_scan(){
    for(n = 0; n<4; n+=1){
        for(i = 0; i<4096; i+=1){
            galv_val[n]=i;
            Write_all_DACs_and_update(galv_val[0],galv_val[1],galv_val[2],galv_val[3]);
            SysCtlDelay(10000);
        }
    }
}


int main(void)
{
    uint32_t ui32Sys_clock;
    uint32_t clock_rate; //for servok
    /*
     * Set the system clock to run at 80Mhz off PLL with external crystal as
     * reference.
     */
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
            SYSCTL_OSC_MAIN);

    SysCtlDelay(3);

    ui32Sys_clock = SysCtlClockGet();

    clock_rate = ui32Sys_clock/64;


    IntMasterDisable();

    /* PINS E4 = LDAC1
     *      E5 = LDAC2
     *      A2 = SPI_CLK1
     *      A3 = SPI_CS1
     *      A5 = SPI_MOSI1
     *      B4 = SPI_CLK2
     *      B5 = SPI_CS2
     *      B7 = SPI_MOSI2
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Enable PWM, use pin PB6
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);


    // Configure the PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, clock_rate/50);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 3);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);

    /* Configure ADC */
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);
    SysCtlDelay(10);
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH5 | ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE, 0);

    /* Configure GPIOA Pins */
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Configure GPIOB Pins */
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);

    /* Configure GPIOD Pins */
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    /* Configure GPIOE Pins */
    GPIOPinTypeGPIOOutput(LDAC_PORT, LDAC_1 | LDAC_2);
        /* Write the default state (high) to LDAC pins */
    GPIOPinWrite(LDAC_PORT, LDAC_1 | LDAC_2, LDAC_1 | LDAC_2);

    /* Configure SSI0 */
    SSIIntClear(SSI0_BASE, SSI_TXEOT);
    SSIConfigSetExpClk(SSI0_BASE, ui32Sys_clock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 20000000, 16);
    SSIEnable(SSI0_BASE);

    /* Configure SSI2 */
    SSIIntClear(SSI2_BASE, SSI_TXEOT);
    SSIConfigSetExpClk(SSI2_BASE, ui32Sys_clock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 20000000, 16);
    SSIEnable(SSI2_BASE);

    /* Configure Timer0 */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Sys_clock/TIMER_FREQ);

    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Configure UART0 */
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UART0_BASE, 16000000, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTStdioConfig(0, 115200, 16000000);

    /* Enable timer and interrupts */
    /* TimerEnable(TIMER0_BASE, TIMER_A); */
    IntMasterEnable();

    /* From here, all config is done */

   // test_scan();


    // move servo to position for no filter
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);
    SysCtlDelay(3000000);



    //initial value from sensor
    Write_all_DACs_and_update(galv_val[0],galv_val[1],galv_val[2],galv_val[3]);
    SysCtlDelay(delay);
    Start_ADC();
    sensor = Read_ADC();
    UARTprintf("Initial value of sensor: %d.\n", sensor);

    sensor_max=sensor;              // find the noise value and initialize sensor_max to be the larger of noise or initial reading

    initialscan=true;                             // initial scan,
    nmax=3; delaymax=8000;
    size=50; delay = 4200; thresh = 20;
    //linear_scan();

    // stops scanning if no significant light was detected, asks user to manually align beam and press key to continue scan
    i=0;
    if(sensor_max<200){
        UARTprintf(" [%d, %d, %d, %d]. \n",  galv_val[0], galv_val[1], galv_val[2], galv_val[3]);
        UARTprintf("No initial light detected\n");
        UARTprintf("manually align beam of press any key to align with keyboard\n");
        UARTprintf("e to exit and continue scanning\n");
        while(!UARTCharsAvail(UART0_BASE)){
            i++;
            SysCtlDelay(delay);
            Start_ADC();
            sensor = Read_ADC();
            total_sensor += sensor;
            average_sensor = total_sensor / i;
            //takes average of 21 sensor values to reduce noise
            if(i>20){
                UARTprintf("SENSOR = %04d \r", average_sensor);
                average_sensor = 0; total_sensor = 0; i = 0;
            }
        }


        //keyboard controlled mirror adjustment to help initial position for scan
        key_pressed = UARTgetc();
        UARTprintf("wasd to align position\n");
        UARTprintf("FH to align m0\n");
        UARTprintf("GT to align m1\n");
        UARTprintf("IK to align m2\n");
        UARTprintf("JL to align m3\n");
        UARTprintf("e to exit and continue scanning\n");

        while(key_pressed != 'e'){
            int incr_amount = 10;

            if (UARTCharsAvail(UART0_BASE)){
            key_pressed = UARTgetc();

            switch(key_pressed){
            case 'a':galv_val[0] -= incr_amount; galv_val[3] -= incr_amount;
            break;
            case 'd':galv_val[0] += incr_amount; galv_val[3] += incr_amount;
            break;
            case 'w':galv_val[1] += incr_amount; galv_val[2] += incr_amount;
            break;
            case 's':galv_val[1] -= incr_amount; galv_val[2] -= incr_amount;
            break;
            case 'j':galv_val[3] -= incr_amount;
            break;
            case 'l':galv_val[3] += incr_amount;
            break;
            case 'i':galv_val[2] += incr_amount;
            break;
            case 'k':galv_val[2] -= incr_amount;
            break;
            case 'f':galv_val[0] -= incr_amount;
            break;
            case 'h':galv_val[0] += incr_amount;
            break;
            case 'g':galv_val[1] += incr_amount;
            break;
            case 't':galv_val[1] -= incr_amount;
            break;
            case 'z':galv_val[2] = 2048; galv_val[0] = 2048; galv_val[1] = 2048; galv_val[3] = 2048;
            break;
            case 'x':galv_val[2] = 0000; galv_val[0] = 0000; galv_val[1] = 0000; galv_val[3] = 0000;
            break;
            case '0': filter = 1; scale_1 = 1; scale_2 = 1; number = 0; angle = 30; servo_filter=number;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);
            break;

            case '1': filter = 1; scale_1 = 10; scale_2 = 3; number = 1; angle = 53; servo_filter=number;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);

            break;
            case '2': filter = 1; scale_1 = 100; scale_2 = 9; number = 2; angle = 77; servo_filter=number;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);

            break;
            case '3': filter = 1; scale_1 = 1000; scale_2 = 14; number = 3; angle = 102; servo_filter=number;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);

            break;
            case '4': filter = 1; scale_1 = 1000; scale_2 = 42; number = 4; angle = 125; servo_filter=number;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);

            }

            galv_val[0] &= 0xFFF;
            galv_val[1] &= 0xFFF;
            galv_val[2] &= 0xFFF;
            galv_val[3] &= 0xFFF;



            Write_all_DACs_and_update(galv_val[0],galv_val[1],galv_val[2],galv_val[3]);
            }
            total_sensor = 0;
            for(i = 0; i < 20000; ++i) {
                Start_ADC();
                sensor = Read_ADC();
                total_sensor += sensor;
            }
            average_sensor = total_sensor/2000;
            UARTprintf("M0 %04d/M1 %04d/M2 %04d/M3 %04d/SENSOR %05d\r", galv_val[0], galv_val[1], galv_val[2], galv_val[3], average_sensor);
            UARTprintf("FILTER = %01d ," , number);
            SysCtlDelay(delay);
        }
    }

    UARTprintf("Start Continual Scan               \n");
    while(true) {

        //switch statement moves servo to higher filter values when threshold is met at current filter
        //'filter' controls the threshold, default is 4000,
        //'scale_1' is not used, but can be used to create more accurate decimal multipliers for filters
        //'scale_2' is used to find corrected intensity value based on the filter used
        //'number' is the filter being used
        //'angle' is the position of the servo


           if(sensor_first_max>10 && angle < 135){
                servo_filter++;
                switch(servo_filter){
                    case 1:filter = 1; scale_1 = 10; scale_2 = 3; number = 1; angle = 53;
                    break;
                    case 2:filter = 1; scale_1 = 100; scale_2 = 9; number = 2; angle = 77;
                    break;
                    case 3:filter = 1; scale_1 = 1000; scale_2 = 14; number = 3; angle = 102;
                    break;
                    case 4:filter = 1; scale_1 = 1000; scale_2 = 42; number = 4; angle = 125;
                }
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);
                SysCtlDelay(3000000);
                sensor_first_max = 2;
                angle += 10;
                start_max = 0;
            }

           //if max value is found at the highest filter setting
           //stops scan and shows user sensor value
           //allows user to type entry to rescan starting from a specific filter value
           //for user to make adjustments to focus with lenses and then rescan
           if(angle>135 && sensor_first_max>10){
               while(UARTCharsAvail(UART0_BASE)){
                       UARTgetc();
               }
               UARTprintf("\n\nBest position found\n");
               UARTprintf("Type filter number [0 -> 4] you would like to rescan at\n");
               while(!UARTCharsAvail(UART0_BASE)){
                       i++;
                       SysCtlDelay(delay);
                       Start_ADC();
                       sensor = Read_ADC();
                       total_sensor += sensor;
                       average_sensor = total_sensor / i;
                       if(i>20){
                           UARTprintf("SENSOR = %04d \r", average_sensor);
                           average_sensor = 0; total_sensor = 0; i = 0;
                       }
                   }
               char servo_input = UARTgetc();

               switch(servo_input){
                    case '0':filter = 1; scale_1 = 1; scale_2 = 1; number = 0; angle = 30;
                    break;
                    case '1':filter = 1; scale_1 = 10; scale_2 = 3; number = 1; angle = 53;
                    break;
                    case '2':filter = 1; scale_1 = 100; scale_2 = 9; number = 2; angle = 77;
                    break;
                    case '3':filter = 1; scale_1 = 1000; scale_2 = 14; number = 3; angle = 102;
                    break;
                    case '4':filter = 1; scale_1 = 1000; scale_2 = 42; number = 4; angle = 125;
                    break;
               }
               PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, clock_rate/50000*angle);
               SysCtlDelay(3000000);
               sensor_first_max = 2;
               servo_filter = number;
               start_max=0;
           }

           // below is the scan that runs after the servo position is determined above
            i++;
            SysCtlDelay(delay);
            Start_ADC();
            sensor = Read_ADC();
            total_sensor += sensor;
            average_sensor = total_sensor / i;
            if (i>20){

                corrected_sensor = (average_sensor*scale_2);

                //updates max value if current value is larger
                if (average_sensor>sensor_max){
                    sensor_max=average_sensor;}
                UARTprintf("FILTER = %01d ," , number);
                UARTprintf(" COR_SENSOR = %06d ," , corrected_sensor);
                UARTprintf(" RAW_SENSOR = %04d , MAX_RAW= %04d" , average_sensor, sensor_max);
                UARTprintf(" [%d, %d, %d, %d]. \r",  galv_val[0], galv_val[1], galv_val[2], galv_val[3]);



                //if current value is above threshold, update galv_val_max and adjust servo position
                if (sensor_first_max<5 && sensor>(4000/filter) && number<4){ size=0;
                            sensor_first_max = sensor;
                            repeats = 0;
                            galv_val_max[0] = galv_val[0];     galv_val_max[1] = galv_val[1];     galv_val_max[2] = galv_val[2];     galv_val_max[3] = galv_val[3];

                            }

                //if current value is below threshold and not on the last filter
                //updates sensor max to current value 'sometimes this causes the sensor value to decrease, but in practice helps the device find a mac faster
                //by ignoring noise.
                //could be updated to better filter out noise in updating sensor_max and leave the max value to keep it only increasing
                if(sensor_first_max<5 && average_sensor<(4000/filter) && number<4){
                        repeats++;
                        if (true){size=50+5*(repeats-6);sensor_max = average_sensor; linear_scan(); count++;}
                    }


                //if on the highest filter setting
                //set sensor_max to first value seen at this filter setting
                if(number==4){
                         if(start_max<1){
                             sensor_max = average_sensor;
                             start_max = 2;
                         }

                         repeats_max++;

                         //scan 5 times for better values with a decreasing step size each time
                         if (repeats_max>5){size=52-10*(repeats_max-6); linear_scan(); count++;}

                         //if no better value was found for 5 scans, call this value the max
                         if (repeats_max>10){
                             sensor_first_max = 100;

                         }
                  }

                total_sensor = 0; average_sensor = 0; i = 0;
            }

}
}
