#define TARGET_IS_BLIZZARD_RA1
//#define UART
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/i2c.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.c"
//////////// user header file ////////////
#include "rom_i2c0_func.h"

#define M_PI 3.14159265358979323846
#define SYS_DELAY_1SEC 40000000
#define SYS_DELAY_100MS 4000000
#define SYS_DELAY_10MS 400000
#define SYS_DELAY_1MS 40000
#define SYS_DELAY_100US 4000
#define SYS_DELAY_10US 400
#define DT_1SEC 100000
#define DT_100MS 10000
#define DT_10MS 1000
#define DT_7MS 700
#define DT_1MS 100
//////////// MPU6050 register address ////////////
#define MPU6050_ADDRESS 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_CONFIG 0x1A
#define MPU6050_SMPRT_DIV 0x19
#define MPU6050_GYRO_FS_SEL 0x1B
#define MPU6050_ACCEL_AFS_SEL 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_INT_BYPASS_REG 0x37
#define MPU6050_SLV0_ADDR 0x25
#define MPU6050_SLV0_REG 0x26
#define MPU6050_SLV_CTRL 0x27
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_EXT_SENS_DATA 0x49
//////////// MPU6050 config register address ////////////
#define MPU6050_DLPF_CFG    MPU6050_CONFIG
#define MPU6050_SLEEP_REG   MPU6050_PWR_MGMT_1
//////////// MPU6050 config value ////////////
#define MPU6050_RESET           0x80
#define MPU6050_SLEEP_DIS       0x00
#define MPU6050_BYPASS_EN       0x02
#define MPU6050_BYPASS_DIS      0x00
#define MPU6050_DLPF_BAND_260   0
#define MPU6050_DLPF_BAND_188   1
#define MPU6050_DLPF_BAND_98    2
#define MPU6050_DLPF_BAND_42    3
#define MPU6050_DLPF_BAND_20    4
#define MPU6050_DLPF_BAND_10    5
#define MPU6050_DLPF_BAND_5     6
//////////// Scale Factor ////////////
#define GYRO_SCALE_FACTOR 131.0
//////////// HMC5883L register address ////////////
#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_MEASURE_REG 0x00
#define HMC5883L_MODE_REG 0x02
#define HMC5883L_DATA_X_REG 0x03
//////////// HMC5883L register address ////////////
#define MS5611_ADDRESS 0x77
#define MS5611_ADC_READ 0x00
#define MS5611_RESET 0x1E
#define MS5611_PROM_ADDR_C1 0xA1
#define MS5611_PROM_ADDR_C2 0xA2
#define MS5611_PROM_ADDR_C3 0xA3
#define MS5611_PROM_ADDR_C4 0xA4
#define MS5611_PROM_ADDR_C5 0xA5
#define MS5611_PROM_ADDR_C6 0xA6
#define MS5611_PROM_C1 34047
#define MS5611_PROM_C2 61951
#define MS5611_PROM_C3 61951
#define MS5611_PROM_C4 43519
#define MS5611_PROM_C5 43519
#define MS5611_PROM_C6 61695
#define MS5611_OSR_D1_256 0x40
#define MS5611_OSR_D1_512 0x42
#define MS5611_OSR_D1_1024 0x44
#define MS5611_OSR_D1_2048 0x46
#define MS5611_OSR_D1_4096 0x48
#define MS5611_OSR_D2_256 0x50
#define MS5611_OSR_D2_512 0x52
#define MS5611_OSR_D2_1024 0x54
#define MS5611_OSR_D2_2048 0x56
#define MS5611_OSR_D2_4096 0x58
//////////// Define pin to LED color mapping ////////////
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
//////////// Motor Control ////////////
#define MOTOR_SPEED_OFFSET 750.0

//////////// Global value ////////////
double base_accel_xyz[3];
double base_gyro_xyz[3];
double angle_xyz_f[3];
double Kp = 0.5;
double Ki = 0.2;
double Kd = 0.3;
int clock_10us;
int crt_clock;
double pd0_speed = MOTOR_SPEED_OFFSET;
double pd1_speed = MOTOR_SPEED_OFFSET;
double pe4_speed = MOTOR_SPEED_OFFSET;
double pe5_speed = MOTOR_SPEED_OFFSET;
int32_t rcv_char;

void UART0Send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}
void UART1Send(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    while(ui32Count--)
    {
        ROM_UARTCharPutNonBlocking(UART1_BASE, *pui8Buffer++);
    }
}
void BlueSendKp(){
    int cnt;
    UART1Send("Kp : ", sizeof("Kp : "));
    cnt = (int)(Kp * 10);
//    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1000 + '0');
//    cnt %= 1000;
//    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
//    cnt %= 100;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
    cnt %= 10;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');
}
void BlueSendKi(){
    int cnt;
    UART1Send("Ki : ", sizeof("Ki : "));
    cnt = (int)(Kp * 10);
//    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1000 + '0');
//    cnt %= 1000;
//    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
//    cnt %= 100;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
    cnt %= 10;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');
}
void BlueSendKd(){
    int cnt;
    UART1Send("Kd : ", sizeof("Kd : "));
    cnt = (int)(Kd * 10);
//    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1000 + '0');
//    cnt %= 1000;
//    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
//    cnt %= 100;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
    cnt %= 10;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');
}
void BlueSendPitch(){
    int cnt;
    UART1Send("Pitch : ", sizeof("Pitch : "));
    cnt = (int)angle_xyz_f[1];
    if(cnt < 0){
        cnt = -cnt;
        ROM_UARTCharPutNonBlocking(UART1_BASE, '-');
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
        cnt %= 100;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
        cnt %= 10;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');
    }else{
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
        cnt %= 100;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
        cnt %= 10;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');
    }
}
void BlueSendRoll(){
    int cnt;
    UART1Send("Roll : ", sizeof("Roll : "));
    cnt = (int)angle_xyz_f[0];
    if(cnt < 0){
        cnt = -cnt;
        ROM_UARTCharPutNonBlocking(UART1_BASE, '-');
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
        cnt %= 100;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
        cnt %= 10;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');
    }else{
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
        cnt %= 100;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
        cnt %= 10;
        ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');
    }
}
void BlueSendD1Speed(){
    int cnt;
    UART1Send("D1Speed : ", sizeof("D1Speed : "));
    cnt = pd1_speed;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1000 + '0');
    cnt %= 1000;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 100 + '0');
    cnt %= 100;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 10 + '0');
    cnt %= 10;
    ROM_UARTCharPutNonBlocking(UART1_BASE, cnt / 1 + '0');

}
void nLine(){
    ROM_UARTCharPutNonBlocking(UART1_BASE, '\n');
    ROM_UARTCharPutNonBlocking(UART1_BASE, '\r');
}
void UART1IntHandler(){
    uint32_t ui32Status;
//    int32_t cnt;
//    static uint8_t str0[] = "Receive : ";
//    static uint8_t str1[] = "Kp : ";
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        rcv_char = ROM_UARTCharGetNonBlocking(UART1_BASE);
//#ifdef UART
//        UART0Send(str0, sizeof(str0));
//#endif
//        UART1Send(str1, sizeof(str1));
//        if(rcv_char == '1'){
//            pd0_speed = (pd0_speed <= 600) ? 600 : pd0_speed - 10;
//            pd1_speed = (pd1_speed <= 600) ? 600 : pd1_speed - 10;
//            pe4_speed = (pe4_speed <= 600) ? 600 : pe4_speed - 10;
//            pe5_speed = (pe5_speed <= 600) ? 600 : pe5_speed - 10;
//        }else if(rcv_char == '2'){
//            pd0_speed = (pd0_speed >= 1480) ? 1480 : pd0_speed + 10;
//            pd1_speed = (pd1_speed >= 1480) ? 1480 : pd1_speed + 10;
//            pe4_speed = (pe4_speed >= 1480) ? 1480 : pe4_speed + 10;
//            pe5_speed = (pe5_speed >= 1480) ? 1480 : pe5_speed + 10;
//        }else if(rcv_char == '3'){
//            pd0_speed = 600;
//            pd1_speed = 600;
//            pe4_speed = 600;
//            pe5_speed = 600;
//        }
        if(rcv_char == '1'){
            Kp += 0.1;
            BlueSendKp();
            nLine();
        }else if(rcv_char == '2'){
            Kp -= 0.1;
            BlueSendKp();
            nLine();
        }else if(rcv_char == '3'){
            pd0_speed = 0;
            pd1_speed = 0;
            pe4_speed = 0;
            pe5_speed = 0;
        }else if(rcv_char == '4'){
            Ki += 0.1;
            BlueSendKi();
            nLine();
        }else if(rcv_char == '5'){
            Ki -= 0.1;
            BlueSendKi();
            nLine();
        }else if(rcv_char == '6'){
            pd0_speed += 2;
            pd1_speed += 2;
            pe4_speed += 2;
            pe5_speed += 2;
            BlueSendD1Speed();
            nLine();
        }else if(rcv_char == '7'){
            Kd += 0.1;
            BlueSendKd();
            nLine();
        }else if(rcv_char == '8'){
            Kd -= 0.1;
            BlueSendKd();
            nLine();
        }else if(rcv_char == '9'){
            pd0_speed -= 2;
            pd1_speed -= 2;
            pe4_speed -= 2;
            pe5_speed -= 2;
            BlueSendD1Speed();
            nLine();
        }

    }
}

void Timer0IntHandler(void){
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_IntMasterDisable();
    clock_10us++;
    ROM_IntMasterEnable();
}
void ConfigureTimer(){
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() / 100000);//10usec per interrupt config
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER0_BASE, TIMER_A);
#ifdef UART
    UARTprintf("Timer enabled.\n");
#endif
}
void ConfigureUART(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinConfigure(GPIO_PC4_U1RX);
    ROM_GPIOPinConfigure(GPIO_PC5_U1TX);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 115200,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));
    ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    UARTStdioConfig(0, 115200, 16000000);
#ifdef UART
    UARTprintf("UART console enabled.\n");
#endif
}

void InitPWM(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
    ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0);
    ROM_GPIOPinConfigure(GPIO_PD1_M1PWM1);
    ROM_GPIOPinConfigure(GPIO_PE4_M1PWM2);
    ROM_GPIOPinConfigure(GPIO_PE5_M1PWM3);
#ifdef UART
    UARTprintf("PWM enabled.\n");
#endif
}
void InitMotorESCKETI(void) {
    volatile uint32_t PWM_FREQUENCY1 = 50;
    volatile uint32_t ui32PWMClock = ROM_SysCtlClockGet() / 64;
    volatile uint32_t ui32Load = (ui32PWMClock / PWM_FREQUENCY1) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1250);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1250);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 1250);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 1250);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    SysCtlDelay(50000000);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 625);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 625);
    PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 625);
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 625);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    SysCtlDelay(50000000);
#ifdef UART
    UARTprintf("Motor ESC initialized.\n");
#endif
}
void InitMotorESC(void) {
    volatile uint32_t PWM_FREQUENCY1 = 50;
    volatile uint32_t ui32PWMClock = ROM_SysCtlClockGet() / 64;
    volatile uint32_t ui32Load = (ui32PWMClock / PWM_FREQUENCY1) - 1;

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1250);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 625);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1250);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 625);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 1250);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 625);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_2);

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 1250);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 625);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    ROM_SysCtlDelay(ROM_SysCtlClockGet());
#ifdef UART
    UARTprintf("Motor ESC initialized.\n");
#endif
}
void SettingPWMPD0() {//min = 664, max = 1499
//    volatile uint32_t ui32PWMClock = ROM_SysCtlClockGet() / 64;
//    volatile uint32_t ui32Load = (ui32PWMClock / 300) - 1;

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, 2082);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, MOTOR_SPEED_OFFSET);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}
void SettingPWMPD1() {//min = 664, max = 1499
//    volatile uint32_t ui32PWMClock = ROM_SysCtlClockGet() / 64;
//    volatile uint32_t ui32Load = (ui32PWMClock / 300) - 1;

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 2082);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, MOTOR_SPEED_OFFSET);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}
void SettingPWMPE4() {//min = 664, max = 1499
//    volatile uint32_t ui32PWMClock = ROM_SysCtlClockGet() / 64;
//    volatile uint32_t ui32Load = (ui32PWMClock / 300) - 1;

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 2082);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, MOTOR_SPEED_OFFSET);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_2);
}
void SettingPWMPE5() {//min = 664, max = 1499
//    volatile uint32_t ui32PWMClock = ROM_SysCtlClockGet() / 64;
//    volatile uint32_t ui32Load = (ui32PWMClock / 300) - 1;

    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 2082);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, MOTOR_SPEED_OFFSET);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
}
void PD0SpeedControl(uint32_t width){
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, width);
}
void PD1SpeedControl(uint32_t width){
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, width);
}
void PE4SpeedControl(uint32_t width){
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, width);
}
void PE5SpeedControl(uint32_t width){
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, width);
}
void ConfigureI2CBus(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    ROM_I2CMasterInitExpClk(I2C0_BASE, ROM_SysCtlClockGet(), false);
#ifdef UART
    UARTprintf("I2C BUS enabled.\n");
#endif
}
//************************************
//          MPU6050
//  Digital Low Pass Filter
//    DLPF_CFG    delay(ms)
//        0           0
//        1           2
//        2           3
//        3           4.9
//        4           8.5
//        5           13.8
//        6           19.0
//       7       reserved
//************************************
void InitMPU6050(void) {
    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, MPU6050_RESET);
    ROM_SysCtlDelay(SYS_DELAY_1MS);
    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_SLEEP_REG, MPU6050_SLEEP_DIS);
    ROM_SysCtlDelay(SYS_DELAY_1MS);
//    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_DLPF_CFG, MPU6050_DLPF_BAND_5);
//    ROM_SysCtlDelay(SYS_DELAY_1MS);
//    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x00); //PWR_MGMT_1, Input disabled.
//    ROM_SysCtlDelay(SYS_DELAY_1MS);
//    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00); //PWR_MGMT_2
//    ROM_SysCtlDelay(SYS_DELAY_1MS);
//    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);//CONFIG, disable sync, DLPF_CFG = 6
//    ROM_SysCtlDelay(SYS_DELAY_1MS);
//    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_SMPRT_DIV, 0x10); //SMPRT_DIV = 31
//    ROM_SysCtlDelay(SYS_DELAY_1MS);  //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
//    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_GYRO_FS_SEL, 0x00); //GYRO_CONFIG, Full scale set to 250deg/sec
//    ROM_SysCtlDelay(SYS_DELAY_1MS);
//    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_ACCEL_AFS_SEL, 0x00); //ACCEL_CONFIG, Full scale set to 2g
//    ROM_SysCtlDelay(SYS_DELAY_1MS * 30);
#ifdef UART
    UARTprintf("MPU6050 initialized.\n");
#endif
}
void InitHMC5883L() {
    int i;
    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_INT_BYPASS_REG, MPU6050_BYPASS_EN);//mpu6050 bypass mode enable (bit1)
    ROM_SysCtlDelay(SYS_DELAY_100US * 5);
    I2C8bitWrite(HMC5883L_ADDRESS, HMC5883L_MEASURE_REG, 0x18);//hmc 75Hz enable
    ROM_SysCtlDelay(SYS_DELAY_100US * 5);
    I2C8bitWrite(HMC5883L_ADDRESS, HMC5883L_MODE_REG, 0x00);//hmc continuous read mode enable
    ROM_SysCtlDelay(SYS_DELAY_100US * 5);
    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_INT_BYPASS_REG, MPU6050_BYPASS_DIS);//mpu6050 bypass mode disable
    ROM_SysCtlDelay(SYS_DELAY_100US * 5);
    for (i = 0; i < 3; i++) {//mpu6050 slave set
        I2C8bitWrite(MPU6050_ADDRESS, MPU6050_SLV0_ADDR + i * 3, HMC5883L_ADDRESS | 0x80);//mpu6050 slave address set
        ROM_SysCtlDelay(SYS_DELAY_100US * 5);
        I2C8bitWrite(MPU6050_ADDRESS, MPU6050_SLV0_REG + i * 3, HMC5883L_DATA_X_REG + i * 2);//mpu6050 slave sub address set
        ROM_SysCtlDelay(SYS_DELAY_100US * 5);
        I2C8bitWrite(MPU6050_ADDRESS, MPU6050_SLV_CTRL + i * 3, 0x82);//mpu6050 slave length and enable set
        ROM_SysCtlDelay(SYS_DELAY_100US * 5);
    }
    I2C8bitWrite(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x20);//mpu6050 master mode enable
    ROM_SysCtlDelay(SYS_DELAY_100US * 5);
#ifdef UART
    UARTprintf("HMC5883L initialized.\n");
#endif
}
void ConfigureLED(){
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
#ifdef UART
    UARTprintf("LED enabled.\n");
#endif
}
int16_t MPU6050_read(uint8_t read_reg){
    int16_t rawData = 0;
    rawData = I2C8bitRead(MPU6050_ADDRESS, read_reg);
//    UARTprintf("raw_data_h = %d\n", rawData);
    rawData = rawData << 8 | I2C8bitRead(MPU6050_ADDRESS, read_reg + 1);
    return rawData;
}
void CalibrateSensors(){
    int num_readings = 10, i;
    double accelXYZ[3] = {0.0, 0.0, 0.0};
    double gyroXYZ[3] = {0.0, 0.0, 0.0};
    for(i = 0; i < 3; i++){
        MPU6050_read(MPU6050_ACCEL_XOUT_H + i);
        MPU6050_read(MPU6050_GYRO_XOUT_H + i);
    }
    for(i = 0; i < num_readings; i++){
        accelXYZ[0] += MPU6050_read(MPU6050_ACCEL_XOUT_H);
        accelXYZ[1] += MPU6050_read(MPU6050_ACCEL_YOUT_H);
        accelXYZ[2] += MPU6050_read(MPU6050_ACCEL_ZOUT_H);
        gyroXYZ[0] += MPU6050_read(MPU6050_GYRO_XOUT_H);
        gyroXYZ[1] += MPU6050_read(MPU6050_GYRO_YOUT_H);
        gyroXYZ[2] += MPU6050_read(MPU6050_GYRO_ZOUT_H);
//        UARTprintf("rawAccelX = %d, rawAccelY = %d, rawAccelZ = %d\n", (int)accelXYZ[0], (int)accelXYZ[1], (int)accelXYZ[2]);
//        UARTprintf("rawGyroX = %d, rawGyroY = %d, rawGyroZ = %d\n", (int)gyroXYZ[0], (int)gyroXYZ[1], (int)gyroXYZ[2]);
    }
//    UARTprintf("Sum_rawAccelX = %d, Sum_rawAccelY = %d, Sum_rawAccelZ = %d\n", (int)accelXYZ[0], (int)accelXYZ[1], (int)accelXYZ[2]);
//    UARTprintf("Sum_rawGyroX = %d, Sum_rawGyroY = %d, Sum_rawGyroZ = %d\n", (int)gyroXYZ[0], (int)gyroXYZ[1], (int)gyroXYZ[2]);
    accelXYZ[0] /=(double) num_readings;
    accelXYZ[1] /=(double) num_readings;
    accelXYZ[2] /=(double) num_readings;
    gyroXYZ[0] /=(double) num_readings;
    gyroXYZ[1] /=(double) num_readings;
    gyroXYZ[2] /=(double) num_readings;
//    UARTprintf("Average_rawAccelX = %d, Average_rawAccelY = %d, Average_rawAccelZ = %d\n", (int)accelXYZ[0], (int)accelXYZ[1], (int)accelXYZ[2]);
//    UARTprintf("Average_rawGyroX = %d, Average_rawGyroY = %d, Average_rawGyroZ = %d\n", (int)gyroXYZ[0], (int)gyroXYZ[1], (int)gyroXYZ[2]);
    base_accel_xyz[0] = accelXYZ[0];
    base_accel_xyz[1] = accelXYZ[1];
    base_accel_xyz[2] = accelXYZ[2];
    base_gyro_xyz[0] = gyroXYZ[0];
    base_gyro_xyz[1] = gyroXYZ[1];
    base_gyro_xyz[2] = gyroXYZ[2];

    accelXYZ[0] = MPU6050_read(MPU6050_ACCEL_XOUT_H);
    accelXYZ[1] = MPU6050_read(MPU6050_ACCEL_YOUT_H);
    angle_xyz_f[0] = atan2(accelXYZ[1], sqrt(pow(accelXYZ[0], 2) + pow(accelXYZ[2], 2))) * 180 / M_PI;
    angle_xyz_f[1] = atan2(-1 * accelXYZ[0], sqrtf(pow(accelXYZ[1], 2) + pow(accelXYZ[2], 2))) * 180 / M_PI;
#ifdef UART
    UARTprintf("Finishing Calibration\n");
#endif
}
double yawGet() {
    int16_t gaussRawXZY[3];
    int i;
    for (i = 0; i < 3; i++) {//external sensor read mode
        gaussRawXZY[i] = MPU6050_read(MPU6050_EXT_SENS_DATA + i);
    }
//    UARTprintf("\rHMC X Raw Data = %d     ", gaussRawXZY[0]);
//    UARTprintf("\rX Gauss = 0.%dG, Y Gauss = 0.%dG, Z Gauss = 0.%dG     "
//            , gaussRawXZY[0] * 100 / 510, gaussRawXZY[2] * 100 / 510, gaussRawXZY[1] * 100 / 510);
//    UARTprintf("\rHMC Yaw = %d     ", (int)(atan2(gaussRawXZY[2], gaussRawXZY[0]) * 180.0 / M_PI));
    return atan2(gaussRawXZY[2], gaussRawXZY[0]) * 180.0 / M_PI;
}
void GyroSpeedGet(double *gyro_speed){
    gyro_speed[0] = ((double)MPU6050_read(MPU6050_GYRO_XOUT_H) - base_gyro_xyz[0]) / GYRO_SCALE_FACTOR;
    gyro_speed[1] = ((double)MPU6050_read(MPU6050_GYRO_YOUT_H) - base_gyro_xyz[1]) / GYRO_SCALE_FACTOR;
    gyro_speed[2] = ((double)MPU6050_read(MPU6050_GYRO_ZOUT_H) - base_gyro_xyz[2]) / GYRO_SCALE_FACTOR;
//    UARTprintf("\rGyro_Speed_X = %3d, Gyro_Speed_Y = %3d, Gyro_Speed_Z = %3d       "
//            , (int)(gyro_speed[0]), (int)(gyro_speed[1]), (int)(gyro_speed[2]));
}
void AccelAngleGet(double *accel_angle_xyz){
    double raw_accel_xyz[3];
    raw_accel_xyz[0] = (double)MPU6050_read(MPU6050_ACCEL_XOUT_H);
    raw_accel_xyz[1] = (double)MPU6050_read(MPU6050_ACCEL_YOUT_H);
    raw_accel_xyz[2] = (double)MPU6050_read(MPU6050_ACCEL_ZOUT_H);
//    UARTprintf("accel_x = %d, accel_y = %d, accel_z = %d\n"
//            , (int)(raw_accel_xyz[0] / 16383.0 * 1000), (int)(raw_accel_xyz[1] / 16383.0 * 1000), (int)(raw_accel_xyz[2] / 16383.0 * 1000));
//    accel_vector_length = sqrt(powf(raw_accel_xyz[0], 2) + powf(raw_accel_xyz[1], 2) + powf(raw_accel_xyz[2], 2));
    accel_angle_xyz[0] = atan2(raw_accel_xyz[1], sqrt(pow(raw_accel_xyz[0], 2) + pow(raw_accel_xyz[2], 2))) * 180 / M_PI;
    accel_angle_xyz[1] = atan2(-1 * raw_accel_xyz[0], sqrt(pow(raw_accel_xyz[1], 2) + pow(raw_accel_xyz[2], 2))) * 180 / M_PI;
    accel_angle_xyz[2] = 0;
//        UARTprintf("\rAngle_x = %d, Angle_y = %d    ", (int)accel_angle_xyz[0], (int)accel_angle_xyz[1]);
}
void GetRPY(double *roll, double *pitch, double *yaw, double *gyro_speed){
    double accel_angle_xyz[3];
    double dt = 0.0, alpha = 0.98;

    GyroSpeedGet(gyro_speed);
    AccelAngleGet(accel_angle_xyz);
//        UARTprintf("clock_10us = %d\n", clock_10us);
//    while(clock_10us < DT_10MS);
    dt = (double)clock_10us / (double)DT_1SEC;
//    UARTprintf("dt * 1000 = %d\n", (int)(dt * 1000));
//    angle_xyz_f[0] += (gyro_speed[0] - (1 - alpha) * (angle_xyz_f[0] - accel_angle_xyz[0])) * dt;
//    angle_xyz_f[0] += gyro_speed[0] * dt * (1.0 - gain) / gain + (accel_angle_xyz[0] - angle_xyz_f[0]) / gain;
//    angle_xyz_f[1] += gyro_speed[1] * dt * (1.0 - gain) / gain + (accel_angle_xyz[1] - angle_xyz_f[1]) / gain;
//    angle_xyz_f[0] = alpha * (angle_xyz_f[0] + gyro_speed[0] * dt) + (1.0 - alpha) * accel_angle_xyz[0];
//    angle_xyz_f[1] = alpha * (angle_xyz_f[1] + gyro_speed[1] * dt) + (1.0 - alpha) * accel_angle_xyz[1];
    *roll = angle_xyz_f[0] = alpha * (angle_xyz_f[0] + gyro_speed[0] * dt) + (1.0 - alpha) * accel_angle_xyz[0];
    *pitch = angle_xyz_f[1] = alpha * (angle_xyz_f[1] + gyro_speed[1] * dt) + (1.0 - alpha) * accel_angle_xyz[1];
    *yaw = yawGet();
//    UARTprintf("Complementary Filter : Angle X = %d, Angle Y = %d, Angle Z = %d   \n", (int)*roll, (int)*pitch, (int)*yaw);

}

int main(void) {
    double roll, pitch, yaw;
    double dt = 0.0;
    double pitch_p, roll_p, pitch_i = 0.0, roll_i = 0.0, pitch_d, roll_d;
    double gyro_speed[3];

    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();
    ROM_SysCtlClockSet(
        SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    ROM_IntMasterEnable();

    ConfigureUART();
    ConfigureI2CBus();
    ConfigureTimer();
    InitPWM();
    ConfigureLED();
    InitMPU6050();
    InitHMC5883L();

    InitMotorESC();
//    SettingPWMPD0();
//    SettingPWMPE4();
//    SettingPWMPD1();
//    SettingPWMPE5();
//    InitMS5611();
    CalibrateSensors();
    ButtonsInit();
#ifdef UART
    UARTprintf("Buttons Initailized\n");
#endif
//    while(rcv_char != '0'){
//        clock_10us = 0;
//
//        SettingPWMPD0();
//        SettingPWMPE4();
////        SettingPWMPD1();
////        SettingPWMPE5();
//        GetRPY(&roll, &pitch, &yaw);
//        BlueSendPitch();
//        nLine();
////        dt = (double)clock_10us / (double)DT_1SEC;
////        UARTprintf("dt * 1000 = %d\n", (int)(dt * 1000));
////        UARTprintf("Complementary Filter : Angle X = %d, Angle Y = %d, Angle Z = %d   \n"
////                , (int)(roll), (int)(pitch), (int)(yaw));
//    }
    while(1){
        clock_10us = 0;

        GetRPY(&roll, &pitch, &yaw, gyro_speed);
        dt = (double)clock_10us / (double)DT_1SEC;
        roll_p = roll * Kp;
        pitch_p = pitch * Kp;
        roll_i += roll * dt * Ki;
        pitch_i += pitch * dt * Ki;
        roll_d = gyro_speed[0] * Kd;
        pitch_d = gyro_speed[1] * Kd;
        PE4SpeedControl(pe4_speed - pitch_p - pitch_i - pitch_d);
        PD0SpeedControl(pd0_speed + pitch_p + pitch_i + pitch_d);
        PD1SpeedControl(pd1_speed - roll_p - roll_i - roll_d);
        PE5SpeedControl(pe5_speed + roll_p + roll_i + roll_d);
//        UARTprintf("pd0_speed = %d, pe4_speed = %d, pitch = %d\n", (int)(pd0_speed + p), (int)(pe4_speed - p), (int)pitch);
//        BlueSendSpeed();
//        nLine();
        BlueSendPitch();
        nLine();
//        UARTprintf("dt * 1000 = %d\n", (int)(dt * 1000));

/*        UARTprintf("Complementary Filter : Angle Roll = %d, Angle Pitch = %d, Angle Yaw = %d   \n"
                , (int)(roll), (int)(pitch), (int)(yaw));*/
        if(ButtonsPoll(0, 0) == 0xfe)
            break;
    }
#ifdef UART
    UARTprintf("\nNOMAL END\n");
#endif
	return 0;
}
/*
void InitMS5611(){
    I2C_single_send(MS5611_ADDRESS, MS5611_RESET); //reset
    ROM_SysCtlDelay(SYS_DELAY_1MS);
#ifdef UART
    UARTprintf("MS561101BA initialized.\n");
#endif
}

/////////MS5611BA Delay////////
//OSR 4096 9.04ms
//OSR 2048 4.54ms
//OSR 1024 2.28ms
//OSR  512 1.17ms
//OSR  256 0.06ms
uint32_t PressRawGet(uint8_t OSR, int sysDelay){
    I2C_single_send(MS5611_ADDRESS, MS5611_OSR_D1_4096);//Convert config
    ROM_SysCtlDelay(sysDelay);
    I2C_single_send(MS5611_ADDRESS, MS5611_ADC_READ);//read
    return I2C_single_24bit_read(MS5611_ADDRESS);
}
uint32_t TempRawGet(uint8_t OSR, int sysDelay){
    I2C_single_send(MS5611_ADDRESS, MS5611_OSR_D1_4096);//Convert config
    ROM_SysCtlDelay(sysDelay);
    I2C_single_send(MS5611_ADDRESS, MS5611_ADC_READ);//read
    return I2C_single_24bit_read(MS5611_ADDRESS);
}
int32_t DTGet(uint32_t rawTempData){
    return rawTempData - ((uint32_t)MS5611_PROM_C5 << 8);
}
int32_t TempGet(int32_t dT){
    return ((dT * (int32_t)MS5611_PROM_C6) >> 23) + 2000;
}
int32_t PressGet(uint32_t rawPressData, int32_t dT){
    int64_t OFF = ((int32_t)MS5611_PROM_C2 << 16) + (((int32_t)MS5611_PROM_C4 * dT) >> 7);
    int64_t SENS = ((int32_t)MS5611_PROM_C2 << 15) + (((int32_t)MS5611_PROM_C3 * dT) >> 8);
//    UARTprintf("\rOFF = %d, SENS = %d   ", OFF, SENS);
    return (((rawPressData * SENS) >> 21) - OFF) >> 15;
}
void AltitudeGet() {
    uint32_t rawPressData, rawTempData;
    uint32_t temp;
    int32_t dT, press;

//    uint16_t PROM_C0 = readPROM(MS5611_ADDRESS, MS5611_PROM_ADDR_C1 - 1);
//    uint16_t PROM_C1 = readPROM(MS5611_ADDRESS, MS5611_PROM_ADDR_C1);
//    uint16_t PROM_C2 = readPROM(MS5611_ADDRESS, MS5611_PROM_ADDR_C2);
//    uint16_t PROM_C3 = readPROM(MS5611_ADDRESS, MS5611_PROM_ADDR_C3);
//    uint16_t PROM_C4 = readPROM(MS5611_ADDRESS, MS5611_PROM_ADDR_C4);
//    uint16_t PROM_C5 = readPROM(MS5611_ADDRESS, MS5611_PROM_ADDR_C5);
//    uint16_t PROM_C6 = readPROM(MS5611_ADDRESS, MS5611_PROM_ADDR_C6);
//    UARTprintf("\rPROM_C0 = %d, PROM_C1 = %d, PROM_C2 = %d, PROM_C3 = %d, PROM_C4 = %d, PROM_C5 = %d, PROM_C6 = %d     "
//            , PROM_C0, PROM_C1, PROM_C2, PROM_C3, PROM_C4, PROM_C5, PROM_C6);
    rawPressData = PressRawGet(MS5611_OSR_D1_4096, SYS_DELAY_10MS);
    rawTempData = TempRawGet(MS5611_OSR_D2_4096, SYS_DELAY_10MS);
    dT = DTGet(rawTempData);
    temp = TempGet(dT);
    press = PressGet(rawPressData, dT);
//    UARTprintf("\rdT = %d     ", dT);
//    UARTprintf("\rPress = %d      ", press);
//    UARTprintf("\rtemp = %d.%d    ", temp / 100, temp % 100);
//    UARTprintf("\rRaw Press Data = %d    ", rawPressData);
//    UARTprintf("\rRaw Temperature Data = %d    ", rawTempData);
}
*/
