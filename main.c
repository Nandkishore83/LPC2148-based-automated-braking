#include <lpc214x.h>
#include <stdint.h>

// System Configuration
#define FOSC 12000000  // 12MHz crystal
#define PCLK (FOSC * 5) // PLL multiplier (60MHz)

// Hardware Pin Definitions
#define TRIG_PIN    (1<<11)  // P0.11
#define ECHO_PIN    (IO0PIN & (1<<9)) // P0.9
#define LCD_RS      (1<<10)  // P0.10
#define LCD_RW      (1<<12)  // P0.12
#define LCD_EN      (1<<13)  // P0.13
#define LCD_DATA_MASK 0x00FF0000 // P0.16-P0.23
#define MOTOR_EN1   (1<<16)  // P1.16
#define MOTOR_EN2   (1<<17)  // P1.17
#define MOTOR_IN1   (1<<18)  // P1.18
#define MOTOR_IN2   (1<<19)  // P1.19
#define MOTOR_IN3   (1<<20)  // P1.20
#define MOTOR_IN4   (1<<21)  // P1.21
// System Parameters
#define SAFE_DISTANCE_CM     100
#define WARNING_DISTANCE_CM  50
#define CRITICAL_DISTANCE_CM 20
#define MAX_PWM_DUTY         100

// Global Variables
unsigned int current_distance = 0;
unsigned char motor_speed = MAX_PWM_DUTY;

// Function Prototypes
void SystemInit(void);
void PLL_Init(void);
void GPIO_Init(void);
void Timer_Init(void);
void LCD_Init(void);
void LCD_Cmd(unsigned char cmd);
void LCD_Data(unsigned char data);
void LCD_DisplayString(char *str);
void LCD_DisplayDistance(unsigned int distance);
void Ultrasonic_Init(void);
void Ultrasonic_Trigger(void);
unsigned int Ultrasonic_GetDistance(void);
void Motor_Control(unsigned int distance);
void Delay_ms(unsigned int ms);
void Delay_us(unsigned int us);

// Delay Functions
void Delay_ms(unsigned int ms) {
    T0TC = 0;
    T0MR0 = ms;
    T0TCR = 0x01;
    while(T0TC < T0MR0);
    T0TCR = 0x00;
}

void Delay_us(unsigned int us) {
    T1TC = 0;
    T1MR0 = us;
    T1TCR = 0x01;
    while(T1TC < T1MR0);
    T1TCR = 0x00;
}

// LCD Functions
void LCD_Init(void) {
    Delay_ms(40);
    LCD_Cmd(0x38);
    LCD_Cmd(0x0C);
    LCD_Cmd(0x06);
    LCD_Cmd(0x01);
    Delay_ms(4);
}

void LCD_Cmd(unsigned char cmd) {
    IO0CLR = LCD_DATA_MASK;
    IO0SET = (cmd << 16);
    IO0CLR = LCD_RS | LCD_RW;
    IO0SET = LCD_EN;
    Delay_us(20);
    IO0CLR = LCD_EN;
    Delay_us(200);
}

void LCD_Data(unsigned char data) {
    IO0CLR = LCD_DATA_MASK;
    IO0SET = (data << 16);
    IO0SET = LCD_RS;
    IO0CLR = LCD_RW;
    IO0SET = LCD_EN;
    Delay_us(20);
    IO0CLR = LCD_EN;
    Delay_us(200);
}

void LCD_DisplayString(char *str) {
    while(*str) {
        LCD_Data(*str++);
    }
}

void LCD_DisplayDistance(unsigned int distance) {
    LCD_Data((distance/100) + '0');
    LCD_Data(((distance/10)%10) + '0');
    LCD_Data((distance%10) + '0');
}

// Ultrasonic Sensor Functions
void Ultrasonic_Init() {
    IO0DIR |= TRIG_PIN;
    T0CTCR = 0;
    T0PR = 59;
}

void Ultrasonic_Trigger() {
    T0TC = T0PC = 0;
    IO0SET = TRIG_PIN;
    Delay_us(10);
    IO0CLR = TRIG_PIN;
}

unsigned int Ultrasonic_GetDistance() {
    unsigned int get = 0;
    Ultrasonic_Trigger();
    
    while(!ECHO_PIN);
    T0TCR = 0x01;
    
    while(ECHO_PIN);
    T0TCR = 0;
    
    get = T0TC;
    get = get / 59;
    
    return get;
}

// Motor Control Function
void Motor_Control(unsigned int distance) {
    static unsigned int pwm_counter = 0;
    
    // Always set direction (forward in this case)
    IO1SET = MOTOR_IN1 | MOTOR_IN3;
    IO1CLR = MOTOR_IN2 | MOTOR_IN4;
    
    if(distance >= SAFE_DISTANCE_CM) {
        // Full speed (100% duty cycle)
        IO1SET = MOTOR_EN1 | MOTOR_EN2;
    }
    else if(distance >= WARNING_DISTANCE_CM) {
        // Proportional speed control
        motor_speed = (distance * MAX_PWM_DUTY) / SAFE_DISTANCE_CM;
        if(pwm_counter < motor_speed) {
            IO1SET = MOTOR_EN1 | MOTOR_EN2;
        } else {
            IO1CLR = MOTOR_EN1 | MOTOR_EN2;
        }
    }
    else if(distance >= CRITICAL_DISTANCE_CM) {
        // Low speed (20% duty cycle)
        motor_speed = 20;
        if(pwm_counter < motor_speed) {
            IO1SET = MOTOR_EN1 | MOTOR_EN2;
        } else {
            IO1CLR = MOTOR_EN1 | MOTOR_EN2;
        }
    }
    else {
        // Stop motors
        IO1CLR = MOTOR_EN1 | MOTOR_EN2;
    }
    
    // Update PWM counter
    pwm_counter++;
    if(pwm_counter >= MAX_PWM_DUTY) {
        pwm_counter = 0;
    }
}
// System Initialization
void SystemInit(void) {
    PLL_Init();
    GPIO_Init();
    Timer_Init();
    LCD_Init();
    Ultrasonic_Init();
}

void PLL_Init(void) {
    PLL0CON = 0x01;
    PLL0CFG = 0x24;
    PLL0FEED = 0xAA;
    PLL0FEED = 0x55;
    while(!(PLL0STAT & (1<<10)));
    PLL0CON = 0x03;
    PLL0FEED = 0xAA;
    PLL0FEED = 0x55;
    VPBDIV = 0x01;
}

void GPIO_Init(void) {
    IO0DIR |= TRIG_PIN;
    IO0DIR &= ~(1<<9);
    IO0DIR |= (LCD_RS | LCD_RW | LCD_EN | LCD_DATA_MASK);
    IO1DIR |= (MOTOR_EN1 | MOTOR_EN2 | MOTOR_IN1 | MOTOR_IN2 | MOTOR_IN3 | MOTOR_IN4);
    
    // Set initial motor state (forward direction)
    IO1SET = MOTOR_IN1 | MOTOR_IN3;  // Set forward direction
    IO1CLR = MOTOR_IN2 | MOTOR_IN4;  // Clear reverse dir
}

void Timer_Init(void) {
    T0CTCR = 0x00;
    T0PR = 59999;
    T1CTCR = 0x00;
    T1PR = 59;
}

int main(void) {
    SystemInit();
    
    LCD_DisplayString("Automated Braking");
    LCD_Cmd(0xC0);
    LCD_DisplayString("Distance: ");
    
    while(1) {
        current_distance = Ultrasonic_GetDistance();
        
        LCD_Cmd(0xC9); // Set cursor to overwrite distance value
        LCD_DisplayDistance(current_distance);
        LCD_DisplayString(" cm");
        
        Motor_Control(current_distance);
        
        Delay_ms(10);
    }
    return 0;
}
