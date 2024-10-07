float Kp = 1.0f;
float Ki = 0.1f;
float Kd = 0.01f;
float desiredSpeed = 5999.0f;
float actualSpeed = 0.0f;
float error = 0.0, previousError = 0.0, integral = 0.0, derivative = 0.0;
float dt = 0.1;
float output = 0;
volatile int encoderCount = 0;
float currentSpeed = 0.0f;

volatile int encoderPulses = 0;
volatile uint32_t lastTime = 0;
float wheelSpeed = 0;
float expected_output = 288.0f;

void setup_SW1_interrupt();
void setup_PORTA_interrupt();
void PORTC_PORTD_IRQHandler(void);
void motorDirection(int direction);
void motor_drive(void);
void timeDelay(unsigned short t);
void pid_update();
void readEncoder();
void updateEncoder();
float adjust_pid(float expectedOutput, float actualOutput);
int t = 0;
