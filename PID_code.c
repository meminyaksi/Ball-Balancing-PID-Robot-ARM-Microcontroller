#include <MKL25Z4.H>
#include <stdlib.h>  // atoi fonksiyonu için
#include <math.h>
#include <string.h>

void UART0_init(void);
char UART0_receive_char(void);
void UART0_receive_line(char* buffer, int max_len);
void parse_coordinates(char* buffer, int* x, int* y);
void PWM_init(void);
void delayMs(int n);

// PID parametreleri (Deneyerek optimize edilmeli)
static float Kp = 0.75f;
static float Ki = 0.01;
static float Kd = 8.62f;
// PID hata degiskenleri
static float errorX, errorY;
static float prevErrorX = 0, prevErrorY = 0;
static float integralX = 0, integralY = 0;
static float derivativeX, derivativeY;
static float outputX, outputY;

// Kamera çözünürlügü (Örnegin 640x480)
static const int camWidth = 640;
static const int camHeight = 480;

// Koordinat merkezi


static const int centerX = camWidth / 2;
static const int centerY = camHeight / 2;

// Servo açi sinirlari
static const int minServoAngleX = 45;
static const int maxServoAngleX = 135;
static const int minServoAngleY = 45;
static const int maxServoAngleY = 135;




int main (void) {
UART0_init();
PWM_init();
	
char buffer[16];
int currentX = 0, currentY = 0;
int servoAngleX = 90;
int servoAngleY = 90;
int pwm_valueX = 1965;
int pwm_valueY = 1965;

while (1) {
	
memset(buffer, 0, sizeof(buffer));  // buffer içerigini '\0' ile temizle

UART0_receive_line(buffer, sizeof(buffer));  // "312,250"
parse_coordinates(buffer, &currentX, &currentY);

// 2. **Normalize et (Pikseli -1 ile 1 arasina dönüstür)**  
float normX = (currentX - centerX)/(float)centerX;  // -1 ile 1 arasinda  
float normY = (currentY - centerY)/(float)centerY;    	
	
errorX = -normX;  
errorY = normY;  

// 4. **PID hesaplamalari**  

integralX += errorX;  
integralY += errorY;  

derivativeX = errorX - prevErrorX;  
derivativeY = errorY - prevErrorY;  

outputX = (Kp * errorX) + (Ki * integralX) + (Kd * derivativeX);  
outputY = (Kp * errorY) + (Ki * integralY) + (Kd * derivativeY);  

prevErrorX = errorX;  
prevErrorY = errorY;  

servoAngleX = 90 + outputX * 45;  

if(servoAngleX > maxServoAngleX)
servoAngleX = maxServoAngleX;
else if (servoAngleX < minServoAngleX)
servoAngleX = minServoAngleX;

servoAngleY = 90 + outputY * 45;  

if(servoAngleY > maxServoAngleY)
servoAngleY = maxServoAngleY;
else if (servoAngleY < minServoAngleY)
servoAngleY = minServoAngleY;

pwm_valueX = (int)(servoAngleX * 14.56f + 655);
TPM0->CONTROLS[4].CnV = pwm_valueX;	

pwm_valueY = (int)(servoAngleY * 14.56f + 655);
TPM1->CONTROLS[1].CnV = pwm_valueY;	


delayMs(50);
}
}



void UART0_init(void) {
SIM->SCGC4 |= 0x0400; /* enable clock for UART0 */
SIM->SOPT2 |= 0x04000000; /* use FLL output for UART Baud rate generator */
UART0->C2 = 0; /* turn off UART0 while changing configurations */
UART0->BDH = 0x00;
UART0->BDL = 0x0B; /* 115200 Baud */
UART0->C4 = 0x0F; /* Over Sampling Ratio 16 */
UART0->C1 = 0x00; /* 8-bit data */
UART0->C2 = 0x04; /* enable receive */
SIM->SCGC5 |= 0x0200; /* enable clock for PORTA */
PORTA->PCR[1] = 0x0200; /* make PTA1 UART0_Rx pin */

}

char UART0_receive_char(void) {
    while (!(UART0->S1 & 0x20)) {}  // RDRF bayragi = Receive Data Register Full
    return UART0->D;
}

void UART0_receive_line(char* buffer, int max_len) {
    char ch;
    int i = 0;
    while (i < max_len - 1) {
        ch = UART0_receive_char();
        if (ch == '\n') break;
        buffer[i++] = ch;
    }
    buffer[i] = '\0'; // null terminator
}



void parse_coordinates(char* buffer, int* x, int* y) {
    char* comma = strchr(buffer, ',');  // ',' konumunu bul
    if (comma) {
        *comma = '\0';  // stringi ikiye böl
        *x = atoi(buffer);        // ilk kisim x
        *y = atoi(comma + 1);     // ikinci kisim y
    }
}

void PWM_init(void) {
SIM->SCGC5 |= 0x1000; /* enable clock to Port D */
PORTD->PCR[4] = 0x0400; /* PTD4 used by TPM0 */
SIM->SCGC6 |= 0x01000000; /* enable clock to TPM0 */
SIM->SOPT2 |= 0x01000000; /* use MCGFLLCLK as timer counter clock */
TPM0->SC = 0; /* disable timer */
TPM0->CONTROLS[4].CnSC = 0x20 | 0x08; /* edge-aligned, pulse high */
TPM0->MOD = 26212; /* Set up modulo register for 50 Hz */
TPM0->CONTROLS[4].CnV = 1925; /* Set up channel value for 33% dutycycle */
TPM0->SC = 0x0C; /* enable TPM0 with prescaler /16 */
	
SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
PORT_PCR_REG(PORTB_BASE_PTR,1) = PORT_PCR_MUX(3);
SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
TPM1->SC = 0; /* disable timer */
TPM1->CONTROLS[1].CnSC = 0x20 | 0x08; /* edge-aligned, pulse high */
TPM1->MOD = 26212; /* Set up modulo register for 50 Hz */
TPM1->CONTROLS[1].CnV = 1925; /* Set up channel value for 33% dutycycle */
TPM1->SC = 0x0C; /* enable TPM0 with prescaler /16 */
	
}


void delayMs(int n) {
int i;
int j;
for(i = 0 ; i < n; i++)
for (j = 0; j < 1333; j++) {}
}
