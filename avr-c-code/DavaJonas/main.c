#include <avr/io.h>

#define LINE_THRESHOLD 700

// LED
#define LED PD6

// left motor
#define pwmL PB1
#define leftMotor1 PD7
#define leftMotor2 PB0

// right motor
#define pwmR PD3
#define rightMotor1 PD5
#define rightMotor2 PD4

// DIP switch
#define DIP1 PB2
#define DIP2 PB3
#define DIP3 PB4
#define DIP4 PB5

// Robocore's line sensor
#define lineL PC0
#define lineR PC1

// Jsumo's distance sensor
#define distL PC2
#define distR PC3

// Jsumo's micro-start
#define microST PD2
/*******PINOUT DEFINES - END*********/

/*******FUNCTIONS*******/
void MotorL(int pwm); // left motor / motor esquerdo / motor izquierdo
void MotorR(int pwm); // right motor / motor direito / motor derecho
uint8_t readDIP(); // read DIP switch / ler chave DIP / leer el interruptor DIP
/*******FUNCTIONS - END*******/

int main(void)
{
	/****************PINOUT CONFIG****************/
	// OUTPUTS
	DDRD = ((1 << LED) | (1 << leftMotor1) | (1 << pwmR) | (1 << rightMotor1) | (1 << rightMotor2));
	DDRB = ((1 << pwmL) | (1 << leftMotor2));
	
	//LEFT MOTOR PWM SETTING
	TCCR1A = (1 << COM1A1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	
	//RIGHT MOTOR PWM SETTING
	TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (1 << CS22);
	
	// INPUTS: DO NOT CHANGE / NAO MUDAR / NO CAMBIAR
	// DIP switch
	PORTB = ((1 << DIP1) | (1 << DIP2) | (1 << DIP3) | (1 << DIP4)); //setting internal pullups
	/****************PINOUT CONFIG - END***************/
	
	/***************INITIAL CONDITIONS*****************/
	PORTB &= ~(1 << LED); // LED off / LED desligado / LED apagado
	MotorL(0); // left motor stopped / motor esquerdo parado / motor izquierdo parado
	MotorR(0); // right motor stopped / motor direito parado / motor derecho parado
	/*************INITIAL CONDITIONS - END*************/
    /* Replace with your application code */
	OCR1A = 0x7F;
	OCR2B = 0x7F;
    while (1) 
    {
    }
}

void interrupt() {
	MotorL(0);
	MotorR(0);
	PORTB &= ~(1 << LED);
	resetSoftware();
	while(1) {}; //It should never get here
}

uint8_t lineCheck() {
	uint16_t sensor_r, sensor_l;
	//sensor_r = analogRead(lineR); TODO
	//sensor_l = analogRead(lineL); TODO

	uint8_t ret = 0;
	if (sensor_r <= LINE_THRESHOLD)
	ret |= 0b01;
	if (sensor_l <= LINE_THRESHOLD)
	ret |= 0b10;

	return ret;
}

/**LEFT MOTOR CONTROL / CONTROLE DO MOTOR ESQUERDO / CONTROL DEL MOTOR IZQUIERDO**/
// pwm = 0 -> stopped / parado / parado
// 0<pwm<=255 -> forward / para frente / seguir adelante
// -255<=pwm<0 -> backward / para tras / seguir espalda
void MotorL(int pwm){
	// leftMotor1=0 and leftMotor2=0 -> stopped / parado / parado
	// leftMotor1=0 and leftMotor2=1 -> moves forward / avanca / avanzar
	// leftMotor1=1 and leftMotor2=0 -> moves back / recua / retrocede
	// leftMotor1=1 and leftMotor2=1 -> stopped (braked) / parado (travado) / parado (frenado)
	
	if(pwm==0){
		PORTD &= ~(1 << leftMotor1);
		PORTB &= ~(1 << leftMotor2);
	}
	else if(pwm<0)
	{
		//analogWrite(pwmL, -pwm); TODO
		PORTD |= (1 << leftMotor1);
		PORTB &= ~(1 << leftMotor2);
	}
	else
	{
		//analogWrite(pwmL, pwm); TODO
		PORTD &= ~(1 << leftMotor1);
		PORTD |= (1 << leftMotor2);
	}
}


/**RIGHT MOTOR CONTROL / CONTROLE DO MOTOR DIREITO / CONTROL DEL MOTOR DERECHO**/
// pwm = 0 -> stopped / parado / parado
// 0<pwm<=255 -> forward / frente / adelante
// -255<=pwm<0 -> backward / tras / espalda
void MotorR(int pwm){
	// rightMotor1=0 and rightMotor2=0 -> stopped / parado / parado
	// rightMotor1=0 and rightMotor2=1 -> moves forward / avanca / avanzar
	// rightMotor1=1 and rightMotor2=0 -> moves back / recua / retrocede
	// rightMotor1=1 and rightMotor2=1 -> stopped (braked) / parado (travado) / parado (frenado)
	
	if(pwm==0){
		PORTD &= ~(1 << rightMotor1);
		PORTD &= ~(1 << rightMotor2);
	}
	else if(pwm<0)
	{
		//analogWrite(pwmR, -pwm); TODO
		PORTD |= (1 << rightMotor1);
		PORTD &= ~(1 << rightMotor2);
	}
	else
	{
		//analogWrite(pwmR, pwm); TODO
		PORTD &= ~(1 << rightMotor1);
		PORTD |= (1 << rightMotor2);
	}
}

/** read DIP switch / ler chave DIP / leer el interruptor DIP **/
// returns a value between 0 and 15
// retorna um valor entre 0 e 15
// devuelve un valor entre 0 y 15
uint8_t readDIP(){
	return (PINB & ((1 << DIP1) | (1 << DIP2) | (1 << DIP3) | (1 << DIP4))) >> DIP1;
}
