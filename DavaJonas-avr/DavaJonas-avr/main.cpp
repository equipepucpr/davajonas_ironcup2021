#define F_CPU 16000000UL

//GCC 4+ needs theses functions for static variables
__extension__ typedef int __guard __attribute__((mode (__DI__)));

int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
void __cxa_guard_abort (__guard *) {};

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#define debug
#define DIP_VAL 0b0000

#ifdef debug
#include <stdio.h>
#endif

//Line sensor threshold
#define LINE_THRESHOLD 700

// LED
#define LED PD6

// MOTORS
#define STOP 0, 2
#define FORWARD(x) x, 1
#define BACKWARD(x) x, 0

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

#define readDist(x) (PINC & (1 << x))

// Jsumo's micro-start
#define microST PD2

//RTDM Start States
#define END 0
#define NOCHECK 1
#define CHECKALL 2
#define CHECKLINE 3
#define CHECKDIST 4

#define BREAKIF(x) (x | 0b1000)

/*******PINOUT DEFINES - END*********/

/*******FUNCTIONS*******/
void MotorL(uint8_t, uint8_t); // left motor / motor esquerdo / motor izquierdo
void MotorR(uint8_t, uint8_t); // right motor / motor direito / motor derecho
uint8_t readDIP(); // read DIP switch / ler chave DIP / leer el interruptor DIP
uint8_t lineCheck();
uint8_t distCheck();
void RTDM();
uint8_t CSL();
void FDL(uint8_t);
void FAL(uint8_t);
void CPL();
/*******FUNCTIONS - END*******/

uint8_t dip = 0;

#ifdef debug
#define UART_PRESCALER 8 //115200
void sendArray(char array[]) {
	uint8_t i = 0;
	while (array[i]) {
		while (!( UCSR0A & (1<<UDRE0))) {}; /* Wait for empty transmit buffer*/
		UDR0 = array[i];            /* Put data into buffer, sends the data */
		i++;
	};
}

#define log(array) sendArray(array)
#else
#define log(array) /*sendArray(array)*/
#endif

void(* resetSoftware)(void) = 0;

volatile uint16_t millis;

ISR (TIMER0_COMPA_vect){
	millis++;
}

ISR (INT0_vect) {
	#ifdef debug
	log("Stopping...\n");
	#endif
	MotorL(FORWARD(0));
	MotorR(FORWARD(0));
	PORTB &= ~(1 << LED);
	resetSoftware();
	while (1) {}; //It should never get here
}

int main(void)
{
	
	#ifdef debug //UART Config
	UBRR0H = (UART_PRESCALER>>8);
	UBRR0L = (UART_PRESCALER);
	
	UCSR0C = 0x06;       /* Set frame format: 8data, 1stop bit  */
	UCSR0B = (1<<TXEN0); /* Enable  transmitter                 */
	#endif

	//Set millis timer
	millis = 0;
	TCCR0A = 0; // set entire TCCR0A register to 0
	TCCR0B = 0; // same for TCCR0B
	TCNT0  = 0; // initialize counter value to 0
	TCCR0A |= (1 << WGM01);
	TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
	TIMSK0 |= (1 << OCIE0A);
	OCR0A = 249;
	
	/****************PINOUT CONFIG****************/
	// OUTPUTS
	PORTD = 0;
	DDRD = (1 << LED) | (1 << leftMotor1) | (1 << rightMotor1) | (1 << rightMotor2);
	PORTB = 0;
	DDRB = (1 << leftMotor2);
	
	//DIP INPUT PULLUP
	DDRB |= (1 << DIP1) | (1 << DIP2) | (1 << DIP3) | (1 << DIP4);
	
	//LEFT MOTOR PWM SETTING
	TCCR1A = (1 << COM1A1) | (1 << WGM10);
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	
	//RIGHT MOTOR PWM SETTING
	TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
	TCCR2B = (1 << CS22);
	
	//ADC INIT
	DDRC = 0;
	PORTC = 0;
	ADCSRA = (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | (1 << ADEN);
	ADMUX = (1 << REFS0); //set ADC VRef to AVCC
	
	//microst input interrupt (INT0 - PD2)
	EICRA |= (1 << ISC01); //Trigger when a falling edge is detected
	EIMSK |= (1 << INT0); //Enable interrupt
	
	// INPUTS: DO NOT CHANGE / NAO MUDAR / NO CAMBIAR
	// DIP switch
	PORTB = (1 << DIP1) | (1 << DIP2) | (1 << DIP3) | (1 << DIP4); //setting internal pullups
	/****************PINOUT CONFIG - END***************/
	
	/***************INITIAL CONDITIONS*****************/
	PORTB &= ~(1 << LED); // LED off / LED desligado / LED apagado
	MotorL(FORWARD(0)); // left motor stopped / motor esquerdo parado / motor izquierdo parado
	MotorR(FORWARD(0)); // right motor stopped / motor direito parado / motor derecho parado
	/*************INITIAL CONDITIONS - END*************/

	//Enable interrupts
	sei();
	
	log("Ready!\n");
	
	while (!(PIND & (1 << microST))) {};
	log("Starting...\n");
	
	#ifdef DIP_VAL
	dip = DIP_VAL;
	#else
	dip = readDIP();
	#endif
	
	RTDM();
	
	while(1) {}; //It should never get here
}

//RTDM()
void RTDM() {
	uint8_t startState = CHECKALL;
	while (startState) {
		startState = CSL(); //Configurable Start Logic
		
		if ((startState & 0b0111) > NOCHECK) {
			if ((startState & 0b0111) != CHECKDIST) {
				//Check line sensors
				uint8_t lineSensor = lineCheck();
				if (lineSensor) { //If any sensor is triggered
					FDL(lineSensor); //Fixed Defense Logic
					if (startState & 0b1000)
						break;
					continue; //Skip everything else
				}
			}
		
			if ((startState & 0b0111) != CHECKLINE)	{
				//Check distance sensors
				uint8_t distSensor = distCheck();
				if (distSensor) { //If any sensor is triggered
					FAL(distSensor); //Fixed Attack Logic
					if (startState & 0b1000)
						break;
					continue; //Skip everything else
				}
			}
		}
		#ifdef debug
		_delay_ms(1000);
		log("\n");
		#endif
	}
	
	while (1) {
		//Check line sensors
		uint8_t lineSensor = lineCheck();
		if (lineSensor) { //If any sensor is triggered
			FDL(lineSensor); //Fixed Defense Logic
			continue; //Skip everything else
		}
		
		//Check distance sensors
		uint8_t distSensor = distCheck();
		if (distSensor) { //If any sensor is triggered
			FAL(distSensor); //Fixed Attack Logic
			continue; //Skip everything else
		}
		
		CPL(); //Configurable Patrol Logic
		
		#ifdef debug
		_delay_ms(1000);
		log("\n");
		#endif
	}
}

//CANNOT BE BLOCKING (EX: DON'T USE SLEEP)!
uint8_t CSL() {
	//TODO> Add all start logics
	static uint16_t start = millis;
	MotorL(BACKWARD(127));
	MotorR(FORWARD(127));
	if (millis - start <= 2000)
		return BREAKIF(CHECKDIST);
	
	return END;
}

//CANNOT BE BLOCKING (EX: DON'T USE SLEEP)!
void CPL() {
	//TODO: Add all patrol logics
	MotorL(FORWARD(127));
	MotorR(FORWARD(127));
}

void FDL(uint8_t sensors) {
	//TODO: change this to something more elegant (Ex: rotate until sensor returns false)
	switch (sensors) {
		case 0b01: //right detect
		MotorL(BACKWARD(255));
		MotorR(FORWARD(255));
		_delay_ms(120);
		break;
		case 0b10:  //left detect
		MotorL(FORWARD(255));
		MotorR(BACKWARD(255));
		_delay_ms(120);
		break;
		case 0b11: //frontal detect
		MotorL(BACKWARD(255));
		MotorR(BACKWARD(255));
		_delay_ms(150);
		MotorL(FORWARD(255));
		MotorR(BACKWARD(255));
		_delay_ms(200);
		break;
	}
}

void FAL(uint8_t sensors) {
	do {
		switch (sensors) {
			case 0b01: //right detect
			MotorL(FORWARD(200));
			MotorR(BACKWARD(200));
			break;
			case 0b10: //left detect
			MotorL(BACKWARD(200));
			MotorR(FORWARD(200));
			break;
			case 0b11: //frontal detect
			MotorL(FORWARD(255));
			MotorR(FORWARD(255));
			break;
		}
		sensors = distCheck();
	} while (sensors);
}

uint8_t distCheck() {
	uint8_t ret = 0;
	if (readDist(distR))
	ret |= 0b01;
	if (readDist(distL))
	ret |= 0b10;
	
	#ifdef debug
	char buf[50];
	sprintf(buf, "distance sensor: %d\n", ret);
	log(buf);
	#endif
	
	return ret;
}

uint8_t lineCheck() {
	uint16_t sensor_r, sensor_l;
	
	ADMUX &= ~(1 << MUX3 | 1 << MUX2  | 1 << MUX1 | 1 << MUX0);
	ADCSRA |= (1 << ADSC);
	while (!(ADCSRA & (1 << ADIF))) {};
	sensor_l = ADC;
	
	ADMUX |= (1 << MUX0);
	ADCSRA |= (1 << ADSC);
	while (!(ADCSRA & (1 << ADIF))) {};
	sensor_r = ADC;

	uint8_t ret = 0;
	if (sensor_r <= LINE_THRESHOLD)
	ret |= 0b01;
	if (sensor_l <= LINE_THRESHOLD)
	ret |= 0b10;
	#ifdef debug
	char buf[50];
	sprintf(buf, "line sensor: %d\n", ret);
	log(buf);
	#endif
	return ret;
}

void setPWML(uint8_t pwm) {
	if (pwm) {
		DDRB |= (1 << pwmL);
		} else {
		DDRB &= ~(1 << pwmL);
	}
	OCR1A = pwm;
}

/**LEFT MOTOR CONTROL / CONTROLE DO MOTOR ESQUERDO / CONTROL DEL MOTOR IZQUIERDO**/
// pwm = 0 -> stopped / parado / parado
// dir == 2 -> braked
// dir == 1 -> forward / para frente / seguir adelante
// dir == 0 -> backward / para tras / seguir espalda
void MotorL(uint8_t pwm, uint8_t state){
	// leftMotor1=0 and leftMotor2=0 -> stopped / parado / parado
	// leftMotor1=0 and leftMotor2=1 -> moves forward / avanca / avanzar
	// leftMotor1=1 and leftMotor2=0 -> moves back / recua / retrocede
	// leftMotor1=1 and leftMotor2=1 -> stopped (braked) / parado (travado) / parado (frenado)
	#ifdef debug
	char buf[50];
	sprintf(buf, "left motor pwm: %d\n", pwm);
	log(buf);
	sprintf(buf, "left motor state: %d\n", state);
	log(buf);
	#else
	switch (state) {
		case 0x00:
		PORTD |= (1 << leftMotor1);
		PORTB &= ~(1 << leftMotor2);
		setPWML(pwm);
		break;
		case 0x01:
		PORTD &= ~(1 << leftMotor1);
		PORTB |= (1 << leftMotor2);
		setPWML(pwm);
		break;
		case 0x02:
		setPWML(0);
		PORTD |= ~(1 << leftMotor1);
		PORTB |= (1 << leftMotor2);
		break;
	}
	#endif
}

void setPWMR(uint8_t pwm) {
	if (pwm) {
		DDRD |= (1 << pwmR);
		} else {
		DDRD &= ~(1 << pwmR);
	}
	OCR2B = pwm;
}


/**RIGHT MOTOR CONTROL / CONTROLE DO MOTOR DIREITO / CONTROL DEL MOTOR DERECHO**/
// pwm = 0 -> stopped / parado / parado
// dir == 1/true -> forward / para frente / seguir adelante
// dir == 0/false -> backward / para tras / seguir espalda
void MotorR(uint8_t pwm, uint8_t state){
	// leftMotor1=0 and leftMotor2=0 -> stopped / parado / parado
	// leftMotor1=0 and leftMotor2=1 -> moves forward / avanca / avanzar
	// leftMotor1=1 and leftMotor2=0 -> moves back / recua / retrocede
	// leftMotor1=1 and leftMotor2=1 -> stopped (braked) / parado (travado) / parado (frenado)
	#ifdef debug
	char buf[50];
	sprintf(buf, "left motor pwm: %d\n", pwm);
	log(buf);
	sprintf(buf, "left motor state: %d\n", state);
	log(buf);
	#else
	switch (state) {
		case 0x00:
		PORTD |= (1 << rightMotor1);
		PORTD &= ~(1 << rightMotor2);
		setPWMR(pwm);
		break;
		case 0x01:
		PORTD &= ~(1 << rightMotor1);
		PORTD |= (1 << rightMotor2);
		setPWMR(pwm);
		break;
		case 0x02:
		setPWMR(0);
		PORTD |= ~(1 << rightMotor1);
		PORTD |= (1 << rightMotor2);
		break;
	}
	#endif
}

/** read DIP switch / ler chave DIP / leer el interruptor DIP **/
// returns a value between 0 and 15
// retorna um valor entre 0 e 15
// devuelve un valor entre 0 y 15
uint8_t readDIP(){
	uint8_t ret = 0;
	if (PINB & (1 << DIP1))
	ret |= 0b1000;
	if (PINB & (1 << DIP2))
	ret |= 0b0100;
	if (PINB & (1 << DIP3))
	ret |= 0b0010;
	if (PINB & (1 << DIP4))
	ret |= 0b0001;
	
	#ifdef debug
	char buf[50];
	sprintf(buf, "dip switch: %d\n", ret);
	log(buf);
	#endif

	return ret;
}