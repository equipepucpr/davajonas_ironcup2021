#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#define debug

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
/*******PINOUT DEFINES - END*********/

/*******FUNCTIONS*******/
void MotorL(uint8_t, uint8_t); // left motor / motor esquerdo / motor izquierdo
void MotorR(uint8_t, uint8_t); // right motor / motor direito / motor derecho
uint8_t readDIP(); // read DIP switch / ler chave DIP / leer el interruptor DIP
uint8_t lineCheck();
uint8_t distCheck();
/*******FUNCTIONS - END*******/

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
	
	/****************PINOUT CONFIG****************/
	// OUTPUTS
	PORTD = 0;
	DDRD = (1 << LED) | (1 << leftMotor1) | (1 << rightMotor1) | (1 << rightMotor2);
	PORTB = 0;
	DDRB = (1 << leftMotor2);
	
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
		
	//RTDM()
	while(1) {
		  //DEFENSE
		  uint8_t lineSensor = lineCheck();
		  switch (lineSensor) {
			  case 0b01: //right detect
				  MotorL(BACKWARD(255));
				  MotorR(FORWARD(255));
				  _delay_ms(100);
				  break;
			  case 0b10:  //left detect
				  MotorL(FORWARD(255));
				  MotorR(BACKWARD(255));
				  _delay_ms(100);
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
		  //ATTACK
		  uint8_t distSensor;
		  do {
			  distSensor = distCheck();
			  switch (distSensor) {
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
		  } while (distSensor);
		MotorL(FORWARD(200));
		MotorR(FORWARD(200));
		
#ifdef debug
		_delay_ms(1000);
#endif
	}
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
	ret |= (PINB & (1 << DIP1)) << 3;
	ret |= (PINB & (1 << DIP2)) << 2;
	ret |= (PINB & (1 << DIP3)) << 1;
	ret |= PINB & (1 << DIP4);
	
#ifdef debug
	char buf[50];
	sprintf(buf, "dip switch: %d\n", ret);
	log(buf);
#endif

	return ret;
}