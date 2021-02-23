#define F_CPU 16000000UL

//GCC 4+ needs these functions for non-constant static variable initialization
__extension__ typedef int __guard __attribute__((mode (__DI__)));

int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
void __cxa_guard_abort (__guard *) {};
//END of GCC 4+ functions

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//debug -> Print motor/sensor values on serial (motors disabled)
//#define debug

//DIP_VAL -> set a constant dip value (it then ignores the dip switch)
//#define DIP_VAL 0b0000

//#define disableAttack
//#define disableDefense

#ifdef debug
#include <stdio.h> //Needed for sprintf
#endif

//Motor Rotate Compensation (Compensate the time needed to rotate X degrees)
#define PERCENT 1.0F
#define ROT_DELAY(x) ((uint8_t) x * (PERCENT * 395.0F/180.0F))
#define ROT_LEFT  MotorR(FORWARD(200)); MotorL(BACKWARD(200));
#define ROT_RIGHT MotorR(BACKWARD(200)); MotorL(FORWARD(200));

//Line sensor threshold
#define LINE_THRESHOLD 700 //Analog threshold for Dohyo line detection

// LED Port
#define LED PD6

// MOTORS
#define STOP 0, 3 //Stop and break wheel
#define FREEWHEEL 0, 2 //Stop but don't brake
#define FORWARD(x) x, 1 //Go forward with pwm of value X (0 to 255)
#define BACKWARD(x) x, 0 //Go backward with pwm of value X (0 to 255)

// left motor ports
#define pwmL PB1
#define leftMotor1 PD7
#define leftMotor2 PB0

// right motor ports
#define pwmR PD3
#define rightMotor1 PD5
#define rightMotor2 PD4

// DIP switch ports
#define DIP1 PB2
#define DIP2 PB3
#define DIP3 PB4
#define DIP4 PB5

// Robocore's line sensor ports
#define lineL PC0
#define lineR PC1

// Jsumo's presence sensor ports
#define distL PC2
#define distR PC3

#define readDist(x) (PINC & (1 << x)) //Digitally read the presence/distance sensor

// Jsumo's micro-start port
#define microST PD2
/*******PINOUT DEFINES - END*********/

//RTDM Start States
#define END 0 //Exit start logic
#define NOCHECK 1 //Run start logic loop without sensor checking
#define CHECKALL 2 //Run start logic loop checking all sensors
#define CHECKLINE 3 //Run start logic loop checking line sensors
#define CHECKDIST 4 //Run start logic loop checking presence sensors

#define ENDIF(x) (x | 0b1000) //Break start logic loop if sensor is triggered - Ex: ENDIF(CHECKLINE) -> break start logic loop if line is detected

/*******FUNCTIONS*******/
void MotorL(uint8_t, uint8_t); // left motor / motor esquerdo / motor izquierdo
void MotorR(uint8_t, uint8_t); // right motor / motor direito / motor derecho
uint8_t readDIP(); // read DIP switch / ler chave DIP / leer el interruptor DIP
uint8_t lineCheck(); //Read both line sensors
uint8_t distCheck(); //Read both presence sensors
void RTDM(); //Real Time Decision Making (main loop)
uint8_t CSL(); //Customizable Start Logic
void FDL(uint8_t); //Fixed Defense Logic
void FAL(uint8_t); //Fixed Attack Logic
void CPL(); //Customizable Patrol Logic
/*******FUNCTIONS - END*******/

uint8_t dip = 0; //Initial DIP value

#ifdef debug
#define UART_PRESCALER 8 //115200bps (3.5% error)
//Send string over serial
void sendArray(const char array[]) {
	uint8_t i = 0;
	//Go through all characters, one by one
	while (array[i]) {
		while (!( UCSR0A & (1<<UDRE0))) {}; /* Wait for empty transmit buffer*/
		UDR0 = array[i];            /* Put data into buffer, sends the data */
		i++;
	};
}

//Disable log if debug isn't defined
#define log(array) sendArray(array)
#else
#define log(array) /*sendArray(array)*/
#endif

//Reset function, points to address 0 of memory space (first assembly instruction)
void(* resetSoftware)(void) = 0;

//Milliseconds variable -> increased by timer0
volatile uint32_t millis;

//Timer0 interrupt -> used for millis
ISR (TIMER0_COMPA_vect){
	millis++;
}

//microST interrupt -> stop motors and restart mcu
ISR (INT0_vect) {
	#ifdef debug
	log("Stopping...\n");
	#endif
	MotorL(FREEWHEEL);
	MotorR(FREEWHEEL);
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
	PORTB |= (1 << DIP1) | (1 << DIP2) | (1 << DIP3) | (1 << DIP4);
	
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
	MotorL(FREEWHEEL); // left motor stopped / motor esquerdo parado / motor izquierdo parado
	MotorR(FREEWHEEL); // right motor stopped / motor direito parado / motor derecho parado
	/*************INITIAL CONDITIONS - END*************/

	//Enable interrupts
	sei();
	
	log("Ready!\n");
	
	//While microST is low (0v) don't do anything
	while (!(PIND & (1 << microST))) {};
	log("Starting...\n");
	
	//set DIP value
#ifdef DIP_VAL
dip = DIP_VAL;
	#else
	dip = readDIP();
#endif

	//Main program loop -> It should never return to main
	RTDM();
	
	while(1) {}; //It should never get here
}

//RTDM Real Time Decision Making
//This function executes CSL, CPL, FDL and FAL if the conditions are met
void RTDM() {
	uint8_t startState = CHECKALL;
	
	//Start Logic Loop
	while (startState) {
		startState = CSL(); //Configurable Start Logic
		
		if ((startState & 0b0111) > NOCHECK) {
			if ((startState & 0b0111) != CHECKDIST) {
				//Check line sensors
				uint8_t lineSensor = lineCheck();
				if (lineSensor) { //If any sensor is triggered
					FDL(lineSensor); //Fixed Defense Logic
					if (startState & 0b1000) //ENDIF
						break;
					continue; //Skip everything else
				}
			}
		
			if ((startState & 0b0111) != CHECKLINE)	{
				//Check distance sensors
				uint8_t distSensor = distCheck();
				if (distSensor) { //If any sensor is triggered
					FAL(distSensor); //Fixed Attack Logic
					if (startState & 0b1000) //ENDIF
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
	
	//Patrol Logic Loop
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

//Configurable Start Logic
//CANNOT BE BLOCKING (EX: DON'T USE SLEEP)!
uint8_t CSL() {
	static uint32_t start = millis;
	
	//Check the two msb
	switch ((dip & 0b1100) >> 2) {
		case 0x0: //GO FORWARD (A3/B3)
			if (millis - start < 400) {
				MotorL(FORWARD(100));
				MotorR(FORWARD(100));
				return ENDIF(CHECKALL);
			}
			return END; 
			
		case 0x1: //Omae Wa Mou Shindeiru left (A5)
			if (millis - start < 1000) {
				MotorL(FORWARD(85));
				MotorR(FORWARD(127));
				return NOCHECK;
			}
			if (millis - start < 1500) {
				MotorL(FORWARD(85));
				MotorR(FORWARD(127));
				return ENDIF(CHECKDIST);
			}
			if (millis - start < 1500 + ROT_DELAY(90)) {
				ROT_LEFT;
				return ENDIF(CHECKDIST);
			}
			return END;
			
		case 0x2: //Omae Wa Mou Shindeiru Right (A1)
			if (millis - start < 1000) {
				MotorR(FORWARD(85));
				MotorL(FORWARD(127));
				return NOCHECK;
			}
			if (millis - start < 1500) {
				MotorR(FORWARD(85));
				MotorL(FORWARD(127));
				return ENDIF(CHECKDIST);
			}
			if (millis - start < 1500 + ROT_DELAY(90)) {
				ROT_RIGHT;
				return ENDIF(CHECKDIST);
			}
			return END;
			
		case 0x3: //Wait for enemy (check front, left and right) (A3)
			if (millis - start < ROT_DELAY(45)) {
				ROT_LEFT;
				return ENDIF(CHECKALL);
			}
			if (millis - start < ROT_DELAY(45) + 200) {
				MotorR(STOP);
				MotorL(STOP);
				return ENDIF(CHECKALL);
			}
			if (millis - start < 3*ROT_DELAY(45) + 200) {
				ROT_RIGHT;
				return ENDIF(CHECKALL);
			}
			if (millis - start < 3*ROT_DELAY(45) + 400) {
				MotorR(STOP);
				MotorL(STOP);
				return ENDIF(CHECKALL);
			}
			if (millis - start < 4*ROT_DELAY(45) + 400) {
				ROT_LEFT;
				return ENDIF(CHECKALL);
			}
			start = millis;
			return ENDIF(CHECKALL);
	}
	
	return END;
}

//Configurable Patrol Logic
//CANNOT BE BLOCKING (EX: DON'T USE SLEEP)!
void CPL() {
	static uint32_t start = millis;
	
	//Check the two lsb
	switch (dip & 0b0011) {
		case 0x0: //GO FORWARD 200
			MotorL(FORWARD(100));
			MotorR(FORWARD(100));
			break;
			
		case 0x1: //ZigZag
			if (millis - start < ROT_DELAY(45)) {
				ROT_LEFT;
				return;
			}
			if (millis - start < ROT_DELAY(45) + 200) {
				MotorL(FORWARD(100));
				MotorR(FORWARD(100));
				return;
			}
			if (millis - start < ROT_DELAY(90) + ROT_DELAY(45) + 200) {
				ROT_RIGHT;
				return;
			}
			if (millis - start < ROT_DELAY(90) + ROT_DELAY(45) + 2*200) {
				MotorL(FORWARD(100));
				MotorR(FORWARD(100));
				return;
			}
			if (millis - start <  ROT_DELAY(90) + 2*ROT_DELAY(45) + 2*200) {
				ROT_LEFT;
				return;
			}
			start = millis;
			break;
			
		case 0x2: //Woodpecker
			if (millis - start < ROT_DELAY(45)) {
				ROT_LEFT;
				return;
			}
			if (millis - start < ROT_DELAY(45) + 500) {
				MotorL(STOP);
				MotorR(STOP);
				return;
			}
			if (millis - start < ROT_DELAY(45) + 500 + 200) {
				MotorL(FORWARD(100));
				MotorR(FORWARD(100));
				return;
			}
			if (millis - start < ROT_DELAY(90) + ROT_DELAY(45) + 500 + 200) {
				ROT_RIGHT;
				return; 
			}
			if (millis - start < ROT_DELAY(90) + ROT_DELAY(45) + 200 + 2*500) {
				MotorL(STOP);
				MotorR(STOP);
				return;
			}
			if (millis - start < ROT_DELAY(90) + ROT_DELAY(45) + 2*200 + 2*500) {
				MotorL(FORWARD(100));
				MotorR(FORWARD(100));
				return;
			}
			if (millis - start <  ROT_DELAY(90) + 2*ROT_DELAY(45) + 2*200 + 2*500) {
				ROT_LEFT;
				return;
			}
			start = millis;
			break;
			
		case 0x3: //Tornado
			ROT_RIGHT;
			break;
	}
}

//Fixed Defense Logic
void FDL(uint8_t sensors) {
	switch (sensors) {
		case 0b01: //right detect
		MotorL(BACKWARD(255));
		MotorR(BACKWARD(255));
		while (lineCheck()) {};
		ROT_LEFT;
		_delay_ms(ROT_DELAY(90));
		MotorL(STOP);
		MotorR(STOP);
		break;
		case 0b10:  //left detect
		MotorL(BACKWARD(255));
		MotorR(BACKWARD(255));
		while (lineCheck()) {};
		ROT_RIGHT;
		_delay_ms(ROT_DELAY(90));
		MotorL(STOP);
		MotorR(STOP);
		break;
		case 0b11: //frontal detect
		MotorL(BACKWARD(255));
		MotorR(BACKWARD(255));
		while (lineCheck()) {};
		ROT_RIGHT;
		_delay_ms(ROT_DELAY(180));
		MotorL(STOP);
		MotorR(STOP);
		break;
	}
}

//Fixed Attack Logic
void FAL(uint8_t sensors) {
	do {
		switch (sensors) {
			case 0b01: //right detect
			MotorL(FORWARD(255));
			MotorR(BACKWARD(255));
			break;
			case 0b10: //left detect
			MotorL(BACKWARD(255));
			MotorR(FORWARD(255));
			break;
			case 0b11: //frontal detect
			MotorL(FORWARD(255));
			MotorR(FORWARD(255));
			break;
		}
		sensors = distCheck();
	} while (sensors);
}

//Check distance/presence sensors
uint8_t distCheck() {
#ifdef disableAttack
	return 0;
#else
	uint8_t ret = 0;
	if (readDist(distR))
		ret |= 0b01; //set bit 0 if right sensor found something
	if (readDist(distL))
		ret |= 0b10; //set bit 1 if left sensor found something
	
#ifdef debug
	char buf[50];
	sprintf(buf, "distance sensor: %d\n", ret);
	log(buf);
#endif
	
	return ret;
#endif
}

uint8_t lineCheck() {
#ifdef disableDefense
	return 0;
#else
	uint16_t sensor_r, sensor_l;
	
	ADMUX &= ~(1 << MUX3 | 1 << MUX2  | 1 << MUX1 | 1 << MUX0); //Set ADMUX to left sensor port
	ADCSRA |= (1 << ADSC); //Start analog reading
	while (!(ADCSRA & (1 << ADIF))) {}; //Wait for analog result
	sensor_l = ADC; //Sensor sensor_l value
	
	ADMUX |= (1 << MUX0); //Set ADMUX to right sensor port
	ADCSRA |= (1 << ADSC); //Start analog reading
	while (!(ADCSRA & (1 << ADIF))) {}; //Wait for analog result
	sensor_r = ADC; //Sensor sensor_l value

	uint8_t ret = 0;
	if (sensor_r <= LINE_THRESHOLD)
		ret |= 0b01; //set bit 0 if right sensor found line
	if (sensor_l <= LINE_THRESHOLD)
		ret |= 0b10; //set bit 1 if left sensor found line
		
#ifdef debug
	char buf[50];
	sprintf(buf, "line sensor: %d\n", ret);
	log(buf);
#endif

	return ret;
#endif
}

void setPWML(uint8_t pwm) {
	if (pwm) { //Disable port if pwm equals 0
		DDRB |= (1 << pwmL);
		} else {
		DDRB &= ~(1 << pwmL);
	}
	OCR1A = pwm; //Set pwm value
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
		PORTD &= ~(1 << leftMotor1);
		PORTB &= ~(1 << leftMotor2);
		break;
		case 0x03:
		setPWML(0);
		PORTD |= (1 << leftMotor1);
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
	sprintf(buf, "right motor pwm: %d\n", pwm);
	log(buf);
	sprintf(buf, "right motor state: %d\n", state);
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
		PORTD &= ~(1 << rightMotor1);
		PORTD &= ~(1 << rightMotor2);
		break;
		case 0x03:
		setPWMR(0);
		PORTD |= (1 << rightMotor1);
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