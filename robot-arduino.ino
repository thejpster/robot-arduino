/*******************************************************************************
*
* Pi Wars Robot Software (PWRS) Motor Controller
*
* Copyright (c) 2014 Matt Kingston (mattkingston@gmail.com)
* Copyright (c) 2014-2016 Jonathan 'theJPster' Pallant (pwrs@thejpster.org.uk)
*
* PWRS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* PWRS is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with PWRS.  If not, see <http://www.gnu.org/licenses/>.
*
* This is the motor controller software, in the form of an
* Arduino sketch.
*
* Messages look like this:
*
* COMMAND DATA_LEN <DATA> CRC
*
* Message are then SLIP encoded for transmission over the UART.
*
* Frame Start/End: MESSAGE_HEADER
* MESSAGE_HEADER => MESSAGE_ESC MESSAGE_ESC_HEADER
* MESSAGE_ESC    => MESSAGE_ESC MESSAGE_ESC_ESC
*
******************************************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/
// None

/*******************************************************************************
 * Defines / Constants
 ******************************************************************************/


#define MESSAGE_HEADER             0xC0
#define MESSAGE_ESC                0xDB
#define MESSAGE_ESC_HEADER         0xDC
#define MESSAGE_ESC_ESC            0xDD

#define MAX_MESSAGE_LEN 254

#define SERIAL_RX  0
#define SERIAL_TX  1

#define QUAD1      2
#define PWM1       3 // PWM
#define QUAD2      4
#define PWM2       5 // PWM
#define PWM3       6 // PWM
#define QUAD4      7
#define QUAD3      8
#define PWM4       9 // PWM

#define DIR12     10 // PWM
#define DIR34     11 // PWM

// #define UNUSED 12

#define SONAR_A   A0
#define SONAR_B   A1
#define SONAR_C   A2

// #define UNUSED A3

#define CUR1      A4
#define CUR2      A5
// A6/A7 are analog inputs only
#define CUR3      A6
#define CUR4      A7

// Max distance (in cm)
#define MAX_DISTANCE_CM 100

#define ELEMOF(x) (sizeof (x) / sizeof (x)[0])

#define MAX_SPEED 320

#define NUM_SONARS 3
// Max uS to wait for a sensor to start (guards against being
// unplugged).
#define MAX_SENSOR_WAIT 5800


/*******************************************************************************
 * Data Types
 ******************************************************************************/

/**
Indicates we're counting in microseconds
*/
typedef uint32_t microseconds_t;

// The different commands we support
typedef enum motor_command_t
{
	MESSAGE_COMMAND_SPEED_REQ,
	MESSAGE_COMMAND_SPEED_IND,
	MESSAGE_COMMAND_CURRENT_OVERFLOW_IND,
	MESSAGE_COMMAND_CURRENT_IND,
	MESSAGE_COMMAND_RANGE_IND,
	MAX_VALID_COMMAND
} motor_command_t;

typedef struct message_speed_req_t
{
	uint32_t ctx;
	uint8_t side; // 0 = left, 1 = right
	uint8_t clicks; // max clicks to travel
	int16_t speed; // clicks per second
} message_speed_req_t;

typedef struct message_speed_ind_t
{
	uint16_t speed;
	uint8_t motor;
} message_speed_ind_t;

typedef struct message_range_ind_t
{
	uint16_t range;
	uint8_t sensor;
} message_range_ind_t;

typedef struct message_current_ind_t
{
	uint16_t current;
	uint8_t motor;
} message_current_ind_t;

typedef struct rx_message_t
{
	motor_command_t command;
	size_t data_len;
	size_t data_read;
	uint8_t data[MAX_MESSAGE_LEN];
} rx_message_t;

typedef enum read_state_t
{
	READ_STATE_IDLE,
	READ_STATE_COMMAND,
	READ_STATE_LEN,
	READ_STATE_DATA,
	READ_STATE_CHECKSUM,
} read_state_t;

// A number of clicks (from the quadrature encoder logic on the controller
// board) per second
typedef uint16_t cps_t;

// The robot can go forwards or in reverse
typedef enum direction_t
{
	FORWARD,
	REVERSE
} direction_t;

// Represents a motor we try and control, with a current speed,
// a click count, and a pin to PWM.
class Motor
{
public:
	Motor(int index, int pwm_pin, int current_pin, int irq_pin);
	void process(direction_t dir, microseconds_t delta, cps_t target_speed);
	void report();
	void tick();
	void halt();
	uint16_t get_current();
private:
	int m_index;
	int m_pwm_pin;
	int m_current_pin;
	int m_irq_pin;
	cps_t m_current_speed;
	volatile uint8_t m_counter;
};

// Each side has two motors, which run independently but we always drive them at
// the same target speed and direction, otherwise the belt would snap.
class Side
{
public:
	Side(int index, int front_pwm, int front_current_pin, int front_irq,
	     int rear_pwm, int rear_current, int rear_irq, int dir_pin);
	void process(microseconds_t delta);
	void tick_front() { m_front.tick(); };
	void tick_rear() { m_rear.tick(); };
	void set_speed(direction_t dir, cps_t speed);
	bool current_overflow();
	void halt();
private:
	Motor m_front;
	Motor m_rear;
	cps_t m_target_speed;
	direction_t m_dir;
	int m_dir_pin;
};

/*******************************************************************************
 * Global Data
 ******************************************************************************/


// Update time in microseconds
const microseconds_t MAX_GAP = 50UL * 1000UL;

/* Empirically, the motors don't seem to exceed this */
/* 1 Amp in units of 4.9 mA */
const uint16_t MAX_CURRENT = 1000 / 4.9;

// Time in microseconds at the start of the measurement
static microseconds_t start = 0;

// For doing periodic things in the main loop
static uint16_t step_counter = 0;

// The left side of the robot
static Side left(0, PWM1, CUR1, QUAD1, PWM2, CUR2, QUAD2, DIR12);

// The right side of the robot
static Side right(2, PWM3, CUR3, QUAD3, PWM4, CUR4, QUAD4, DIR34);

// Message being received
static rx_message_t rx_message;
static read_state_t read_state = READ_STATE_IDLE;

static uint8_t current_sonar = 0;

static const int sonar_pins[NUM_SONARS] =
{
	SONAR_A,	SONAR_B,
	SONAR_C
};

static microseconds_t ping_start;
static bool ping_running = false;

/*******************************************************************************
 * Functions
 ******************************************************************************/

static void pin_irq1();
static void pin_irq2();
static void pin_irq3();
static void pin_irq4();
static void get_instruction();

static uint8_t calc_checksum(const rx_message_t* p_message);
static void process_rx_message(const rx_message_t* p_message);
static void process_rx_byte(uint8_t byte);
static void send_message(motor_command_t command, size_t data_len, const uint8_t* p_data);

/*******************************************************************************
 * Primary Functions
 ******************************************************************************/

void setup()
{
	Serial.begin(115200);
	// Not required at this time
	// enableInterrupt(QUAD1, pin_irq1, CHANGE);
	// enableInterrupt(QUAD2, pin_irq2, CHANGE);
	// enableInterrupt(QUAD3, pin_irq3, CHANGE);
	// enableInterrupt(QUAD4, pin_irq4, CHANGE);
	start = micros();
}

void loop()
{
	microseconds_t now = micros();
	microseconds_t delta = now - start;
	if (delta > MAX_GAP)
	{
		// Process left
		left.process(delta);
		if (left.current_overflow())
		{
			left.halt();
			right.halt();
			send_message(MESSAGE_COMMAND_CURRENT_OVERFLOW_IND, 0, nullptr);
			delay(5000);
		}
		// Process right (more time has elapsed)
		delta = micros() - start;
		right.process(delta);
		if (right.current_overflow())
		{
			left.halt();
			right.halt();
			send_message(MESSAGE_COMMAND_CURRENT_OVERFLOW_IND, 0, nullptr);
			delay(5000);
		}

		if (ping_running)
		{
			// Timed out...
			send_range(current_sonar, 65535);
		}

		// Pick next sonar
		if (++current_sonar >= NUM_SONARS)
		{
			current_sonar = 0;
		}

		// Trigger a ping by lifting high for 10us
		pinMode(sonar_pins[current_sonar], OUTPUT);
		digitalWrite(sonar_pins[current_sonar], 1);
		delayMicroseconds(10);
		digitalWrite(sonar_pins[current_sonar], 0);
		ping_start = micros();
		pinMode(sonar_pins[current_sonar], INPUT);
		ping_running = true;
		// Spin until pin goes high (start of ping)
		while (!digitalRead(sonar_pins[current_sonar]))
		{
			// Spin
			microseconds_t delta = micros() - ping_start;
			if (delta > MAX_SENSOR_WAIT)
			{
				// Not found
				send_range(current_sonar, 0);
				ping_running = false;
				break;
			}
		}

		// Reset loop delay
		start = now;
	}

	if (ping_running)
	{
		if (!digitalRead(sonar_pins[current_sonar]))
		{
			microseconds_t delta = micros() - ping_start;
			uint16_t range = (uint16_t) (delta > 65535 ? 65535 : delta);
			send_range(current_sonar, range);
			ping_running = false;
		}
	}

	get_instruction();
}

/*******************************************************************************
 * Other Functions
 ******************************************************************************/


void send_range(uint8_t sensor, uint16_t range_us)
{
	message_range_ind_t ind =
	{
		.range = range_us,
		.sensor = sensor,
	};
	send_message(MESSAGE_COMMAND_RANGE_IND, sizeof(ind), reinterpret_cast<const uint8_t*>(&ind));
}

Side::Side(int index, int front_pwm, int front_current_pin, int front_irq, int rear_pwm, int rear_current, int rear_irq, int dir_pin) :
	m_front(index, front_pwm, front_current_pin, front_irq),
	m_rear(index + 1, rear_pwm, rear_current, rear_irq),
	m_dir_pin(dir_pin),
	m_dir(FORWARD),
	m_target_speed(0)
{
	pinMode(dir_pin, OUTPUT);
	digitalWrite(dir_pin, HIGH);
}

void Side::process(microseconds_t delta)
{
	m_front.process(m_dir, delta, m_target_speed);
	m_rear.process(m_dir, delta, m_target_speed);
	if ((m_dir == FORWARD) && (m_target_speed != 0))
	{
		digitalWrite(m_dir_pin, HIGH);
	}
	else
	{
		digitalWrite(m_dir_pin, LOW);
	}
	m_front.report();
	m_rear.report();
}

bool Side::current_overflow()
{
	if (m_front.get_current() > MAX_CURRENT)
	{
		return true;
	}
	if (m_rear.get_current() > MAX_CURRENT)
	{
		return true;
	}
	return false;
}

void Side::halt()
{
	m_target_speed = 0;
	m_front.halt();
	m_rear.halt();
}

void Side::set_speed(direction_t dir, cps_t cps)
{
	m_dir = dir;
	m_target_speed = cps > MAX_SPEED ? MAX_SPEED : cps;
}

Motor::Motor(int index, int pwm_pin, int current_pin, int irq_pin) :
	m_index(index),
	m_pwm_pin(pwm_pin),
	m_current_pin(current_pin),
	m_irq_pin(irq_pin)
{
	// Turn motor off
	analogWrite(m_pwm_pin, 0);
	// pullup on
	pinMode(irq_pin, INPUT);
	digitalWrite(irq_pin, HIGH);
	// pullup off
	pinMode(current_pin, INPUT);
	digitalWrite(current_pin, LOW);
}

void Motor::report()
{
	// message_speed_ind_t ind =
	// {
	// 	.speed = m_current_speed,
	// 	.motor = (uint8_t) m_index,
	// };
	// send_message(MESSAGE_COMMAND_SPEED_IND, sizeof(ind), reinterpret_cast<const uint8_t*>(&ind));
}

void Motor::process(direction_t dir, microseconds_t delta, cps_t target)
{
	const uint8_t counter_copy = m_counter;
	m_counter = 0;
	m_current_speed = (1000UL * 1000UL * counter_copy) / delta;
	uint32_t speed = (((uint32_t) target) * 255) / MAX_SPEED;
	analogWrite(m_pwm_pin, speed > 255 ? 255 : speed);
}

void Motor::tick()
{
	m_counter++;
}

void Motor::halt()
{
	analogWrite(m_pwm_pin, 0);
	m_current_speed = 0;
	m_counter = 0;
}

/**
 * Returns the amount of current used by this motor. Takes 100us to execute.
 * Reads in units of 4.9mA.
 */
uint16_t Motor::get_current()
{
	uint16_t current = analogRead(m_current_pin);
	message_current_ind_t ind = {current, (uint8_t) m_index};
	send_message(MESSAGE_COMMAND_CURRENT_IND, sizeof(ind), reinterpret_cast<const uint8_t*>(&ind));
}

/**
 * The encoder runs at 333.3 clicks per revolution and the motor can't
 * run at more than 3 revolutions per second, so we've got at least 1 millisecond
 * in between clicks. This is for motor 1.
 */
static void pin_irq1()
{
	left.tick_front();
}

/**
 * This is for motor 2. See `pin_irq1`.
 */
static void pin_irq2()
{
	left.tick_rear();
}

/**
 * This is for motor 3. See `pin_irq1`.
 */
static void pin_irq3()
{
	right.tick_front();
}

/**
 * This is for motor 4. See `pin_irq1`.
 */
static void pin_irq4()
{
	right.tick_rear();
}

/**
 * Read from the serial port and update the motor state depending on the command
 * received.
 */
static void get_instruction()
{
	static bool is_escape = false;
	while (Serial.available())
	{
		uint8_t data = Serial.read();
		if (is_escape)
		{
			if (data == MESSAGE_ESC_HEADER)
			{
				// Escaped header => process normally
				process_rx_byte(MESSAGE_HEADER);
			}
			else if (data == MESSAGE_ESC_ESC)
			{
				process_rx_byte(MESSAGE_ESC);
			}
			else
			{
				// printf("Bad escape 0x%02x\r\n", data);
			}
			is_escape = false;
		}
		else if (data == MESSAGE_ESC)
		{
			is_escape = true;
		}
		else if (data == MESSAGE_HEADER)
		{
			// Unescaped header => start of message
			read_state = READ_STATE_COMMAND;
		}
		else
		{
			process_rx_byte(data);
		}
	}
}

/**
 * Calculate the message checksum.
 *
 * XORs the all the bytes in the given message.
 *
 * @return the calculated checksum
 */
static uint8_t calc_checksum(const rx_message_t* p_message)
{
	uint8_t result = 0xFF;
	result ^= (uint8_t) p_message->command;
	result ^= (uint8_t) p_message->data_len;
	for (size_t i = 0; i < p_message->data_len; i++)
	{
		result ^= p_message->data[i];
	}
	return result;
}

/**
 * Process the message from the controller. Currently just
 * does some logging. Checksums have already been verified at
 * this stage.
 *
 * @param[in] p_message The received message
 */
static void process_rx_message(const rx_message_t* p_message)
{
	switch (p_message->command)
	{
	case MESSAGE_COMMAND_SPEED_REQ:
		if (p_message->data_len == sizeof(message_speed_req_t))
		{
			const message_speed_req_t* p_req = reinterpret_cast<const message_speed_req_t*>(p_message->data);
			direction_t dir = (p_req->speed < 0) ? FORWARD : REVERSE;
			cps_t speed = abs(p_req->speed);
			if (p_req->side == 0)
			{
				left.set_speed(dir, speed);
			}
			else if (p_req->side == 1)
			{
				right.set_speed(dir, speed);
			}
		}
		break;
	}
}


/**
 * Feed incoming bytes through the state machine.
 * Will call process_rx_message() when a valid message
 * has been received.
 *
 * @param[in] byte The received byte
 */
static void process_rx_byte(uint8_t byte)
{
	switch (read_state)
	{
	case READ_STATE_IDLE:
		break;
	case READ_STATE_COMMAND:
		if (byte <= MAX_VALID_COMMAND)
		{
			rx_message.command = (motor_command_t) byte;
			read_state = READ_STATE_LEN;
		}
		else
		{
			read_state = READ_STATE_IDLE;
		}
		break;
	case READ_STATE_LEN:
		rx_message.data_read = 0;
		rx_message.data_len = byte;
		read_state = rx_message.data_len ? READ_STATE_DATA : READ_STATE_CHECKSUM;
		break;
	case READ_STATE_DATA:
		rx_message.data[rx_message.data_read++] = byte;
		if (rx_message.data_read == rx_message.data_len)
		{
			read_state = READ_STATE_CHECKSUM;
		}
		break;
	case READ_STATE_CHECKSUM:
		if (byte == calc_checksum(&rx_message))
		{
			process_rx_message(&rx_message);
		}
		else
		{
			// printf("Dropping bad packet\r\n");
		}
		read_state = READ_STATE_IDLE;
		break;
	}
}

static void write_esc(uint8_t data)
{
	if (data == MESSAGE_ESC)
	{
		Serial.write(MESSAGE_ESC);
		Serial.write(MESSAGE_ESC_ESC);
	}
	else if (data == MESSAGE_HEADER)
	{
		Serial.write(MESSAGE_ESC);
		Serial.write(MESSAGE_ESC_HEADER);
	}
	else
	{
		Serial.write(data);
	}
}

static void send_message(motor_command_t command, size_t data_len, const uint8_t* p_data)
{
	uint8_t csum = 0xFF;

	if (data_len >= 256)
	{
		return;
	}

	Serial.write(MESSAGE_HEADER);

	write_esc((uint8_t) command);
	csum ^= (uint8_t) command;

	write_esc((uint8_t) data_len);
	csum ^= (uint8_t) data_len;

	for (size_t i = 0; i < data_len; i++)
	{
		write_esc(p_data[i]);
		csum ^= (uint8_t) p_data[i];
	}

	write_esc(csum);
}

/*******************************************************************************
 * End of File
 ******************************************************************************/
