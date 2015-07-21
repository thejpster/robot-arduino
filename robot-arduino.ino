/*****************************************************
*
* Pi Wars Robot Software (PWRS) Motor Controller
*
* Copyright (c) 2014 Matt Kingston (mattkingston@gmail.com)
* Copyright (c) 2014-2015 Jonathan Pallant (pwrs@thejpster.org.uk)
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
* Arduino sketch. It operates four individual motors, each
* with closed loop feedback control.
*
* Four quadrature inputs are required, on pins
*
* A0 / A4
* A1 / A5
* A2 / A6
* A3 / A7
*
* The four motor outputs are (PWM, Dir A, Dir B)
*
* D3, D2, D4
* D9, D5, D6
* D10, D7, D8
* D11, D12, D13
*
* Note that pins D3, D9, D10 and D11 are PWM enabled.
*****************************************************/

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include <PID_v1.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/

#define DISPLAY_INTERVAL ((microseconds_t) (1000UL * 1000UL))
#define TICK_INTERVAL ((microseconds_t) (100UL * 1000UL))
#define MAX_EDGE_GAP ((microseconds_t) (100UL * 1000UL))

#define K_P 2
#define K_I 5
#define K_D 1

#define ELEMOF(x) (sizeof (x) / sizeof (x)[0])

/*******************************************************************************
 * Types
 ******************************************************************************/

/**
Indicates we're counting in microseconds
*/
typedef unsigned long microseconds_t;

/**
Indicates we're counting in milliseconds
*/
typedef unsigned long milliseconds_t;

/**
Tracks input states. The value indicates what we need /next/ as opposed to
what we already have.
*/
enum read_state_e
{
    READ_STATE_M,
    READ_STATE_X,
    READ_STATE_SIDE,
    READ_STATE_DIR,
    READ_STATE_SPEED_1,
    READ_STATE_SPEED_2,
    READ_STATE_COMMA,
    READ_STATE_COUNT_1,
    READ_STATE_COUNT_2,
    READ_STATE_EOL
};

/**
Contains all the state for a single motor.
*/
struct motor_t
{
    uint8_t remaining;
    double set_speed;
    double read_speed;
    double output;
    const int pinPWM;
    const int pinDIR_A;
    const int pinDIR_B;
    const int pinIn_A;
    const int pinIn_B;
    microseconds_t last_edge;
    PID pid;
};

/**
When we last updated the display.
*/
static microseconds_t last_display;

/**
When we last updated the motor counts.
*/
static microseconds_t last_tick;

/**
State for each motor.
*/
static motor_t motors[] =
{
    {
        0, 0, 0, 0, 3, 2, 4, A0, A4, 0, PID(
        &motors[0].read_speed,
        &motors[0].output,
        &motors[0].set_speed,
        K_P, K_I, K_D, DIRECT)
    },
    {
        0, 0, 0, 0, 9, 5, 6, A1, A5, 0, PID(
        &motors[1].read_speed,
        &motors[1].output,
        &motors[1].set_speed,
        K_P, K_I, K_D, DIRECT)
    },
    {
        0, 0, 0, 0, 10, 7, 8, A2, A6, 0, PID(
        &motors[2].read_speed,
        &motors[2].output,
        &motors[2].set_speed,
        K_P, K_I, K_D, DIRECT)
    },
    {
        0, 0, 0, 0, 11, 12, 13, A3, A7, 0, PID(
        &motors[3].read_speed,
        &motors[3].output,
        &motors[3].set_speed,
        K_P, K_I, K_D, DIRECT)
    },
};

/**
Current input state.
*/
static read_state_e read_state = READ_STATE_M;

/**
Variables for collecting input state before updating a motor_t instance.
*/
static uint8_t speed;
static uint8_t count;
static bool forwards;
static bool left_side;

/*******************************************************************************
 * Functions
 ******************************************************************************/

/***************************************************************************//**
 * The setup function is called once, at startup.
 *
 * We initialise pin directions, some PID parameters, and the quadrature
 * interrupts.
 ******************************************************************************/
void setup()
{
    Serial.begin(115200);
    for(int i = 0; i < ELEMOF(motors); i++)
    {
        motors[i].pid.SetOutputLimits(-255, 255);
        motors[i].pid.SetMode(AUTOMATIC);
        pinMode(motors[i].pinPWM, INPUT);
        pinMode(motors[i].pinDIR_A, INPUT);
        pinMode(motors[i].pinDIR_B, INPUT);
        pinMode(motors[i].pinIn_A, INPUT);
        pinMode(motors[i].pinIn_B, INPUT);
    }
}

/***************************************************************************//**
 * The loop function is called repeatedly by the Arduino C startup code.
 *
 * If enough time has passed, we will update the serial port with status.
 *
 * If enough time has passed, we will also reduce each motor tick count by one.
 * If a motor gets to zero, it is stopped. This prevents the controller running
 * on when the main PWRS application goes away for some reason.
 ******************************************************************************/
void loop()
{
    microseconds_t this_time = micros();
    microseconds_t delta;
    delta = this_time - last_display;
    if (delta > DISPLAY_INTERVAL)
    {
        print_status();
        last_display = this_time;
    }

    delta = this_time - last_tick;
    if (delta > TICK_INTERVAL)
    {
        update_motors();
        last_tick = this_time;
    }

    for(int i = 0; i < ELEMOF(motors); i++)
    {
#if USE_PID_MODE
        motors[i].pid.Compute();
        if (motors[i].output > 0)
        {
            digitalWrite(motors[i].pinDIR_A, 1);
            digitalWrite(motors[i].pinDIR_B, 0);
            analogWrite(motors[i].pinPWM, motors[i].output);
        }
        else
        {
            digitalWrite(motors[i].pinDIR_A, 0);
            digitalWrite(motors[i].pinDIR_B, 1);
            analogWrite(motors[i].pinPWM, -motors[i].output);
        }
#else
        if (motors[i].set_speed > 0)
        {
            digitalWrite(motors[i].pinDIR_A, 1);
            digitalWrite(motors[i].pinDIR_B, 0);
            analogWrite(motors[i].pinPWM, motors[i].set_speed);
        }
        else
        {
            digitalWrite(motors[i].pinDIR_A, 0);
            digitalWrite(motors[i].pinDIR_B, 1);
            analogWrite(motors[i].pinPWM, -motors[i].set_speed);
        }
#endif
    }

    get_instruction();
}

void get_instruction()
{
    while (Serial.available())
    {
        char c = Serial.read();

#if DEBUG_MODE
        Serial.print("Checking ");
        Serial.print(c);
        Serial.print(" in ");
        Serial.println(read_state, DEC);
#endif

        switch (read_state)
        {
        case READ_STATE_M:
            if (c == 'M')
            {
                speed = 0;
                count = 0;
                read_state = READ_STATE_X;
            }

            break;

        case READ_STATE_X:
            if (c == 'X')
            {
                read_state = READ_STATE_SIDE;
            }

            break;

        case READ_STATE_SIDE:
            if (c == 'L')
            {
                left_side = true;
                read_state = READ_STATE_DIR;
            }
            else if (c == 'R')
            {
                left_side = false;
                read_state = READ_STATE_DIR;
            }
            else
            {
                read_state = READ_STATE_M;
            }

            break;

        case READ_STATE_DIR:
            if (c == '+')
            {
                forwards = true;
                read_state = READ_STATE_SPEED_1;
            }
            else if (c == '-')
            {
                forwards = false;
                read_state = READ_STATE_SPEED_1;
            }
            else
            {
                read_state = READ_STATE_M;
            }

            break;

        case READ_STATE_SPEED_1:
            if (parse_hex(c, &speed))
            {
                read_state = READ_STATE_SPEED_2;
                speed <<= 4;
            }
            else
            {
                speed = 0;
                read_state = READ_STATE_M;
            }

            break;

        case READ_STATE_SPEED_2:
            if (parse_hex(c, &speed))
            {
                read_state = READ_STATE_COMMA;
            }
            else
            {
                speed = 0;
                read_state = READ_STATE_M;
            }

            break;

        case READ_STATE_COMMA:
            if (c == ',')
            {
                read_state = READ_STATE_COUNT_1;
            }
            else
            {
                read_state = READ_STATE_M;
            }

            break;

        case READ_STATE_COUNT_1:
            if (parse_hex(c, &count))
            {
                read_state = READ_STATE_COUNT_2;
                count <<= 4;
            }
            else
            {
                count = 0;
                read_state = READ_STATE_M;
            }

            break;

        case READ_STATE_COUNT_2:
            if (parse_hex(c, &count))
            {
                read_state = READ_STATE_EOL;
            }
            else
            {
                count = 0;
                read_state = READ_STATE_M;
            }

            break;

        case READ_STATE_EOL:
            if ((c == '\n') || (c == '\r'))
            {
                process_command(speed, forwards, left_side, count);
                read_state = READ_STATE_M;
            }

            break;
        }
    }
}

static bool parse_hex(char c, uint8_t* p_value)
{
    bool result = true;

    if ((c >= '0') && (c <= '9'))
    {
        *p_value |= (c - '0');
    }
    else if ((c >= 'A') && (c <= 'F'))
    {
        *p_value |= (c - 'A') + 10;
    }
    else if ((c >= 'a') && (c <= 'f'))
    {
        *p_value |= (c - 'a') + 10;
    }
    else
    {
        result = false;
    }

    return result;
}

static void process_command(
    uint8_t speed,
    bool is_forward,
    bool is_left,
    uint8_t count
)
{
    motor_t* p_motor;
    if (is_left)
    {
        p_motor = &motors[0];
    }
    else
    {
        p_motor = &motors[1];
    }

    if (is_forward)
    {
        p_motor->set_speed = speed;
    }
    else
    {
        p_motor->set_speed = -((double) speed);
    }
    p_motor->remaining = count;
    print_status();
}

static void update_motors()
{
    for(int i = 0; i < ELEMOF(motors); i++)
    {
        motor_t* p_motor = &motors[i];
        if (p_motor->remaining)
        {
            p_motor->remaining--;
            if (p_motor->remaining == 0)
            {
                p_motor->set_speed = 0;
            }
        }
    }
}

static void print_status()
{
    for(int i = 0; i < ELEMOF(motors); i++)
    {
        motor_t* p_motor = &motors[i];
        Serial.print('M');
        Serial.print('I');
        Serial.print(i ? 'R' : 'L');
        if (p_motor->output > 0)
        {
            Serial.print('+');
        }
        else
        {
            Serial.print('-');
        }
        Serial.print(p_motor->output, DEC);
        Serial.print(',');
        Serial.print(p_motor->read_speed, DEC);
        Serial.print(',');
        Serial.print(p_motor->set_speed, DEC);
        Serial.print(',');
        Serial.println(p_motor->remaining, DEC);
    }
}


// Fires on A0 to A3 change
ISR(PCINT1_vect)
{
    // Read A0, A1, A2 and A3 to see if any have changed
    // If so, stash the timing delta
    // Also check 2, 4, 5 and 6 to verify rotation direction
    // Same level = +ve, different level = -ve (or possibly the other way around)

    // We should use PCMSK2 to limit this to A0..A3
}

/*******************************************************************************
 * End of File
 ******************************************************************************/
