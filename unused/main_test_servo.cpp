#include "mbed.h"

#include "config.h"

#include "DebounceIn.h"
#include "Servo.h"

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition below

int main()
{
    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    // gimbal servo
    Servo gimbalServo(MINI_SEGWAY_SERVO_DOUT);
    float time = 0.0f;

    DigitalOut led1(LED1);
    while (true) {

        time = time + 0.02f;

        if (do_execute_main_task) {

            const float frequency = 0.5f;
            gimbalServo.write( 45.0f * M_PIf / 180.0f * sinf(2.0f * M_PIf * frequency * time) );

        } else {
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                gimbalServo.write(0.0f * M_PIf / 180.0f);
            }
        }

        // printf("servo input: %f\n", servo_input);

        led1 = !led1; // main thread is just blinking the green led on the nucleo
        thread_sleep_for(20);
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
