#include "MiniSegway.h"

MiniSegway::MiniSegway()
{
    // start thread
    _Thread.start(callback(this, &MiniSegway::threadTask));

    // attach sendThreadFlag() to ticker so that sendThreadFlag() is called periodically, which signals the thread to execute
    _Ticker.attach(callback(this, &MiniSegway::sendThreadFlag), std::chrono::microseconds{PERIOD_MUS});
}

MiniSegway::~MiniSegway()
{
}

void MiniSegway::threadTask()
{
    // DigitalOut user_led(LED1);
    // user_led = 0;

    // const unsigned loops_per_seconds = static_cast<int>(ceilf(1.0f / (1.0e-6f * static_cast<float>(PERIOD_MUS))));
    // const unsigned log_time = 3; // sec
    // unsigned cntr = 0;

    // SerialStream serialStream(NUM_OF_FLOATS_MAX, TX, RX, BAUDRATE);

    // Timer timer;
    // int time_previous_us = 0;
    // timer.start();

    // DigitalIn mechanical_button(BUTTON);
    // mechanical_button.mode(PullUp);
    // bool do_execute = false;

    PpmIn ppmIn(PPM_IN);

    // give the logger 1000 msec time to start
    thread_sleep_for(1000);

    while (true) {
        ThisThread::flags_wait_any(_ThreadFlag);

//         const int time_us = std::chrono::duration_cast<std::chrono::microseconds>(timer.elapsed_time()).count();
//         const int dtime_mus = time_us - time_previous_us;
//         time_previous_us = time_us;
//         float dtime_mus_f = static_cast<float>(dtime_mus);

//         if ((!do_execute) && (mechanical_button.read() || serialStream.isStartByteReceived())) {
//             do_execute = true;
//         }

//         // write data to serial for log_time seconds
//         // first coloumn is delta time in micro seconds
//         // second to n-th coloumn is just an increasing number by 1
//         if (do_execute && (cntr < log_time * loops_per_seconds)) {

//             // make the led blink every second while logging is in process
//             if (cntr++ % loops_per_seconds == 0)
//                 user_led = !user_led;

// // UNCOMMENTED            serialStream.write(dtime_mus_f);
//             for (unsigned i = 1; i < NUM_OF_FLOATS_MAX; i++) {
//                 const float val = static_cast<float>((cntr - 1) * NUM_OF_FLOATS_MAX + i + 1);
// // UNCOMMENTED                serialStream.write(val);
//                 // // test if we can also send less than NUM_OF_FLOATS using send()
//                 // const float val = static_cast<float>((cntr - 1) * (NUM_OF_FLOATS - 1) + i + 1);
//                 // if (i <= NUM_OF_FLOATS - 2)
//                 //     serialStream.write(val);
//                 // if (i == NUM_OF_FLOATS - 2)
//                 //     serialStream.send();
//             }
//         }

        // ppm
        for (uint8_t i = 0; i < ppmIn.getNumOfChannels(); i++) {
            printf("ch %d: %d musec ", i + 1, ppmIn.getChannelMus(i));
        }

        if (ppmIn.isDataValid())
            printf("data valid, ");
        else
            printf("data invalid, ");
        printf("period: = %d musec\n", ppmIn.period());

        if (ppmIn.isLow(4)) // 2 way switch
            printf("ch5 low, ");
        else if (ppmIn.isHigh(4))
            printf("ch5 high, ");
        else if (ppmIn.isCenter(4))
            printf("ch5 center, ");
        else printf("ch5 0, ");

        if (ppmIn.isLow(6)) // 3 way switch
            printf("ch7 low");
        else if (ppmIn.isHigh(6))
            printf("ch7 high");
        else if (ppmIn.isCenter(6))
            printf("ch7 center");
        else
            printf("ch7 0");
        printf("\n");
        printf("Roll = %d, Pitch = %d, Throttle = %d, Yaw = %d\n",
               static_cast<int>(1.0e3f * ppmIn.getChannelMinusToPlusOne(0)),
               static_cast<int>(1.0e3f * ppmIn.getChannelMinusToPlusOne(1)),
               static_cast<int>(1.0e3f * ppmIn.getChannelZeroToPlusOne(2)),
               static_cast<int>(1.0e3f * ppmIn.getChannelMinusToPlusOne(3)));

    }
}

void MiniSegway::sendThreadFlag()
{
    // set the thread flag to trigger the thread task
    _Thread.flags_set(_ThreadFlag);
}

