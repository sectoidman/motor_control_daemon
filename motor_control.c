#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <gpiod.h>
#include <sched.h>
#include <signal.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

// azimuth control motor outputs
#define GPIO_DIR_PIN 13   // Direction GPIO (clockwise/counter-clockwise)
#define GPIO_PWM_PIN 19   // PWM GPIO pin
#define GPIO_ENABLE_PIN 26 // Enable GPIO pin

// fire-control outputs
#define GPIO_SPINUP_PIN 3
#define GPIO_FEED_PIN 4

// fire-control timing
#define SPINUP_TIME_NS 400000000 // 100 ms

// Stepper motor parameters
#define STEPS_PER_REV 6400  // Steps per full revolution
#define PWM_DUTY_CYCLE 5000 // min pulse width for TB6600 is ~2us?
#define INITIAL_PWM_PERIOD 1000000000
#define MAX_RPM 20.0

// cycle timestep
#define PERIOD_NS 10000000L // 10 ms

// gamepad
#define JOYSTICK_AXIS_MAX 32767.0
#define JOYSTICK_DEADBAND 2000.0
#define BUTTON_X 0
#define BUTTON_CIRCLE 1
#define BUTTON_TRIANGLE 2
#define BUTTON_SQUARE 3
#define BUTTON_L1 4
#define BUTTON_R1 5
#define BUTTON_L2 6
#define BUTTON_R2 7

typedef struct gamepad_data {
    double joystick_x_axis_normal;
    bool drive_enable;
    bool firing_command;
} gamepad_data;

// fire-control state machine states
typedef enum {NOT_FIRING, SPINUP, FIRING} FiringState;


struct gpiod_chip *chip = NULL;

// prototypes
void pwm_enable(bool enable);
void drive_enable(bool enable);
void spinup_enable(bool enable);
void feed_enable(bool enable);
void set_pwm_period(int period);
void setup_pwm();
void setup_gpio(struct gpiod_chip *chip);
void teardown_gpio();
void register_signal_handlers();
void get_gamepad_commands(int js_fd, gamepad_data* outputs);
void control_motor(int speed);
void timespec_add_ns(struct timespec *t, long ns);

///////////////////////////
// GPIO / Peripheral ifaces
///////////////////////////
void pwm_enable(bool enable)
{
    // Enable PWM signal
    FILE *pwm_enable_file = fopen("/sys/class/pwm/pwmchip0/pwm1/enable", "w");
    if (pwm_enable_file == NULL) {
        perror("Failed to enable PWM");
        exit(1);
    }

    if (enable)
    {
        fprintf(pwm_enable_file, "1");
    }
    else
    {
        fprintf(pwm_enable_file, "0");
    }
    fclose(pwm_enable_file);
}


void drive_enable(bool enable)
{
    struct gpiod_line *enable_line = gpiod_chip_get_line(chip, GPIO_ENABLE_PIN);
    int motor_enable = enable ? 0 : 1; // active low
    if (gpiod_line_set_value(enable_line, motor_enable) < 0)
    {
        perror("drive enable set failed");
        teardown_gpio();
    }
}


void spinup_enable(bool enable)
{
    struct gpiod_line *spinup_line = gpiod_chip_get_line(chip, GPIO_SPINUP_PIN);
    int spin_enable = enable ? 0 : 1; // active low
    if (gpiod_line_set_value(spinup_line, spin_enable) < 0)
    {
        perror("spinup enable set failed");
        teardown_gpio();
    }
}


void feed_enable(bool enable)
{
    struct gpiod_line *feed_line = gpiod_chip_get_line(chip, GPIO_FEED_PIN);
    int feed_enable = enable ? 0 : 1; // active low
    if (gpiod_line_set_value(feed_line, feed_enable) < 0)
    {
        perror("feed enable set failed");
        teardown_gpio();
    }
}


void set_pwm_period(int period)
{
    FILE *pwm_period_file = fopen("/sys/class/pwm/pwmchip0/pwm1/period", "w");
    if (pwm_period_file == NULL) {
        perror("Failed to set PWM period");
        teardown_gpio();
    }
    fprintf(pwm_period_file, "%d", period);
    fclose(pwm_period_file);
}


///////////////////
// Init and teardown
//////////////////

// Function to configure PWM
void setup_pwm() {
    // Enable PWM
    FILE *pwm_export = fopen("/sys/class/pwm/pwmchip0/export", "w");
    if (pwm_export == NULL) {
        perror("Failed to export PWM pin");
        exit(1);
    }
    fprintf(pwm_export, "1"); // Export PWM 1 pin
    fclose(pwm_export);

    // disable PWM out for now (motor should hold position)
    pwm_enable(false);

    // Set PWM duty cycle
    FILE *pwm_dc_file = fopen("/sys/class/pwm/pwmchip0/pwm1/duty_cycle", "w");
    if (pwm_dc_file == NULL) {
        perror("Failed to set PWM duty cycle");
        exit(1);
    }
    fprintf(pwm_dc_file, "%d", PWM_DUTY_CYCLE);
    fclose(pwm_dc_file);

    // Set PWM period
    set_pwm_period(INITIAL_PWM_PERIOD);
}

// Initialize GPIO pins using libgpiod
void setup_gpio(struct gpiod_chip *chip) {
    struct gpiod_line *dir_line, *enable_line, *spinup_line, *feed_line;

    // Get lines for direction (13) and enable (26)
    dir_line = gpiod_chip_get_line(chip, GPIO_DIR_PIN);
    enable_line = gpiod_chip_get_line(chip, GPIO_ENABLE_PIN);
    spinup_line = gpiod_chip_get_line(chip, GPIO_SPINUP_PIN);
    feed_line = gpiod_chip_get_line(chip, GPIO_FEED_PIN);


    if (!dir_line || !enable_line || !spinup_line || !feed_line) {
        perror("Failed to get GPIO lines");
        exit(1);
    }

    // Request the lines for output
    if (gpiod_line_request_output(dir_line, "motor_control", 0) < 0) {
        perror("Failed to request direction line");
        exit(1);
    }
    if (gpiod_line_request_output(enable_line, "motor_control", 0) < 0) { // Enable motor (active low)
        perror("Failed to request enable line");
        exit(1);
    }
    // relays active-low, start disabled.
    if (gpiod_line_request_output(spinup_line, "fire_control", 1) < 0) {
        perror("Failed to request spinup line");
        exit(1);
    }
    if (gpiod_line_request_output(feed_line, "fire_control", 1) < 0) {
        perror("Failed to request feed line");
        exit(1);
    }
}

void teardown_gpio()
{
    drive_enable(false);
    // Close the GPIO chip before exiting
    gpiod_chip_close(chip);
    exit(0);
}

void register_signal_handlers()
{
    static struct sigaction siga;
    siga.sa_handler = teardown_gpio;
    for (int sig = 1; sig <= SIGRTMAX; ++sig) {
        // this might return -1 and set errno, but we don't care
        sigaction(sig, &siga, NULL);
    }
}

void get_gamepad_commands(int js_fd, gamepad_data* outputs)
{
    struct js_event e;
    static int x_axis = 0; // static so we persist the last value
    static bool drive_state = false;

    while (read(js_fd, &e, sizeof(e)) > 0)
    {
        if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS && e.number == 0)
        {
            x_axis = e.value; // X-axis of left stick
        }

        if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON && e.number == BUTTON_CIRCLE && e.value == 1)
        {
            // O button pressed
            // toggle the drive enable.
            drive_state = !drive_state;
            outputs->drive_enable = drive_state;
#if defined(DEBUG)
            printf("Joystick Circle Btn pressed");
#endif
        }

        if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON && (e.number == BUTTON_R2 || e.number == BUTTON_L2) && e.value == 1)
        {
            outputs->firing_command = true;
#if defined(DEBUG)
            printf("Joystick Trigger pressed.\n");
#endif
        }

        if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON && (e.number == BUTTON_R2 || e.number == BUTTON_L2) && e.value == 0)
        {
            outputs->firing_command = false;
#if defined(DEBUG)
            printf("Joystick Trigger released.\n");
#endif
        }

    }

    // exponential smoothing low-pass filter
    static const double alpha = 0.2;
    static double x_axis_smoothed = 0.0;
    x_axis_smoothed = alpha * x_axis + (1.0 - alpha) * x_axis_smoothed;

    // deadband - ignore movement commands less than 2%
    if (fabs(x_axis_smoothed) > JOYSTICK_DEADBAND)
    {
        outputs->joystick_x_axis_normal = x_axis_smoothed / JOYSTICK_AXIS_MAX;
#if defined(DEBUG)
    printf("Joystick X-axis: %.3f\n", outputs->joystick_x_axis_normal);
#endif
    }
}


// Convert RPM speed command to PWM and direction GPIO settings
void control_motor(int speed) {
    struct gpiod_line *dir_line;
    dir_line = gpiod_chip_get_line(chip, GPIO_DIR_PIN);

    // Set period
    int pwm_period = (int)(((60.0 / abs(speed)) / STEPS_PER_REV) * 1e9);

    // Set direction
    if (speed < 0) {
        set_pwm_period(pwm_period);
        gpiod_line_set_value(dir_line, 1); // Counter-clockwise (high)
        pwm_enable(true);
    } else if (speed > 0) {
        set_pwm_period(pwm_period);
        gpiod_line_set_value(dir_line, 0); // Clockwise (low)
        pwm_enable(true);
    }
    else // 0
    {
        pwm_enable(false);
    }
}


// Handle firing sequence
void control_firing(bool fire_command)
{
    static FiringState state = NOT_FIRING;
    static struct timespec start;

    switch (state)
    {
        case NOT_FIRING:
        {
            if (fire_command)
            {
                state = SPINUP;
                clock_gettime(CLOCK_MONOTONIC, &start);
                spinup_enable(true);
#if defined(DEBUG)
                printf("Entering spinup state.\n");
#endif
            }
            break;
        }

        case SPINUP:
        {
            // wait for the throwers to get up to speed
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long ns_elapsed = (now.tv_sec - start.tv_sec) * 1000000000L + (now.tv_nsec - start.tv_nsec);
#if defined(DEBUG)
            printf("Time in spinup: %f s\n", ns_elapsed / 1e9);
#endif
            if (ns_elapsed > SPINUP_TIME_NS)
            {
                feed_enable(true);
                state = FIRING;
#if defined(DEBUG)
                printf("Entering firing state.\n");
#endif
            }
            break;
        }

        case FIRING:
        {
            if (fire_command == false)
            {
                feed_enable(false);
                spinup_enable(false);
                state = NOT_FIRING;
#if defined(DEBUG)
                printf("Entering idle state.\n");
#endif
            }
            break;
        }
        default:
        {
#if defined(DEBUG)
            printf("Hit default case somehow\n");
#endif
            break;
        }

    }
}

void timespec_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec++;
    }
}

int main() {
    // Open the GPIO chip (usually the default chip is "/dev/gpiochip0")
    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        perror("Failed to open GPIO chip");
        return 1;
    }

    // configure controller
    int js_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (js_fd < 0) {
        perror("Could not open joystick");
        exit(1);
    }

    // Set up GPIO and PWM
    setup_gpio(chip);
    setup_pwm();

    // so we clean up on exit
    register_signal_handlers();

    // Set real-time priority
    struct sched_param param;
    param.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("Failed to set real-time priority");
        return 1;
    }

    // get initial clock time
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    // cyclic command loop
    gamepad_data cmd;
    while (1) {
        // get speed and do motor control
        get_gamepad_commands(js_fd, &cmd);

        drive_enable(cmd.drive_enable);
        control_motor((int)(cmd.joystick_x_axis_normal * MAX_RPM));
        control_firing(cmd.firing_command);

        // Schedule next activation time
        timespec_add_ns(&next, PERIOD_NS);

        // Sleep until the next period (absolute time)
        int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (ret != 0) {
            if (ret == EINTR) continue; // Interrupted by signal
            perror("clock_nanosleep");
        }
    }

    return 0;
}

