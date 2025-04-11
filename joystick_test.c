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

#define GPIO_DIR_PIN 13   // Direction GPIO (clockwise/counter-clockwise)
#define GPIO_PWM_PIN 19   // PWM GPIO pin
#define GPIO_ENABLE_PIN 26 // Enable GPIO pin

// Stepper motor parameters
#define STEPS_PER_REV 800  // Steps per full revolution
#define PWM_DUTY_CYCLE 50000
#define INITIAL_PWM_PERIOD 1000000000
#define MAX_RPM 1000.0

// timestep
#define PERIOD_NS 10000000L // 10 ms

// joystick
#define JOYSTICK_AXIS_MAX 32767.0
#define JOYSTICK_DEADBAND 2000.0

struct gpiod_chip *chip = NULL;

void pwm_enable(bool enable) {
	// Enable PWM signal
	char pwm_enable_path[256];
	snprintf(pwm_enable_path, sizeof(pwm_enable_path), "/sys/class/pwm/pwmchip0/pwm1/enable");
	FILE *pwm_enable_file = fopen(pwm_enable_path, "w");
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
    int motor_enable = enable ? 1 : 0;
#if defined(DEBUG)
    printf("Drive enable: %d\n", motor_enable);
#endif
    if (gpiod_line_set_value(enable_line, motor_enable) < 0)
    {
	    perror("drive enable set failed");
	    exit(1);
    }
}

void set_pwm_period(int period)
{
    char pwm_period_path[256];
    snprintf(pwm_period_path, sizeof(pwm_period_path), "/sys/class/pwm/pwmchip0/pwm1/period");
    FILE *pwm_period_file = fopen(pwm_period_path, "w");
    if (pwm_period_file == NULL) {
        perror("Failed to set PWM period");
        exit(1);
    }
    fprintf(pwm_period_file, "%d", period);
    fclose(pwm_period_file);
}

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
    char pwm_dc_path[256];
    snprintf(pwm_dc_path, sizeof(pwm_dc_path), "/sys/class/pwm/pwmchip0/pwm1/duty_cycle");
    FILE *pwm_dc_file = fopen(pwm_dc_path, "w");
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
    struct gpiod_line *dir_line, *enable_line;

    // Get lines for direction (13) and enable (26)
    dir_line = gpiod_chip_get_line(chip, GPIO_DIR_PIN);
    enable_line = gpiod_chip_get_line(chip, GPIO_ENABLE_PIN);

    if (!dir_line || !enable_line) {
        perror("Failed to get GPIO lines");
        exit(1);
    }

    // Request the lines for output
    if (gpiod_line_request_output(dir_line, "motor_control", 0) < 0) {
        perror("Failed to request direction line");
        exit(1);
    }
    if (gpiod_line_request_output(enable_line, "motor_control", 1) < 0) { // Enable motor (set to high)
        perror("Failed to request enable line");
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


int get_speed(int js_fd)
{
    struct js_event e;
    static int x_axis = 0; // static so we persist the last value
    static bool drive_state = false;

     while (read(js_fd, &e, sizeof(e)) > 0) {
         if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS && e.number == 0) {
             x_axis = e.value; // X-axis of left stick
         }
       if ((e.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON && e.number == 1 && e.value == 1)
       {
               // O button pressed
               // toggle the drive enable.
               drive_state = !drive_state;
               drive_enable(drive_state);
       }
     }

    // exponential smoothing low-pass filter
    static const double alpha = 0.2;
    static double x_axis_smoothed = 0.0;
    x_axis_smoothed = alpha * x_axis + (1.0 - alpha) * x_axis_smoothed;

    // deadband - ignore movement commands less than 2% 
    double x_axis_normal = 0.0;
    if (fabs(x_axis_smoothed) > JOYSTICK_DEADBAND)
    {
        x_axis_normal = x_axis_smoothed / JOYSTICK_AXIS_MAX;
#if defined(DEBUG)
       printf("Joystick: %.3f\n", x_axis_normal);
#endif
    }

    return (int)(x_axis_normal * MAX_RPM);
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

void timespec_add_ns(struct timespec *t, long ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) {
        t->tv_nsec -= 1000000000L;
        t->tv_sec++;
    }
}

int main() {
    // Set real-time priority
    struct sched_param param;
    param.sched_priority = 99;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("Failed to set real-time priority");
        return 1;
    }

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

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    // Set up GPIO and PWM
    setup_gpio(chip);
    setup_pwm();

    register_signal_handlers();

    // Command loop
    int speed = 0;
    while (1) {
	// get speed and do motor control
	speed = get_speed(js_fd);
        control_motor(speed);

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

