all: 
	gcc -Wall -Werror -O2 joystick_test.c -lgpiod

debug: 
	gcc -Wall -Werror -g joystick_test.c -lgpiod -DDEBUG
