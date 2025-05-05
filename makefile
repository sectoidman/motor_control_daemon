# vim: filetype=make

BINARY = motor_control
SOURCES = motor_control.c

all:
	gcc -Wall -Werror -O2 ${SOURCES} -lgpiod -o ${BINARY}

debug:
	gcc -Wall -Werror -g ${SOURCES} -lgpiod -DDEBUG -o ${BINARY}

clean:
	rm ${BINARY}

install:
	cp ${BINARY} /usr/local/bin
	setcap 'cap_sys_nice=eip' /usr/local/bin/${BINARY}
