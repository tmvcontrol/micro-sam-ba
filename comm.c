/*
 * Copyright (c) 2015-2016, Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "comm.h"
#include "utils.h"

static bool switch_to_binary(int fd);

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#include <Windows.h>

HANDLE hSerial;
char lastError[1024];

char deviceFullName[1024];

char * get_error()
{
	FormatMessage(
		FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		GetLastError(),
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		lastError,
		1024,
		NULL);
	return lastError;
}

int samba_open(const char* device)
{
	strcpy(deviceFullName, "\\\\.\\");
	strncat(deviceFullName, device, sizeof(deviceFullName));

	hSerial = CreateFile(deviceFullName,
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if (hSerial == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			//serial port does not exist. Inform user.
			fprintf(stderr, "Port %s does not exist", device);
			return -1;
		}
		//some other error occurred. Inform user.
		perror("Could not open device");
		return -1;
	}

	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams)) {
		//error getting state
	}
	dcbSerialParams.BaudRate = CBR_115200;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams)) {
		//error setting serial port state
		perror("Could not set serial port state");
		return -1;
	}

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if (!SetCommTimeouts(hSerial, &timeouts)) {
		//error occureed. Inform user
		perror("Could not set serial port timeouts");
		return -1;
	}

	if (!switch_to_binary(1)) {
		//error occureed. Inform user
		perror("Could not switch to binary mode");
		CloseHandle(hSerial);
		return -1;
	}

	return 1;
}

void samba_close(int fd)
{
	if(fd > 0) CloseHandle(hSerial);
}

inline int SerialRead(int fd, void * buffer, unsigned int count)
{
	DWORD dwBytesRead = 0;
	
	if (fd <= 0) {
		perror("Serial port not open!");
		return -1;
	}
	
	if (!ReadFile(hSerial, buffer, count, &dwBytesRead, NULL)) {
		//error occurred. Report to user.
		perror("Read Error occured");
		return -1;
	}

	return (int)dwBytesRead;
}

inline int SerialWrite(int fd, void const* buffer, unsigned int count)
{
	DWORD dwBytesWritten = 0;

	if (fd <= 0) {
		perror("Serial port not open!");
		return -1;
	}

	if (!WriteFile(hSerial, buffer, count, &dwBytesWritten, NULL)) {
		//error occurred. Report to user.
		perror("Write Error occured");
		return -1;
	}

	return (int)dwBytesWritten;
}

#else
#include <termios.h>

static bool configure_tty(int fd, int speed)
{
	struct termios tty;

	memset(&tty, 0, sizeof(tty));

	if (tcgetattr(fd, &tty) != 0) {
		perror("error from tcgetattr: ");
		return false;
	}

	if (speed) {
		cfsetospeed(&tty, speed);
		cfsetispeed(&tty, speed);
	}

	tty.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
	tty.c_cflag |= CS8 | CLOCAL | CREAD;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 5;
	tty.c_iflag &= ~(ICRNL | IGNBRK | IXON | IXOFF | IXANY);

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		perror("error from tcsetattr: ");
		return false;
	}

	return true;
}

/*static bool switch_to_binary(int fd)
{
	char cmd[] = "N#";
	if (write(fd, cmd, strlen(cmd)) != strlen(cmd))
		return false;
	return read(fd, cmd, 2) == 2;
}*/

int samba_open(const char* device)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		perror("Could not open device");
		return -1;
	}

	if (!configure_tty(fd, 0xB115200)) {
		close(fd);
		return -1;
	}

	if (!switch_to_binary(fd)) {
		close(fd);
		return -1;
	}

	return fd;
}

void samba_close(int fd)
{
	close(fd);
}

#define SerialRead(fd, buffer, count) read(fd, buffer, count)
#define SerialWrite(fd, buffer, count) write(fd, buffer, count)

#endif

static bool switch_to_binary(int fd)
{
	char cmd[] = "N#";
	if (SerialWrite(fd, cmd, strlen(cmd)) != strlen(cmd))
		return false;
	return SerialRead(fd, cmd, 2) == 2;
}

bool samba_read_word(int fd, uint32_t addr, uint32_t* value)
{
	char cmd[12];
	snprintf(cmd, sizeof(cmd), "w%08x,#", addr);
	if (SerialWrite(fd, cmd, strlen(cmd)) != strlen(cmd))
		return false;
	return SerialRead(fd, value, 4) == 4;
}

bool samba_write_word(int fd, uint32_t addr, uint32_t value)
{
	char cmd[20];
	snprintf(cmd, sizeof(cmd), "W%08x,%08x#", addr, value);
	return SerialWrite(fd, cmd, strlen(cmd)) == strlen(cmd);
}

bool samba_read(int fd, uint8_t* buffer, uint32_t addr, uint32_t size)
{
	char cmd[20];
	while (size > 0) {
		uint32_t count = MIN(size, 1024);
		// workaround for bug when size is exactly 512
		if (count == 512)
			count = 1;
		// workaround for other bug when size is exactly 1024
		if (count == 1024)
			count = 1024 - 1;
		snprintf(cmd, sizeof(cmd), "R%08x,%08x#", addr, count);
		printf("%s\n", cmd);
		if (SerialWrite(fd, cmd, strlen(cmd)) != strlen(cmd))
			return false;
		if (SerialRead(fd, buffer, count) != count)
			return false;
		addr += count;
		buffer += count;
		size -= count;
	}
	return true;
}

bool samba_write(int fd, uint8_t* buffer, uint32_t addr, uint32_t size)
{
	char cmd[20];
	while (size > 0) {
		uint32_t count = MIN(size, 1024);
		// workaround for bug when size is exactly 512
		if (count == 512)
			count = 1;
		snprintf(cmd, sizeof(cmd), "S%08x,%08x#", addr, count);
		if (SerialWrite(fd, cmd, strlen(cmd)) != strlen(cmd))
			return false;
		if (SerialWrite(fd, buffer, count) != count)
			return false;
		buffer += count;
		size -= count;
	}
	return true;
}
