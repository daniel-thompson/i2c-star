/*
 * i2cwrite.c
 *
 * Perform multi-byte write sequences using I2C.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>

int main(int argc, const char *argv[])
{
	int fd;

	if (argc != 4) {
		fprintf(stderr, "Usage: i2cwrite I2CBUS CHIP-ADDRESS DATA\n");
		return 1;
	}

	char fname[20];
	snprintf(fname, sizeof(fname), "/dev/i2c-%s", argv[1]);
	fname[sizeof(fname)-1] = '\0';

	fd = open(fname, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Cannot open I2C device (%s)\n",
			strerror(errno));
		return 2;
	}
	
	int addr = strtol(argv[2], NULL, 0);
	if (ioctl(fd, I2C_SLAVE, addr) < 0) {
		fprintf(stderr, "Cannot set I2C slave address to 0x%x (%s)\n",
			addr, strerror(errno));
		return 3;
	}

	unsigned int buflen = strlen(argv[3]) / 2;
	unsigned char *buf = malloc(buflen);
	if (NULL == buf) {
		fprintf(stderr, "Cannot allocate scratch memory\n");
		return 4;
	}

	char hex[3] = "00";
	for (int i=0; i<buflen; i++) {
		hex[0] = argv[3][i*2];
		hex[1] = argv[3][i*2+1];
		buf[i] = strtol(hex, NULL, 16);
	}

	if (write(fd, buf, buflen) != buflen) {
		fprintf(stderr, "Cannot write I2C data (%s)\n",
			strerror(errno));
		return 5;
	}
	
	return 0;
}
