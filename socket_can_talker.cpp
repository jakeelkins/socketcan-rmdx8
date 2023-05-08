#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

void print_usage()
{
	printf("Usage: vscansend <CAN-Channel>\n"
	       "Example: vscansend slcan0\n");
}

int main (int argc, char *argv[])
{
	int s = -1;
	int nbytes;
	struct sockaddr_can addr;
	struct can_frame frame;
	struct ifreq ifr;
	char *ifname;
	int ret = EXIT_FAILURE;

	if (argc != 2) {
		print_usage();
		goto error;
	}

	ifname = argv[1];

	/* create CAN socket */
	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while creating socket");
		goto error;
	}

	/* get interface index */
	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/* bind CAN socket to the given interface */
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		goto error;
	}

	frame.can_id  = 0x141;
	frame.can_dlc = 8;

	frame.data[0] = 0x30;
	frame.data[1] = 0x00;
	frame.data[2] = 0x00;
	frame.data[3] = 0x00;
	frame.data[4] = 0x00;
	frame.data[5] = 0x00;
	frame.data[6] = 0x00;
	frame.data[7] = 0x00;

	/* send CAN frame */
	nbytes = write(s, &frame, sizeof(struct can_frame));

	printf("Wrote %d bytes\n", nbytes);

	for (int i = 0; i < 8; i++) {
		printf("%02X ", frame.data[i]);
	}

    printf("\n");

	ret = EXIT_SUCCESS;

error:
	/* close socket */
	if (s >= 0)
		close(s);

	return ret;
}