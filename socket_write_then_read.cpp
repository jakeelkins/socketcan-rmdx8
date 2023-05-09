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

void dump_can_frame(struct can_frame *frame)
{
	int i;

	if (frame->can_id & CAN_EFF_FLAG)
		printf("%08X   ", frame->can_id);
	else
		printf("%03X   ", frame->can_id);

	printf("[%d]   ", frame->can_dlc);

	for (i = 0; i < 8; i++) {
		printf("%02X ", frame->data[i]);
	}

	printf("\n");
}

void read_pid_msg(struct can_frame *frame)
{
	frame->can_id  = 0x141;
	frame->can_dlc = 8;

    // read PID cmnd
	frame->data[0] = 0x30;
	frame->data[1] = 0x00;
	frame->data[2] = 0x00;
	frame->data[3] = 0x00;
	frame->data[4] = 0x00;
	frame->data[5] = 0x00;
	frame->data[6] = 0x00;
	frame->data[7] = 0x00;
}

void write_pid_msg(struct can_frame *frame)
{
    // NOTE: this differes from the old one in Python.
	frame->can_id  = 0x141;
	frame->can_dlc = 8;

    // write PID cmnd
	frame->data[0] = 0x31;
	frame->data[1] = 0x00;
	frame->data[2] = 0x01; // current/torque Kp
	frame->data[3] = 0x01; // current/torque Ki
	frame->data[4] = 0x00; // speed Kp
	frame->data[5] = 0x00; // speed Ki
	frame->data[6] = 0x00; // pos Kp
	frame->data[7] = 0x00; // pos Ki
}

void read_angle_msg(struct can_frame *frame)
{
	frame->can_id  = 0x141;
	frame->can_dlc = 8;

    // read angle
	frame->data[0] = 0x92;
	frame->data[1] = 0x00;
	frame->data[2] = 0x00; 
	frame->data[3] = 0x00; 
	frame->data[4] = 0x00;
	frame->data[5] = 0x00;
	frame->data[6] = 0x00;
	frame->data[7] = 0x00;
}

int main (int argc, char *argv[])
{
	int s = -1;
	int nbytes;
	struct sockaddr_can addr;
    socklen_t len = sizeof(addr);
	struct can_frame frame;
	struct ifreq ifr;
	int ret = EXIT_FAILURE;

	/* create CAN socket */
	if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while creating socket");
		goto error;
	}

	/* get interface index */
	strcpy(ifr.ifr_name, "slcan0");
    //strcpy(ifr.ifr_name, "/dev/ttyUSB0");
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/* bind CAN socket to the given interface */
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		goto error;
	}

	printf("s: %i \n", s);

    // get write PID cmnd
    write_pid_msg(&frame);

    nbytes = sendto(s, &frame, sizeof(struct can_frame),
            0, (struct sockaddr*)&addr, sizeof(addr));

    // get PID cmnd
    read_pid_msg(&frame);

	/* send CAN frame */
	//nbytes = write(s, &frame, sizeof(struct can_frame));
    nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

	// printf("Wrote %d bytes\n", nbytes);

	// for (int i = 0; i < 8; i++) {
	// 	printf("%02X ", frame.data[i]);
	// }

    printf("\n");
    printf("now reading reply ... \n");

    /* read CAN frame */
    nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, &len);
    dump_can_frame(&frame);

	ret = EXIT_SUCCESS;

error:
	/* close socket */
	if (s >= 0)
		close(s);

	return ret;
}