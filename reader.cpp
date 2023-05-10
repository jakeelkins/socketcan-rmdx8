#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

const double pi = 3.14159265358979323846;

class ServoReader {
    public:

        int s;
        int stat;
        struct sockaddr_can addr;
        struct ifreq ifr;

        struct can_frame frame;
        int nbytes;
        socklen_t len;

        int servo_id = 0x141;
        int can_dlc = 8;
        int gear_ratio = 6;

    int connect(){
        // connect and bind to the servo

        s = -1;
        stat = EXIT_SUCCESS;
        len = sizeof(addr);

        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (s < 0){
            perror("Error while creating socket");
            stat = EXIT_FAILURE;
        }

        // bind
    	/* get interface index */
        strcpy(ifr.ifr_name, "slcan0");
        ioctl(s, SIOCGIFINDEX, &ifr);

        // NOTE: this has to come after the strcpy and ioctl.
        // this gave me fits.
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        /* bind CAN socket to the given interface */
        int bind_stat = bind(s, (struct sockaddr *)&addr, sizeof(addr));

        if (bind_stat < 0){
            perror("Error in socket bind");
            stat = EXIT_FAILURE;
        }

        return stat;
    }

    void disconnect(){
        // disconnect when something happens
        if (s >= 0){
            close(s);
            printf("Socket disconnected. \n");
        }
    }

    // ----------- commands -----------

    int read_angle(float* curr_angle){

        // puts the current angle (in radians) into variable curr_angle

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // read multi_turn_angle cmnd
        frame.data[0] = 0x92;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in read angle send");
            stat = EXIT_FAILURE;
        }

        for(;;){
            // need a while loop to ensure we are filtering the
            // right angle command reply (not the torque command replies)

            nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                    0, (struct sockaddr*)&addr, &len);

            if (nbytes <= 0){
                perror("Read zero bytes in read angle");
                stat = EXIT_FAILURE;
                break;
            }

            if (frame.data[0] == 0x92){
                // got our command reply, report
                break;
            };
        }

        // now need to convert angle from int64 little-endian, 7 bytes, to float.
        // also comes out in degrees.
        unsigned char angle_bytes[7]{frame.data[1],
                                    frame.data[2],
                                    frame.data[3],
                                    frame.data[4],
                                    frame.data[5],
                                    frame.data[6],
                                    frame.data[7]};

        int angle_int;
        std::memcpy(&angle_int, angle_bytes, sizeof(int));

        // angle_int now holds angle in degrees to 0.01 LSB:
        *curr_angle = angle_int*0.01f*pi/180.0f/gear_ratio;

        return stat;
    }


    int get_motor_status(float* curr_velo){

        // this command gets lots of motor info (temp, voltage, speed, encoder pos)
        // we just use it to get motor speed (velocity to be precise)

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // read motor status 2 cmnd
        frame.data[0] = 0x9C;
        frame.data[1] = 0x00;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in get motor status send");
            stat = EXIT_FAILURE;
        }

        for(;;){
            // need a while loop to ensure we are filtering the
            // right command reply (not the torque command replies)

            nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                    0, (struct sockaddr*)&addr, &len);

            if (nbytes <= 0){
                perror("Read zero bytes in get motor status");
                stat = EXIT_FAILURE;
                break;
            }

            if (frame.data[0] == 0x9C){
                // got our command reply, report
                break;
            };
        }

        // now need to convert speed from int16 little-endian, 2 bytes, to float.
        // also comes out in degrees.
        unsigned char velo_bytes[2]{frame.data[4],
                                    frame.data[5]};

        // DEBUG:
        // printf(" \n Received bytes:  ");
        // std::cout << std::hex << frame.data[4];
        // std::cout << std::hex << frame.data[5] << '\n';
        //printf("/n Received: %i \t %i \n", frame.data[4], frame.data[5]);

        short velo_short;
        std::memcpy(&velo_short, velo_bytes, sizeof(short));

        // velo_int now holds speed in degrees/s to 1 LSB:
        *curr_velo = (float)velo_short*pi/180.0f/gear_ratio;  // rad/s

        return stat;
    }

};


int main (int argc, char *argv[]){

    ServoReader rdr;
    int stat;

    // state data to read
    float curr_angle;
    float curr_velo;

    // init the servo class
    rdr = ServoReader();

    // connect the socket
    stat = rdr.connect();

    if (stat){
        rdr.disconnect();
        return 1;
    } else {
        printf("Socket connected. \n");
    }

    printf("Reading angle data... \n");

    // read angle data
    for (unsigned int i = 0; i<100; ++i){
        stat = rdr.read_angle(&curr_angle);
        printf("Current angle: %f rad \t", curr_angle);

        stat = rdr.get_motor_status(&curr_velo);
        printf("Current velo: %f rad/s \n", curr_velo);

        // 10 Hz:
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    printf("Success. Disconnecting... \n");
    rdr.disconnect();

    return 0;
}