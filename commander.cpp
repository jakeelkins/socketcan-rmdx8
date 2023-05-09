#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <cmath>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

class ServoCommander {
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
    int turn_off_motor(){

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // pid set cmnd
        frame.data[0] = 0x81;
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
            perror("Error in turn off motor send");
            stat = EXIT_FAILURE;
        }

        // now read reply
        nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, &len);

        if (nbytes == 0){
            perror("Read zero bytes in turn off motor");
            stat = EXIT_FAILURE;
        }

        // reply is same as we sent.

        return stat;
    }



    int read_pid_gains(unsigned int* pos_kp, unsigned int* pos_ki,
                        unsigned int* speed_kp, unsigned int* speed_ki,
                        unsigned int* torque_kp, unsigned int* torque_ki){

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // pid read cmnd
        frame.data[0] = 0x30;
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
            perror("Error in read PID send");
            stat = EXIT_FAILURE;
        }

        // now read reply
        nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, &len);

        if (nbytes == 0){
            perror("Read zero bytes in read PID");
            stat = EXIT_FAILURE;
        }

        // torque is current here, but i dont
        // want to say current bc it sounds like contemporary
        *pos_kp = frame.data[2]; 
        *pos_ki = frame.data[3]; 

        *speed_kp = frame.data[4]; 
        *speed_ki = frame.data[5];

        *torque_kp = frame.data[6];
        *torque_ki = frame.data[7];

        return stat;
    }


    int set_pid_gains(unsigned int pos_kp, unsigned int pos_ki,
                        unsigned int speed_kp, unsigned int speed_ki,
                        unsigned int torque_kp, unsigned int torque_ki){

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // pid set cmnd
        frame.data[0] = 0x31;
        frame.data[1] = 0x00;
        frame.data[2] = pos_kp;
        frame.data[3] = pos_ki;
        frame.data[4] = speed_kp;
        frame.data[5] = speed_ki;
        frame.data[6] = torque_kp;
        frame.data[7] = torque_ki;

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in set PID send");
            stat = EXIT_FAILURE;
        }

        // now read reply
        nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, &len);

        if (nbytes == 0){
            perror("Read zero bytes in set PID");
            stat = EXIT_FAILURE;
        }

        // reply is same as we sent.

        return stat;
    }


    int go_to_angle(float desired_angle, float desired_speed){

        // input: float of desired angle, in degrees.
        int desired_angle_int = (int)(desired_angle*100);
        desired_angle_int *= gear_ratio;
        unsigned char const * desired_angle_array = reinterpret_cast<unsigned char const *>(&desired_angle_int);

        int desired_speed_int = (int)desired_speed;
        desired_speed_int *= gear_ratio;
        unsigned char const * desired_speed_array = reinterpret_cast<unsigned char const *>(&desired_speed_int);

        frame.can_id = servo_id;
        frame.can_dlc = can_dlc;

        // angle slew cmnd
        frame.data[0] = 0xA4;
        frame.data[1] = 0x00;
        frame.data[2] = desired_speed_array[0];
        frame.data[3] = desired_speed_array[1];
        frame.data[4] = desired_angle_array[0];
        frame.data[5] = desired_angle_array[1];
        frame.data[6] = desired_angle_array[2];
        frame.data[7] = desired_angle_array[3];

        nbytes = sendto(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, sizeof(addr));

        if (nbytes <= 0){
            perror("Error in go-to-angle send");
            stat = EXIT_FAILURE;
        }

        // now read reply
        nbytes = recvfrom(s, &frame, sizeof(struct can_frame),
                0, (struct sockaddr*)&addr, &len);

        if (nbytes == 0){
            perror("Read zero bytes in go-to-angle");
            stat = EXIT_FAILURE;
        }

        // reply is a motor status command, which we dont care about here.

        return stat;
    }
};


int main (int argc, char *argv[]){

    ServoCommander cmdr;
    int stat;

    // save these for future PID experiment restarts
    unsigned int default_pos_kp;
    unsigned int default_pos_ki;
    unsigned int default_speed_kp;
    unsigned int default_speed_ki;
    unsigned int default_torque_kp;
    unsigned int default_torque_ki;

    // init the servo class
    cmdr = ServoCommander();

    // connect the socket
    stat = cmdr.connect();

    if (stat){
        cmdr.disconnect();
        return 1;
    } else {
        printf("Socket connected. \n");
    }

    printf("Reading PID params... \n");

    // read PID gains
    stat = cmdr.read_pid_gains(&default_pos_kp, &default_pos_ki,
                                &default_speed_kp, &default_speed_ki,
                                &default_torque_kp, &default_torque_ki);

    printf("PID default params: q_kp: %i   q_ki: %i   speed_kp: %i   speed_ki: %i   I_kp: %i   I_ki: %i   \n",
            default_pos_kp,default_pos_ki,default_speed_kp,default_speed_ki,default_torque_kp,default_torque_ki);

    
    // set our new PID params: zero pos and speed gains, set torque Kp to 1
    // stat = cmdr.set_pid_gains(0, 0,
    //                           0, 0,
    //                           1, 0);

    printf("Sending 1st slew command... \n");
    cmdr.go_to_angle(45.0f, 10.0f);
    sleep(5);

    printf("Sending 2nd slew command... \n");
    cmdr.go_to_angle(0.0f, 10.0f);
    sleep(5);

    printf("Success. Turning off and disconnecting... \n");
    cmdr.turn_off_motor();
    cmdr.disconnect();

    return 0;
}