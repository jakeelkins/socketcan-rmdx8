#include <cstdio>

int main(int argc, char* argv[]){

    float desired_angle = 30.25;
    int desired_angle_int = desired_angle*100;

    unsigned char const * p = reinterpret_cast<unsigned char const *>(&desired_angle_int);

    for (size_t i = 0; i < sizeof(float); ++i)
    {
        printf("The byte #%zu is 0x%02X\n", i, p[i]);
    }

    return 0;
}