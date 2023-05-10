#include <iostream>
#include <cstdio>
#include <cstring>

int main() {
    // should be 3025
    //unsigned char bytes[7]{ 0xD1, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00};
    // should be -3025
    //unsigned char bytes[7]{ 0x2F, 0xF4, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    // should be -10
    unsigned char bytes[2]{0xF6, 0xFF};
    //unsigned char bytes[2]{0xFF, 0xF6};

    //int value;
    short value;
    //float value;
    //std::memcpy(&value, bytes, sizeof(int));
    std::memcpy(&value, bytes, sizeof(short));

    //std::cout << std::hex << value << '\n';
    printf("short read: %i \n", value);
    printf("short to float: %f \n", (float)value);
}