#include <iostream>
// #include <stdlib.h>


//vai pro slave
void uint16_to_char_array(uint16_t value, char *array) {
    // Convert the uint16_t value to two separate bytes
    array[0] = (char)(value & 0xFF);          // Extract the lower byte
    array[1] = (char)((value >> 8) & 0xFF);   // Extract the higher byte
}


//vai pro master
uint16_t char_array_to_uint16(const char *array) {
    // Combine the two bytes into a single uint16_t value
    uint16_t value = ((uint16_t)array[1] << 8) | (uint16_t)array[0];
    return value;
}

int main() {
    uint16_t number = 72;
    char array[2];
    uint16_to_char_array(number, array);
    uint16_t value = char_array_to_uint16(array);
    std::cout<<"O char eh: ";
    std::cout << array[0];
    std::cout << array[1]<<std::endl;
    std::cout<<"O numero eh: ";
    std::cout << value;


    return 0;
}
