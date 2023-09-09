
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/ioctl.h>


#define DAQ_FNAME   "/dev/amc7812"
#define BUF_SIZE    512
#define DELIM_CHAR  0xA5
#define DELIM_COUNT 4

int main(int argc, char** argv){

    int daq_fd = open(DAQ_FNAME, 'r'); // open character device for reading
    if (daq_fd < 0){
        printf("Driver likely not loaded. Exiting...");
        return -1;
    }

    bool found_delim = false;
    uint8_t buffer[BUF_SIZE];
    int index = 0;
    int delim_count = 0;

    while(true){
        if(!found_delim){
            // read one byte at a time until we've gotten our delimiter
            read(daq_fd, buffer + delim_count, 1);

            if (buffer[delim_count] == DELIM_CHAR){
                delim_count++;
            }else{
                delim_count = 0;
            }
            found_delim = (delim_count == DELIM_COUNT);
        }else{

        }
    }

    return 0;
}