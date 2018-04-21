
#include <iostream>
#include "standard/mavlink.h"

#include <cstdlib>
#include <stdio.h>   // Standard input/output definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>

using namespace std;

int main(int argc, const char * argv[]) {
    cout << "1";
    int fd = open("The/Serial/Port/With/Mavlink", O_RDWR | O_NOCTTY | O_NDELAY);
    
    // Check for Errors
    if (fd == -1)
    {
        /* Could not open the port. */
        return(-1);
    }
    
    // Finalize
    else
    {
        fcntl(fd, F_SETFL, 0);
    }
    
    struct termios  config;
    tcgetattr(fd, &config);
    
    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);
    
    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                        ONOCR | OFILL | OPOST);

    
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;
    
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0

    
    cfsetispeed(&config, B57600);
    
    cfsetospeed(&config, B57600);
    
    tcsetattr(fd, TCSAFLUSH, &config);
    
    
    
    
    int c = 0;
    while(1){
        read(fd, &c, 1);
        mavlink_message_t msg;
        mavlink_status_t stat;
        if (mavlink_parse_char(0, c, &msg, &stat))
        {
            if(msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT){
                mavlink_global_position_int_t gps;
                mavlink_msg_global_position_int_decode(&msg, &gps);
                double lon = gps.lon / 10000000.0 , lat = gps.lat / 10000000.0;
                printf("lat:%lg long:%lg alt:%d\n\n", lon , lat, gps.alt);
            }
        }
        
    }
}
