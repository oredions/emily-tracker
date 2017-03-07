/* 
 * File:   Communication.hpp
 * Author: Jan Dufek
 */

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <netdb.h>
#include <arpa/inet.h>
#include "Command.hpp"

using namespace std;

class Communication {
public:
    Communication(const char *, const short);
    Communication(const Communication& orig);
    virtual ~Communication();
    
    void send_command(Command&);
    
    void stop_robot();
    
    void close_communication();
    
private:
    
    int socket_descriptor;
    
    struct sockaddr_in socket_address;

};

#endif /* COMMUNICATION_HPP */

