/* 
 * File:   Communication.cpp
 * Author: Jan Dufek
 */

#include "Communication.hpp"

Communication::Communication(const char * ip_address, const short port) {

    // Create socket descriptor. We want to use datagram UDP.
    socket_descriptor = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_descriptor < 0) {
        cout << "Error creating socket descriptor." << endl;
    }

    // Create socket address
    socket_address.sin_family = AF_INET;
    socket_address.sin_addr.s_addr = inet_addr(ip_address);
    socket_address.sin_port = htons(port);

}

Communication::Communication(const Communication& orig) {
}

Communication::~Communication() {
    
    stop_robot();
    close_communication();
    
}

void Communication::send_command(Command& command) {

    // Pack datagram
    double package[2];
    package[0] = command.get_throttle();
    package[1] = command.get_rudder();

    // Send throttle
    sendto(socket_descriptor, &package, 2 * sizeof (double), 0, (struct sockaddr *) &socket_address, sizeof (socket_address));

}

void Communication::stop_robot() {
    
    Command * stop_command = new Command(0, 0);
    
    send_command(* stop_command);
    
}

void Communication::close_communication() {
    
    // Close socket
    close(socket_descriptor);
    
}