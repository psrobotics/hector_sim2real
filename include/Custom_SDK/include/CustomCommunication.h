#ifndef CUSTOMCOMMUNICATION_H
#define CUSTOMCOMMUNICATION_H

#include <iostream>
#include <vector>
#include <fcntl.h> // For non-blocking socket
#include <stdexcept>
#include "Full_encode.pb.h" // Ensure this path is correct
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "pb_encode.h"
#include "pb_decode.h"

class CustomCommunication {
public:
    CustomCommunication(int port);
    ~CustomCommunication();

    std::vector<uint8_t> encodeLowLevelCmd(const LowLevelCmd& message);
    LowLevelState decodeLowLevelState(const std::vector<uint8_t>& buffer);

    void sendUdpMessage(const std::vector<uint8_t>& message, int server_port);
    std::vector<uint8_t> receiveUdpMessage();

private:
    int sockfd;
    struct sockaddr_in servaddr;
};
#endif // CUSTOMCOMMUNICATION_H