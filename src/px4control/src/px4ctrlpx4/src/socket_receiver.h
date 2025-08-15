#ifndef SOCKET_RECEIVER_H
#define SOCKET_RECEIVER_H

#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <netinet/in.h>

struct Waypoint {
    double latitude;
    double longitude;
    double altitude;
    double yaw;
};

class SocketReceiver {
public:
    SocketReceiver(int port, int buffer_size = 4096);
    ~SocketReceiver();
    
    void start();
    void stop();
    std::vector<Waypoint> getReceivedWaypoints() const;
    std::vector<Waypoint> popProcessedWaypoints(size_t count);
    void clearWaypoints();
    
    
private:
    void run();
    bool setupSocket();
    void handleConnection(int client_socket);
    bool parseWaypoints(const std::string& data, std::vector<Waypoint>& waypoints); // 修改为解析字符串

    int port_;
    int buffer_size_;
    int server_fd_ = -1;
    bool running_ = false;
    std::thread thread_;
    mutable std::mutex mutex_;
    std::vector<Waypoint> waypoints_;
    struct sockaddr_in address_;
};

#endif