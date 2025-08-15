#include "socket_receiver.h"
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <stdexcept>
#include <sstream>


SocketReceiver::SocketReceiver(int port, int buffer_size) 
    : port_(port), buffer_size_(buffer_size) {
    address_.sin_family = AF_INET;
    address_.sin_addr.s_addr = INADDR_ANY;
    address_.sin_port = htons(port_);
}

SocketReceiver::~SocketReceiver() {
    stop();
}

void SocketReceiver::start() {
    if (running_) return;

    if (!setupSocket()) {
        throw std::runtime_error("Failed to setup socket");
    }

    running_ = true;
    thread_ = std::thread(&SocketReceiver::run, this);
}

void SocketReceiver::stop() {
    if (!running_) return;

    running_ = false;
    if (server_fd_ != -1) {
        shutdown(server_fd_, SHUT_RDWR);
        close(server_fd_);
        server_fd_ = -1;
    }

    if (thread_.joinable()) {
        thread_.join();
    }
    // ROS_INFO("Socket receiver stopped gracefully");
    std::cout << "Socket receiver stopped gracefully " << port_ << std::endl;
}

bool SocketReceiver::setupSocket() {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        std::cerr << "Socket creation error: " << strerror(errno) << std::endl;
        return false;
    }

    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        std::cerr << "Setsockopt error: " << strerror(errno) << std::endl;
        return false;
    }

    if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0) {
        std::cerr << "Bind failed: " << strerror(errno) << std::endl;
        return false;
    }

    if (listen(server_fd_, 3) < 0) {
        std::cerr << "Listen failed: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

void SocketReceiver::run() {
    std::cout << "Server started on port " << port_ << std::endl;

    while (running_) {
        sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        
        int client_socket = accept(server_fd_, (struct sockaddr*)&client_addr, &addr_len);
        if (client_socket < 0) {
            if (running_) {
                std::cerr << "Accept error: " << strerror(errno) << std::endl;
            }
            continue;
        }

        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
        std::cout << "Connection accepted from " << client_ip << std::endl;

        handleConnection(client_socket);
        close(client_socket);
    }
}

void SocketReceiver::handleConnection(int client_socket) {
    char buffer[buffer_size_];
    ssize_t bytes_read;
    std::string received_data;

    while ((bytes_read = read(client_socket, buffer, buffer_size_ - 1)) > 0) {
        buffer[bytes_read] = '\0';
        received_data += buffer;

        // 检查是否收到完整数据（以分号结尾）
        size_t pos;
        while ((pos = received_data.find(';')) != std::string::npos) {
            std::string segment = received_data.substr(0, pos);
            received_data.erase(0, pos + 1);

            std::vector<Waypoint> new_waypoints;
            if (parseWaypoints(segment, new_waypoints)) {
                std::lock_guard<std::mutex> lock(mutex_);
                waypoints_.insert(waypoints_.end(), new_waypoints.begin(), new_waypoints.end());
                
                // 发送确认
                std::string ack = "ACK\n";
                send(client_socket, ack.c_str(), ack.length(), 0);
            } else {
                std::string nack = "INVALID_FORMAT\n";
                send(client_socket, nack.c_str(), nack.length(), 0);
            }
        }
    }

    if (bytes_read < 0) {
        std::cerr << "Read error: " << strerror(errno) << std::endl;
    }
}


bool SocketReceiver::parseWaypoints(const std::string& data, std::vector<Waypoint>& waypoints) {
    std::istringstream stream(data);
    std::string field;
    
    try {
        Waypoint wp;
        char delimiter = ',';
        
        // 格式：latitude,longitude,altitude
        std::getline(stream, field, delimiter);
        wp.latitude = std::stod(field);
        
        std::getline(stream, field, delimiter);
        wp.longitude = std::stod(field);
        
        std::getline(stream, field, delimiter);
        wp.altitude = std::stod(field);

        std::getline(stream, field, delimiter);
        wp.yaw = std::stod(field);
        
        waypoints.push_back(wp);
        return true;
    } catch (...) {
        std::cerr << "Failed to parse waypoint: " << data << std::endl;
        return false;
    }
}

// 获取数据（保持const限定）
std::vector<Waypoint> SocketReceiver::getReceivedWaypoints() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return waypoints_; 
}

// 单独清空方法
void SocketReceiver::clearWaypoints() {
    std::lock_guard<std::mutex> lock(mutex_);
    waypoints_.clear();
}

std::vector<Waypoint> SocketReceiver::popProcessedWaypoints(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 获取要处理的数量（不超过实际大小）
    count = std::min(count, waypoints_.size());
    
    // 复制要返回的航点
    std::vector<Waypoint> result(
        waypoints_.begin(),
        waypoints_.begin() + count
    );
    
    // 移除已处理的航点
    waypoints_.erase(waypoints_.begin(), waypoints_.begin() + count);
    
    return result;
}