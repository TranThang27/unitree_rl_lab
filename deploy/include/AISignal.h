// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <atomic>
#include <thread>
#include <zmq.hpp>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>

// Module selection enum
enum class AIModule {
    RAISING_HAND_AI = 1   // RaisingHand controlled by AI signal
};

class AISignal
{
public:
    static AISignal& getInstance()
    {
        static AISignal instance;
        return instance;
    }

    // Module selection
    void setModule(AIModule module) { selected_module_ = module; }
    AIModule getModule() const { return selected_module_.load(); }

    void start(const std::string& address = "tcp://localhost:5555")
    {
        if (running_.load()) return;
        
        running_ = true;
        receiver_thread_ = std::thread([this, address]() {
            try {
                zmq::context_t context(1);
                zmq::socket_t subscriber(context, ZMQ_SUB);
                
                // Set timeout ngắn để phản ứng nhanh với lệnh stop()
                subscriber.set(zmq::sockopt::rcvtimeo, 100);
                subscriber.connect(address);

                subscriber.set(zmq::sockopt::subscribe, "vel");
                subscriber.set(zmq::sockopt::subscribe, "raisinghand");
                
                spdlog::info("AISignal: Connected to {}", address);
                
                while (running_.load()) {
                    zmq::message_t msg;
                    auto res = subscriber.recv(msg, zmq::recv_flags::none);
                    
                    // KIỂM TRA DỪNG LUỒNG: Nếu stop() đã được gọi trong lúc đợi, thoát ngay
                    if (!running_.load()) break;

                    if (res.has_value()) {
                        std::string received = std::string(static_cast<char*>(msg.data()), msg.size());
                        
                        // Xử lý tín hiệu kích hoạt giơ tay
                        if (received.find("raisinghand") != std::string::npos) {
                            raising_hand_triggered_ = true;
                            // Reset vận tốc nội bộ về 0 ngay khi nhận lệnh
                            vel_x_ = 0.0f; vel_y_ = 0.0f; vel_ang_ = 0.0f;
                            spdlog::info("AISignal: RAISINGHAND triggered! Port logic suspended.");
                        }
                        // Xử lý lệnh vận tốc (chỉ khi không bị override)
                        else if (received.find("vel ") == 0) {
                            if (override_velocity_.load()) {
                                continue;
                            }
                            
                            size_t space_pos = received.find(' ');
                            if (space_pos != std::string::npos) {
                                std::string data = received.substr(space_pos + 1);
                                std::vector<std::string> parts;
                                size_t pos = 0;
                                while ((pos = data.find('|')) != std::string::npos) {
                                    parts.push_back(data.substr(0, pos));
                                    data.erase(0, pos + 1);
                                }
                                parts.push_back(data);
                                
                                if (parts.size() >= 3) {
                                    float vx = std::stof(parts[0]);
                                    float vy = std::stof(parts[1]);
                                    float vang = std::stof(parts[2]);
                                    
                                    vel_x_ = vx;
                                    vel_y_ = vy;
                                    vel_ang_ = vang;
                                    
                                    // Chỉ log khi thread vẫn đang được phép chạy
                                    if (running_.load() && (std::abs(vx - last_vx_) > 0.05f || 
                                        std::abs(vy - last_vy_) > 0.05f || 
                                        std::abs(vang - last_vang_) > 0.02f)) {
                                        spdlog::info("AISignal: Vel ({:.2f}, {:.2f}, {:.2f})", vx, vy, vang);
                                        last_vx_ = vx; last_vy_ = vy; last_vang_ = vang;
                                    }
                                }
                            }
                        }
                    }
                }
                subscriber.close();
            } catch (const zmq::error_t& e) {
                if (running_.load()) spdlog::error("AISignal ZMQ error: {}", e.what());
            }
        });
        
        spdlog::info("AISignal: Receiver thread started");
    }

    void stop()
    {
        // 1. Hạ cờ chạy để thread ZMQ thoát vòng lặp ngay lập tức
        running_ = false;

        // 2. CƯỠNG ÉP VẬN TỐC VỀ 0
        // Việc này đảm bảo robot đứng yên dù thread ZMQ có đang kẹt hay không
        vel_x_ = 0.0f;
        vel_y_ = 0.0f;
        vel_ang_ = 0.0f;
        
        // 3. Đợi thread kết thúc an toàn
        if (receiver_thread_.joinable()) {
            receiver_thread_.join();
        }
        spdlog::warn("AISignal: Port 5555 CLOSED. All velocities reset to 0.0.");
    }

    // Getters
    float getVelX() const { return vel_x_.load(); }
    float getVelY() const { return vel_y_.load(); }
    float getVelAng() const { return vel_ang_.load(); }
    
    // Setters (FSM override)
    void setVelocity(float vx, float vy, float vang) {
        vel_x_ = vx;
        vel_y_ = vy;
        vel_ang_ = vang;
        override_velocity_ = true;
    }
    
    void clearVelocityOverride() {
        override_velocity_ = false;
    }
    
    bool isVelocityOverride() const { return override_velocity_.load(); }
    bool isRaisingHandTriggered() const { return raising_hand_triggered_.load(); }
    void clearRaisingHandTrigger() { raising_hand_triggered_ = false; }

private:
    AISignal() : running_(false), vel_x_(0.0f), vel_y_(0.0f), vel_ang_(0.0f),
                 raising_hand_triggered_(false), selected_module_(AIModule::RAISING_HAND_AI),
                 last_vx_(0.0f), last_vy_(0.0f), last_vang_(0.0f), override_velocity_(false) {}
    ~AISignal() { stop(); }
    
    AISignal(const AISignal&) = delete;
    AISignal& operator=(const AISignal&) = delete;

    std::atomic<bool> running_;
    std::atomic<float> vel_x_;
    std::atomic<float> vel_y_;
    std::atomic<float> vel_ang_;
    std::atomic<bool> raising_hand_triggered_;
    std::atomic<AIModule> selected_module_;
    std::atomic<bool> override_velocity_;
    float last_vx_;
    float last_vy_;
    float last_vang_;
    std::thread receiver_thread_;
};