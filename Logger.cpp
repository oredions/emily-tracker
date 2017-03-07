/* 
 * File:   Logger.cpp
 * Author: Jan Dufek
 */

#include "Logger.hpp"

Logger::Logger(string name) {

    general_log_file.open(name + ".txt");

    rudder_log_file.open(name + "_rudder.txt");

    throttle_log_file.open(name + "_throttle.txt");

}

Logger::Logger(const Logger& orig) {
}

Logger::~Logger() {
    close();
}

void Logger::log_general(string message) {
    general_log_file << message;
}

void Logger::log_general(double message) {
    general_log_file << message;
}

void Logger::log_rudder(string message) {
    rudder_log_file << message;
}

void Logger::log_rudder(double message) {
    rudder_log_file << message;
}

void Logger::log_throttle(string message) {
    throttle_log_file << message;
}

void Logger::log_throttle(double message) {
    throttle_log_file << message;
}

void Logger::close() {
    general_log_file.close();
    rudder_log_file.close();
    throttle_log_file.close();
}