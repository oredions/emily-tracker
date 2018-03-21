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

/**
 * Log string message into general log.
 * 
 * @param message
 */
void Logger::log_general(string message) {
    general_log_file << message;
}

/**
 * Log double value into general log.
 * 
 * @param message
 */
void Logger::log_general(double message) {
    general_log_file << message;
}

/**
 * Log rudder string value into rudder log.
 * 
 * @param message
 */
void Logger::log_rudder(string message) {
    rudder_log_file << message;
}

/**
 * Log rudder double value into rudder log.
 * 
 * @param message
 */
void Logger::log_rudder(double message) {
    rudder_log_file << message;
}

/**
 * Log throttle string value into throttle log.
 * 
 * @param message
 */
void Logger::log_throttle(string message) {
    throttle_log_file << message;
}

/**
 * Log throttle double value into throttle log.
 * 
 * @param message
 */
void Logger::log_throttle(double message) {
    throttle_log_file << message;
}

/**
 * Close log files.
 * 
 */
void Logger::close() {
    general_log_file.close();
    rudder_log_file.close();
    throttle_log_file.close();
}