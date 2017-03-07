/* 
 * File:   Logger.hpp
 * Author: Jan Dufek
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <fstream>

using namespace std;

class Logger {
public:
    Logger();
    Logger(string);
    Logger(const Logger& orig);
    virtual ~Logger();
    
    void log_general(string);
    
    void log_general(double);
    
    void log_rudder(string);
    
    void log_rudder(double);
    
    void log_throttle(string);
    
    void log_throttle(double);
    
    void close();
    
private:
    
    // General log file
    ofstream general_log_file;
    
    // Rudder log file
    ofstream rudder_log_file;
    
    // Control log file
    ofstream throttle_log_file;

};

#endif /* LOGGER_HPP */

