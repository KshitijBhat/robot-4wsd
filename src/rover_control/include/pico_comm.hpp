#ifndef PICO_COMM_HPP
#define PICO_COMM_HPP

#include <libserial/SerialPort.h>
#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <chrono>

#define PI 3.1416

// ENCODER_TICKS_PER_REV 660 for 100 rpm motor
// ENCODER_TICKS_PER_REV 660*100/30 for 30 rpm motor
// ENCODER_TICKS_PER_REV 660*100/10 for 10 rpm motor

#define STEERING_ENCODER_TICKS_PER_REV 6600
#define DRIVING_ENCODER_TICKS_PER_REV 2200


class PicoComms{
    public:
        void sendchr(std::string data_byte)
        {
            // Write the data to the serial port.
            serial_port_.Write(data_byte) ;

            // Wait until the data has actually been transmitted.
            serial_port_.DrainWriteBuffer();
        }

        void writeMotor(int motorCommand1, int motorCommand2)
        {
            std::string string_data = std::to_string(motorCommand1) + " " + std::to_string(motorCommand2) + ".";
            sendchr(string_data);
        }

        void setPID(std::vector<double> set_gains)
        {
            Kp = set_gains[0];
            Ki = set_gains[1];
            Kd = set_gains[2];
        }

        void setvPID(std::vector<double> set_vgains)
        {
            vKp = set_vgains[0];
            vKi = set_vgains[1];
            vKd = set_vgains[2];
        }

        void readEncoder(float &encoder_val1, float &encoder_val2)
        {
            std::string read_buffer ;
            sendchr("\r");
            try
            {
                // Read as many bytes as are available during the timeout period.
                serial_port_.Read(read_buffer, 0, ms_timeout) ;
            }
            catch (const LibSerial::ReadTimeout&)
            {
                std::string delimiter = "\t";
                size_t del_pos = read_buffer.find(delimiter);
                std::string token_1 = read_buffer.substr(0, del_pos);
                std::string token_2 = read_buffer.substr(del_pos + delimiter.length());

                encoder_val1 = std::atoi(token_1.c_str())*2*PI/STEERING_ENCODER_TICKS_PER_REV;
                encoder_val2 = std::atoi(token_2.c_str())*2*PI/DRIVING_ENCODER_TICKS_PER_REV;
            }
        }

        void readDrivingEncoderVelocity(float &velocity)
        {
            auto new_time = std::chrono::system_clock::now();
            std::chrono::duration<float> diff = new_time - time_;
            float deltaSeconds = diff.count();
            time_ = new_time;
            float pos_now, _;
            readEncoder(_, pos_now);
            velocity = (pos_now - pos_prev) / deltaSeconds;
            pos_prev = pos_now;
        }

        void readSteeringDriving(float &pos_steering, float &pos_driving, float &vel_driving)
        {
            auto new_time = std::chrono::system_clock::now();
            std::chrono::duration<float> diff = new_time - time_;
            float deltaSeconds = diff.count();
            time_ = new_time;
            readEncoder(pos_steering, pos_driving);
            vel_driving = (pos_driving - pos_prev) / deltaSeconds;
            pos_prev = pos_driving;
        }

        void controlPosition(float targetPosition)
        {
            //https://vanhunteradams.com/Pico/ReactionWheel/Tuning.html
            float encoder_s, encoder_d;
            readEncoder(encoder_s, encoder_d);
            // Compute the error encoder_s is the current position
            float error = targetPosition - encoder_s;

            // Integrate the error
            errorIntegral += error;

            // Approximate the rate of change of the error
            float errorDerivative = (error - prev_error);

            // Clamp the integrated error (start with Imax = max_duty_cycle/2)
            if (errorIntegral>Imax) errorIntegral=Imax;
            if (errorIntegral<(-Imax)) errorIntegral=-Imax;

            float Command;

            Command = Kp*error + Ki*errorIntegral + Kd*errorDerivative;
            if (Command<70 && Command >0){
                Command = 70;
            }
            if (Command>-70 && Command <0){
                Command = -70;
            }
            
            // Update prev_error
            prev_error = error;

            std::cout << error << " "<< int(Command)<<std::endl;
            writeMotor( int(Command), 0);
        }

        void controlVelocity(float targetVelocity)
        {
            float velocity;
            readDrivingEncoderVelocity(velocity);

              // Low-pass filter (25 Hz cutoff) https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl/SpeedControl.ino
            vfilt = 0.9*vfilt + 0.1*velocity;
            // vfilt = 0.854*vfilt + 0.0728*velocity + 0.0728*vprev;
            vprev = velocity;
            // Compute the error encoder_s is the current position
            float verror = targetVelocity - vfilt;

            // Integrate the error
            verrorIntegral += verror;

            // Approximate the rate of change of the error
            float verrorDerivative = (verror - vprev_error);

            // Clamp the integrated error (start with Imax = max_duty_cycle/2)
            if (verrorIntegral>vImax) errorIntegral=vImax;
            if (verrorIntegral<(-vImax)) errorIntegral=-vImax;

            
            float vCommand;
            vCommand = vKp*verror + vKi*verrorIntegral + vKd*verrorDerivative;
            
            
            // Update prev_error
            vprev_error = verror;

            std::cout << vfilt << " "<< verror << " "<< int(vCommand)<<std::endl;
            writeMotor(0, int(vCommand));
        }

        void controlLeg(float targetPosition, float targetVelocity)
        {
            // Control Position ________________________________________

            //https://vanhunteradams.com/Pico/ReactionWheel/Tuning.html
            float pos_s, pos_d, vel_d;
            readSteeringDriving(pos_s, pos_d, vel_d);
            // Compute the error encoder_s is the current position
            float error = targetPosition - pos_s;

            // Integrate the error
            errorIntegral += error;

            // Approximate the rate of change of the error
            float errorDerivative = (error - prev_error);

            // Clamp the integrated error (start with Imax = max_duty_cycle/2)
            if (errorIntegral>Imax) errorIntegral=Imax;
            if (errorIntegral<(-Imax)) errorIntegral=-Imax;

            float Command;

            Command = Kp*error + Ki*errorIntegral + Kd*errorDerivative;
            if (Command<MIN_COMMAND && Command >0){
                Command = MIN_COMMAND;
            }
            if (Command>-MIN_COMMAND && Command <0){
                Command = -MIN_COMMAND;
            }

// Steering Joint Limits 
            if (pos_s>PI || pos_s<-PI){
                Command = 0.0;
            }
            
            // Update prev_error
            prev_error = error;

            
            // Control Velocity ________________________________________

              // Low-pass filter (25 Hz cutoff) https://github.com/curiores/ArduinoTutorials/blob/main/SpeedControl/SpeedControl/SpeedControl.ino
            vfilt = 0.5*vfilt + 0.5*vel_d;
            // vfilt = 0.854*vfilt + 0.0728*velocity + 0.0728*vprev;
            vprev = vel_d;
            // Compute the error encoder_s is the current position
            float verror = targetVelocity - vfilt;

            // Integrate the error
            verrorIntegral += verror;

            // Approximate the rate of change of the error
            float verrorDerivative = (verror - vprev_error);

            // Clamp the integrated error (start with Imax = max_duty_cycle/2)
            if (verrorIntegral>vImax) verrorIntegral=vImax;
            if (verrorIntegral<(-vImax)) verrorIntegral=-vImax;

            
            float vCommand;
            vCommand = vKp*verror; // + vKi*verrorIntegral + vKd*verrorDerivative;
            
            
            if (vCommand<MIN_COMMAND && vCommand >0){
                vCommand = MIN_COMMAND;
            }
            if (vCommand>-MIN_COMMAND && Command <0){
                vCommand = -MIN_COMMAND;
            }
            // Update prev_error
            vprev_error = verror;

            writeMotor(int(Command), int(vCommand));
        }

        int connect(const std::string serial_device_)
        {
            try
            {
                // Open the Serial Port at the desired hardware port.
                serial_port_.Open(serial_device_) ;
                std::cout << "SERIAL PORT " << serial_device_ << " opened successfully!" << std::endl;
            }
            catch (const LibSerial::OpenFailed&)
            {
                std::cerr << "The serial port did not open correctly." << std::endl ;
                return EXIT_FAILURE ;
            }

            // Set the baud rate of the serial port.
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200) ;

            // Set the number of data bits.
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8) ;

            // Turn off hardware flow control.
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE) ;

            // Disable parity.
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE) ;
            
            // Set the number of stop bits.
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1) ;

            return 0;
        }
        void disconnect()
        {
            serial_port_.Close();
        }

        bool connected() const
        {
            return serial_port_.IsOpen();
        }

    private:
        LibSerial::SerialPort serial_port_ ;
        std::string serial_device_ ;
        size_t ms_timeout = 10 ;

        // PID Params
        float Kp = 150;
        float Ki = 0.0;
        float Kd = 0.0;
        float Imax = 128;
        float errorIntegral;
        float prev_error;
        float pos_prev = 0;

        float MIN_COMMAND = 30;

        float vKp = 6;
        float vKi = 0.0;
        float vKd = 0.4;
        float vImax = 128;
        float verrorIntegral;
        float vprev = 0;
        float vfilt = 0;
        float vprev_error;
        std::chrono::time_point<std::chrono::system_clock> time_ = std::chrono::system_clock::now();
        
};

#endif
