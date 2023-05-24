#include <Wire.h>
#include <EEPROM.h>

float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 18.0;
int pid_max_roll = 400;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;
boolean auto_level = true;

byte last_channel[4];
byte eeprom_data[36];
volatile int receiver_input_channel[4];
int esc[4];
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
TWBR = 12;
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long esc_loop_timer;
unsigned long current_time;
unsigned long timer[4];
unsigned long timer_channel[4];
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;

void setup(){
    for (int i = 0; i < 36; i++) {
        eeprom_data[i] = EEPROM.read(i);
    }
    gyro_address = eeprom_data[32];
    int gyro_calibration_samples = eeprom_data[31] == 2 || eeprom_data[31] == 3 ? 1250 : 0;
    Wire.begin();
    DDRD |= 0xF0;
    DDRB |= 0x30;
    digitalWrite(12, HIGH);
    while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B') {
        delay(10);
    }
    
    //set registers for reciever
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);                                  
    PCMSK0 |= (1 << PCINT1);                                
    PCMSK0 |= (1 << PCINT2);                                 
    PCMSK0 |= (1 << PCINT3);  
    
    for (int i = 0; i < gyro_calibration_samples; i++) {
        PORTD |= 0xF0;
        delayMicroseconds(1000);
        PORTD &= 0x0F;
        delayMicroseconds(3000);
        gyro_signalen();
        for (int axis = 0; axis < 3; axis++) {
            gyro_axis_cal[axis] += gyro_axis[axis];
        }
    }
    for (int i = 0; i < 3; i++) {
        gyro_axis_cal[i] /= gyro_calibration_samples;
    }
    battery_voltage = (analogRead(0) + 65) * 1.2317;
    loop_timer = micros();
    digitalWrite(12, LOW);

}

void loop() {
    gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);
    gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
    gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);

    angle_pitch += gyro_pitch * 0.0000611;
    angle_roll += gyro_roll * 0.0000611;

    angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
    angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);

    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

    if (abs(acc_y) < acc_total_vector) {
        angle_pitch_acc = asin((float) acc_y / acc_total_vector) * 57.296;
    }
    if (abs(acc_x) < acc_total_vector) {
        angle_roll_acc = asin((float) acc_x / acc_total_vector) * -57.296;
    }

    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

    pitch_level_adjust = angle_pitch * 15;
    roll_level_adjust = angle_roll * 15;

    if (!auto_level) {
        pitch_level_adjust = 0;
        roll_level_adjust = 0;
    }

    if (receiver_input_channel[2] < 1050 && receiver_input_channel[3] < 1050)
        start = 1;
    bool isStartRequested = (start == 1 && receiver_input_channel[2] < 1050 && receiver_input_channel[3] > 1450);

    if (isStartRequested) {
        start = 2;

        angle_pitch = angle_pitch_acc;
        angle_roll = angle_roll_acc;
        gyro_angles_set = true;

        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
    }

    if (start == 2 && receiver_input_channel[2] < 1050 && receiver_input_channel[3] > 1950)
        start = 0;

    pid_roll_setpoint = 0;
    if (receiver_input_channel[0] > 1508)
        pid_roll_setpoint = receiver_input_channel[0] - 1508;
    else if (receiver_input_channel[0] < 1492)
        pid_roll_setpoint = receiver_input_channel[0] - 1492;

    pid_roll_setpoint -= roll_level_adjust;
    pid_roll_setpoint /= 3.0;

    pid_pitch_setpoint = 0;
    if (receiver_input_channel[1] > 1508)
        pid_pitch_setpoint = receiver_input_channel[1] - 1508;
    else if (receiver_input_channel[1] < 1492)
        pid_pitch_setpoint = receiver_input_channel[1] - 1492;

    pid_pitch_setpoint -= pitch_level_adjust;
    pid_pitch_setpoint /= 3.0;

    pid_yaw_setpoint = 0;
    if (receiver_input_channel[2] > 1050) {
        if (receiver_input_channel[3] > 1508)
            pid_yaw_setpoint = (receiver_input_channel[3] - 1508) / 3.0;
        else if (receiver_input_channel[3] < 1492)
            pid_yaw_setpoint = (receiver_input_channel[3] - 1492) / 3.0;
    }

    battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

    if (battery_voltage < 1000 && battery_voltage > 600)
        digitalWrite(12, HIGH);

    throttle = receiver_input_channel[2];

    if (start == 2) {
        // Ensure throttle is within safe range
        throttle = (throttle > 1800) ? 1800 : throttle;

        // Calculate ESC values based on PID outputs
        esc[0] = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
        esc[1] = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
        esc[2] = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
        esc[3] = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

        // Adjust ESC values based on battery voltage
        if (battery_voltage < 1240 && battery_voltage > 800) {
            float voltage_diff = (1240 - battery_voltage) / 3500.0;
            for (int i = 0; i < 4; i++) {
                esc[i] += esc[i] * voltage_diff;
            }
        }

        // Ensure ESC values are within safe range
        for (int i = 0; i < 4; i++) {
            esc[i] = (esc[i] < 1100) ? 1100 : ((esc[i] > 2000) ? 2000 : esc[i]);
        }

        if (battery_voltage < 800) {
            // Disable motors if battery voltage is too low
            for (int i = 0; i < 4; ++i) {
                esc[i] = 1000;
            }
        } else {
            // Enable motors and set timer channels for ESC signals
            if (micros() - loop_timer > 4050) {
                digitalWrite(12, HIGH);
            }
            while (micros() - loop_timer < 4000);
            loop_timer = micros();
            PORTD |= B11110000;
            for (int i = 0; i < 4; i++) {
                timer_channel[i] = esc[i] + loop_timer;
            }

            // Send signals to ESCs and update gyro signals
            gyro_signalen();
            while (PORTD >= 16) {
                esc_loop_timer = micros();
                if (timer_channel[0] <= esc_loop_timer) {
                    PORTD &= B11101111;
                }
                if (timer_channel[1] <= esc_loop_timer) {
                    PORTD &= B11011111;
                }
                if (timer_channel[2] <= esc_loop_timer) {
                    PORTD &= B10111111;
                }
                if (timer_channel[3] <= esc_loop_timer) {
                    PORTD &= B01111111;
                }
            }
        }
    }

    ISR(PCINT0_vect){
        current_time = micros();
        //Channel 1=========================================
        if(PINB & B00000001){                                   
            if(last_channel[0] == 0){                                     
                last_channel[0] = 1;                                          
                timer[0] = current_time;                                       
            }
        }
        else if(last_channel[0] == 1){                                
            last_channel[0] = 0;                                           
            receiver_input[1] = current_time - timer[0];                       
        }
        //Channel 2=========================================
        if(PINB & B00000010 ){                                       
            if(last_channel[1] == 0){                                   
                last_channel[1] = 1;                                     
                timer[1] = current_time;                           
            }
        }
        else if(last_channel[1] == 1){                                   
            last_channel[1];                                      
            receiver_input[2] = current_time - timer[1];             
        }
        //Channel 3=========================================
        if(PINB & B00000100 ){                                     
            if(last_channel[2] == 0){                              
                last_channel[2] = 1;                                        
                timer[2] = current_time;                                     
            }
        }
        else if(last_channel[2] == 1){                                       
            last_channel[2] = 0;                                           
            receiver_input[3] = current_time - timer[2];                       

        }
        //Channel 4=========================================
        if(PINB & B00001000 ){                                                 
            if(last_channel[3] == 0){                                   
                last_channel[3] = 1;                                        
                timer[3] = current_time;                                              
            }
        }
        else if(last_channel[3] == 1){                                             
            last_channel[3] = 0;                                                     
            receiver_input[4] = current_time - timer[3];                             
        }
    }

    void calculate_pid() {
        // Calculate roll PID output
        float roll_error = gyro_roll_input - pid_roll_setpoint;
        pid_i_mem_roll += pid_i_gain_roll * roll_error;
        pid_i_mem_roll = constrain(pid_i_mem_roll, -pid_max_roll, pid_max_roll);
        float roll_d_error = roll_error - pid_last_roll_error;
        pid_output_roll = pid_p_gain_roll * roll_error + pid_i_mem_roll + pid_d_gain_roll * roll_d_error;
        pid_output_roll = constrain(pid_output_roll, -pid_max_roll, pid_max_roll);
        pid_last_roll_error = roll_error;

        // Calculate pitch PID output
        float pitch_error = gyro_pitch_input - pid_pitch_setpoint;
        pid_i_mem_pitch += pid_i_gain_pitch * pitch_error;
        pid_i_mem_pitch = constrain(pid_i_mem_pitch, -pid_max_pitch, pid_max_pitch);
        float pitch_d_error = pitch_error - pid_last_pitch_error;
        pid_output_pitch = pid_p_gain_pitch * pitch_error + pid_i_mem_pitch + pid_d_gain_pitch * pitch_d_error;
        pid_output_pitch = constrain(pid_output_pitch, -pid_max_pitch, pid_max_pitch);
        pid_last_pitch_error = pitch_error;

        // Calculate yaw PID output
        float yaw_error = gyro_yaw_input - pid_yaw_setpoint;
        pid_i_mem_yaw += pid_i_gain_yaw * yaw_error;
        pid_i_mem_yaw = constrain(pid_i_mem_yaw, -pid_max_yaw, pid_max_yaw);
        float yaw_d_error = yaw_error - pid_last_yaw_error;
        pid_output_yaw = pid_p_gain_yaw * yaw_error + pid_i_mem_yaw + pid_d_gain_yaw * yaw_d_error;
        pid_output_yaw = constrain(pid_output_yaw, -pid_max_yaw, pid_max_yaw);
        pid_last_yaw_error = yaw_error;
    }

    void gyro_signalen() {
        if (eeprom_data[31] == 1) {
            Wire.beginTransmission(gyro_address);
            Wire.write(0x3B);
            Wire.endTransmission();
            Wire.requestFrom(gyro_address, 14);

            for (int i = 0; i < 4; i++) {
                receiver_input_channel[i] = convert_receiver_channel(i + 1);
            }

            while (Wire.available() < 14);

            for (int i = 0; i < 4; i++) {
                acc_axis[i] = Wire.read() << 8 | Wire.read();
            }

            for (int i = 0; i < 4; i++) {
                gyro_axis[i] = Wire.read() << 8 | Wire.read();
            }
        }

        if (cal_int == 2000) {
            for (int i = 0; i < 4; i++) {
                gyro_axis[i] -= gyro_axis_cal[i];
            }
        }

        gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
        gyro_roll *= (eeprom_data[28] & 0b10000000) ? -1 : 1;

        gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
        gyro_pitch *= (eeprom_data[29] & 0b10000000) ? -1 : 1;

        gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
        gyro_yaw *= (eeprom_data[30] & 0b10000000) ? -1 : 1;

        acc_x = acc_axis[eeprom_data[29] & 0b00000011];
        acc_x *= (eeprom_data[29] & 0b10000000) ? -1 : 1;

        acc_y = acc_axis[eeprom_data[28] & 0b00000011];
        acc_y *= (eeprom_data[28] & 0b10000000) ? -1 : 1;

        acc_z = acc_axis[eeprom_data[30] & 0b00000011];
        acc_z *= (eeprom_data[30] & 0b10000000) ? -1 : 1;
    }

    int convert_receiver_channel(byte function) {
        byte channel = eeprom_data[function + 23] & 0b00000111;
        bool reverse = (eeprom_data[function + 23] & 0b10000000) ? true : false;
        int actual = receiver_input[channel];
        int low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
        int center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
        int high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];
        int difference;

        if (actual < center) {
            if (actual < low) actual = low;
            difference = ((long) (center - actual) * (long) 500) / (center - low);
            return (reverse) ? (1500 + difference) : (1500 - difference);
        } else if (actual > center) {
            if (actual > high) actual = high;
            difference = ((long) (actual - center) * (long) 500) / (high - center);
            return (reverse) ? (1500 - difference) : (1500 + difference);
        } else {
            return 1500;
        }
    }

    void set_gyro_registers() {
        const byte GYRO_REG_NUM = 5;
        const byte gyro_reg_addr[GYRO_REG_NUM] = {0x6B, 0x1B, 0x1C, 0x1B, 0x1A};
        const byte gyro_reg_value[GYRO_REG_NUM] = {0x00, 0x08, 0x10, 0x00, 0x03};

        if (eeprom_data[31] == 1) {
            Wire.beginTransmission(gyro_address);

            for (byte i = 0; i < GYRO_REG_NUM; i++) {
                Wire.write(gyro_reg_addr[i]);
                Wire.write(gyro_reg_value[i]);
            }

            Wire.endTransmission();

            Wire.beginTransmission(gyro_address);
            Wire.write(0x1B);
            Wire.endTransmission();
            Wire.requestFrom(gyro_address, 1);
            while (Wire.available() < 1);
            if (Wire.read() != 0x08) {
                digitalWrite(12, HIGH);
                while (1) delay(10);
            }
        }
    }
}
