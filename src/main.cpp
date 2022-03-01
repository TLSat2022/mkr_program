#include <Arduino.h>
#include <arduino-timer.h>
#include <LSM6.h>
#include <PID_v1.h>
#include <RadioLib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <VMA430_GPS.h>
#include <Adafruit_BMP280.h>

#define LIGHT_SENSOR_PIN A0

enum class Error : uint16_t {
    BMP_ERROR = 0x01,
    IMU_ERROR = 0x02,
    SD_ERROR = 0x04,
    GPS_ERROR = 0x08,
    RADIO_ERROR = 0x10,
};

struct MainData {
    double pressure;
    double temperature;
    struct {
        int16_t x, y, z;
    } acceleration;
    struct {
        int16_t x, y, z;
    } angular_velocity;
    struct {
        double latitude;
        double longitude;
    } location;
    int light_value;
};

uint16_t error;
MainData data;

void set_error(Error err){
    error |= (uint16_t)err;
}

void unset_error(Error err){
    error &= ~(uint16_t)err;
}

Adafruit_BMP280 bmp;
LSM6 imu;
VMA430_GPS gps(&Serial1);
Timer<10> timer;
SX1278 radio = new Module(7, 6, RADIOLIB_NC, RADIOLIB_NC);
File file;

#define Kp 2
#define Ki 5
#define Kd 1

double setpoint, input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int transmissionState = RADIOLIB_ERR_NONE;

bool read_bmp(void*);
bool read_imu(void*);
bool read_pos(void*);
bool save_and_send(void*);
bool read_light(void*);
void message_sent();

void setup() {
    Wire.begin();

    while (!bmp.begin(0x76))
    {
        set_error(Error::BMP_ERROR);
        delay(500);
    }

    while(!imu.init())
    {
        set_error(Error::IMU_ERROR);
        delay(500);
    }

    while(!SD.begin(SDCARD_SS_PIN)){
        set_error(Error::SD_ERROR);
        delay(500);
    }
    gps.begin(9600);
    gps.setUBXNav();

    int state = ~RADIOLIB_ERR_NONE;
    while(state != RADIOLIB_ERR_NONE)
    {
        state = radio.begin();
        Serial.print("Radio not working, this is state: ");
        Serial.println(state);
        delay(500);
    }

    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    imu.enableDefault();

    timer.every(1000L, read_bmp);
    timer.every(1000L, read_imu);
    timer.every(1000L, save_and_send);
    timer.every(1000L, read_pos);
    timer.every(1000L, read_light);
}

void loop() {
    timer.tick();
}

bool read_bmp(void*){
    if(!bmp.takeForcedMeasurement())
    {
        set_error(Error::BMP_ERROR);
        return true;
    }

    data.temperature = bmp.readTemperature();
    data.pressure = bmp.readPressure();

    return true;
}

bool read_imu(void*){
    imu.read();

    data.acceleration = {imu.a.x, imu.a.y, imu.a.z};
    data.angular_velocity = {imu.g.x, imu.g.y, imu.g.z};
}

bool save_and_send(void*) {
    // Here save and transmit data
    Serial.println("We are actually sending");
    file = SD.open("data.txt");
    String report = String(data.pressure) + ";" +
            String(data.pressure) + ";" +
            String(data.temperature) + ";" +
            String(data.location.latitude) + ";" +
            String(data.location.longitude) + ";" +
            String(data.light_value) + ";" +
            String(data.acceleration.x) + ";" +
            String(data.acceleration.y) + ";" +
            String(data.acceleration.z) + ";" +
            String(data.angular_velocity.x) + ";" +
            String(data.angular_velocity.y) + ";" +
            String(data.angular_velocity.z) + ";";
    file.println(report);
    file.close();
    radio.transmit(report);

}

bool read_pos(void*){
    if(!gps.getUBX_packet()){
        set_error(Error::GPS_ERROR);
        return true;
    }

    if(!gps.location.valid) set_error(Error::GPS_ERROR);
    data.location.longitude = gps.location.longitude;
    data.location.latitude = gps.location.latitude;
}

void message_sent(){

}

bool read_light(void*) {
    data.light_value = analogRead(LIGHT_SENSOR_PIN);
}