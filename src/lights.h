#ifndef LIGHTS_H
#define LIGHTS_H

#include <string>
#include "boost/asio.hpp"

#include <QTimer>
#include <QtCore/QTime>

#include <QObject>

class Lights : public QObject
{
    Q_OBJECT
public:
    Lights(std::string port="/dev/ttyACM0", unsigned int baud_rate=115200);
    ~Lights() {}

public slots:
    void lighting(int number, int intensity, int duration);
    void lights_off();

private:
    boost::asio::io_service io_service;
    boost::asio::serial_port serial_port;

    QTimer *lightTimer;
    

};

#endif // LIGHTS_H
