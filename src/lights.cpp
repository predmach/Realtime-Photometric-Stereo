#include "lights.h"


#include <cstdlib>
#include <fstream>
#include <iostream>

Lights::Lights(std::string port, unsigned int baud_rate)
    : io_service(), serial_port(io_service, port)
{
  serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
  lightTimer = new QTimer();
  lightTimer->setSingleShot(true);
  connect(lightTimer, SIGNAL(timeout()), this, SLOT(lights_off()));
//  QTimer::singleShot(1000, this, [this]{
//     emit Log("Timeout...");
//     TestFunc();
//  });
}

void Lights::lighting(int number, int intensity, int duration)
{
    std::string command = "[ON W " + std::to_string(intensity) + " 1 " + std::to_string(number) + "]";
    boost::asio::write(serial_port, boost::asio::buffer(
                           command.c_str(), command.size()));

    lightTimer->start(duration);
}

void Lights::lights_off()
{
    std::string command = "[OFF]";
    boost::asio::write(serial_port, boost::asio::buffer(
                           command.c_str(), command.size()));
}
