#ifndef MICROCONTROLLER_H
#define MICROCONTROLLER_H

#include <CPS4042/Hardwares/Boards/Esp8266.h>
#include <CPS4042/Sketchs/AbstractSketch.h>
#include <CPS4042/Utils/ByteStream.h>
#include <CPS4042/Utils/Wave.h>
#include <bitset>

class MicroController : public AbstractSketch<Boards::Esp8266>
{
public:
    explicit MicroController(Boards::Esp8266* node) :
        AbstractSketch<Boards::Esp8266> {node}
    {
        data = 0;
    }

    std::int32_t
    setup(Boards::Esp8266::Gpio& gpio) override
    {
        std::cout << "esp8266 setup completed." << std::endl;
        node()->i2c.init(0x29);
        node()->i2c.start_communicating(true);
        // delay(1'000);
        node()->usart.start();
        return 0;
    }

    std::int32_t
    loop(Boards::Esp8266::Gpio& gpio) override
    {
        std::cout << "Esp" << std::endl;
        if(node()->usart.has_address()) {
            node()->usart.write(0x22);
        }
        //7
        if((!node()->usart.has_received_byte()) && node()->usart.has_sensed_zero() && node()->usart.has_address()) {
            data = node()->usart.read();
            std::cout << "Esp::Received data is:" << (int)data << std::endl; 
        }
        //6
        if(!node()->usart.has_sensed_zero() && node()->usart.has_address()) {
            node()->usart.set_sensed_zero();
            
        }
        //8
        if(!node()->usart.has_sensed_one() && node()->usart.has_received_byte() &&
            node()->usart.has_sensed_zero() && node()->usart.has_address()) {
            node()->usart.set_sensed_one();
        }
        //9
        if(node()->usart.has_received_byte() && node()->usart.has_sensed_one()) {
            std::cout << "Esp::done!" << std::endl;
            node()->usart.reset();
        }

        // -------------------------------------------------I2C--------------------------------------------------------------
        if(node()->i2c.start_communication && !node()->i2c.address_sent)
        {
            node()->i2c.send_address();
        }

        if(node()->i2c.waitingForAck && !node()->i2c.ackReceived)
        {
            node()->i2c.wait_for_recieving_ACK();
        }

        if(node()->i2c.ackReceived && node()->i2c.byteCount < 3)
        {
            node()->i2c.recieve_3_bytes_of_data();
        }
        // delay(100);
        return 0;
    }
    private:
    Byte data;
};


#endif    // MICROCONTROLLER_H
