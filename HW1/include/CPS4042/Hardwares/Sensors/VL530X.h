#ifndef VL53_X_H
#define VL53_X_H

#include <CPS4042/Hardwares/Board.h>
#include <CPS4042/Units/BaudRate.h>
#include <CPS4042/Wires/Pin.h>
#include <boost/pfr.hpp>
#include <iostream>
#include <random>

namespace Sensors
{
using Vl530xVoltage = VoltageLevel3_3v;

template <std::uint64_t bar, std::uint64_t btr, typename WorkingVoltageTp>
requires std::is_base_of_v<AbstractVoltageLevel, WorkingVoltageTp>
struct Vl530xGpio
{
public:
    Pins::Vdd<WorkingVoltageTp> vdd {bar, btr, "Vl530x::vdd"};    // Pin 0
    Pins::Gnd<WorkingVoltageTp> gnd {bar, btr, "Vl530x::gnd"};    // Pin 1
    Pins::Sda<WorkingVoltageTp> sda {bar, btr, "Vl530x::sda"};    // Pin 2
    Pins::Scl<WorkingVoltageTp> scl {bar, btr, "Vl530x::scl"};    // Pin 3
};

class Vl530x : public Board<BaudRates::NotSpecified,
                            BitRates::same(BaudRates::NotSpecified),
                            Frequency::Drived, Vl530xVoltage, Vl530xGpio>
{
public:
    inline static constexpr Byte address = 0x29;

    explicit Vl530x() :
        Parent {"Vl530x::processor"}
    {
        m_processor->installProtocol(&i2c);

        std::cout << "one instance of Vl530x" << " created." << std::endl;
    }

    class I2C : public Protocols::AbstractI2C<Vl530x, Gpio>
    {
    public:
        explicit I2C(Vl530x* b) :
            Protocols::AbstractI2C<Vl530x, Gpio> {b}
        {}

        UByte buffer = 0;

        bool addressMatched = false;
        bool ackSent = false;

        //for sending bytes to the microcontroller
        Byte tx[3];
        int txIndex = 0;
        int sendBitIndex = 7;
        bool dataSent = false;

        void recieve_and_check_address()
        {
            if(m_board->gpio().sda.hasByteToRead())
            {
                Byte rec_byte = read();
                std::cout << "[Sensor] Received byte = "
                            << int(rec_byte) << std::endl;

                if(rec_byte == Vl530x::address)
                {
                    addressMatched = true;
                    std::cout << "[Sensor] Address matched!" << std::endl;
                    
                    send_ACK_to_Esp();
                    generate_2_bytes_and_checksum();
                }
            }

            return;
        }

        void send_ACK_to_Esp()
        {
            Bit ack = Bit::One;
            m_board->gpio().sda.write(ack);
            std::cout << "[Sensor] ACK sent = " << (int)ack <<std::endl;
            ackSent = true;

            return;
        }

        void send_3_bytes_of_data()
        {
            Byte curByte = tx[txIndex];
            write(curByte);

            std::cout << "[Sensor] Sending Byte " << txIndex
                << ": " << (int)(UByte)curByte << std::endl;

            txIndex++;

            if(txIndex == 3)
            {
                std::cout << "[Sensor] Transmission finished\n";
                dataSent = true;
                // addressMatched = false;
            }

            return;
        }

        std::uint16_t make_random_distance()
        {
            static std::mt19937 rng{std::random_device{}()};
            static std::uniform_int_distribution<std::uint16_t> dist(0, 4000);
            return dist(rng);
        }

        void generate_2_bytes_and_checksum()
        {
            std::uint16_t value = make_random_distance(); // بین 0 و 4000

            // تقسیم به دو بایت (Little Endian، مثل چیزی که الان استفاده می‌کنی)
            ByteVector<std::uint16_t> vec(value);
            tx[0] = getByte<0>(vec);   // LSB
            tx[1] = getByte<1>(vec);   // MSB

            // checksum = |tx[0] - tx[1]|
            tx[2] = std::abs((int)(UByte)tx[0] - (int)(UByte)tx[1]);


            txIndex = 0;
            std::cout << "[Sensor] Preparing for sending data to Esp." << std::endl; 
        }
 
        void init(Byte address) override
        {
            srand(time(nullptr));
            
            addressMatched = false;
            ackSent = false;
            dataSent = false;
        }

        void write(Byte byte) override 
        {
            m_board->gpio().sda.write(byte);       
            return;
        }

        Byte read() override
        {
            Byte r = m_board->gpio().sda.read();
            return r;
        }

        void run(Gpio& gpio) override
        {
            
        }
    } mutable i2c {this};


protected:
    inline void
    startModule() override
    {
        m_gpio.scl.onNextEdge([this](Vl530xVoltage level) {
            auto bit = Voltage::toBit(level);

            if(bit == Bit::One)    // positive edge
            {
                m_processor->nextCycle(m_gpio);
            }
        });
    }
};

}    // namespace Sensors

#endif    // VL53_X_H
