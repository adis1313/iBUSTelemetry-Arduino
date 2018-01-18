/*
 * iBUSTelemetry.cpp - adis1313
 *
 * Libraries or parts of the code used in this project:
 *
 * SoftwareSerialWithHalfDuplex
 * https://github.com/nickstedman/SoftwareSerialWithHalfDuplex
 *
 * iBUStelemetry
 * https://github.com/Hrastovc/iBUStelemetry
 *
 * FlySkyI6
 * https://github.com/qba667/FlySkyI6/blob/master/source/source/ibustelemetry.h
 *
 * Big thanks to the authors!
 */

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <iBUSTelemetry.h>
#include <util/delay_basic.h>

iBUSTelemetry * iBUSTelemetry::active_object = 0;
char iBUSTelemetry::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t iBUSTelemetry::_receive_buffer_tail = 0;
volatile uint8_t iBUSTelemetry::_receive_buffer_head = 0;

inline void
iBUSTelemetry::tunedDelay(uint16_t delay)
{
    _delay_loop_2(delay);
}

bool
iBUSTelemetry::listen()
{
    if (!_rx_delay_stopbit)
        return false;

    if (active_object != this) {
        if (active_object)
            active_object->stopListening();

        _buffer_overflow     = false;
        _receive_buffer_head = _receive_buffer_tail = 0;
        active_object        = this;
        setRxIntMsk(true);
        return true;
    }
    return false;
}

bool
iBUSTelemetry::stopListening()
{
    if (active_object == this) {
        setRxIntMsk(false);
        active_object = NULL;
        return true;
    }
    return false;
}

void
iBUSTelemetry::recv()
{
    uint8_t d = 0;

    if (!rx_pin_read()) {
        setRxIntMsk(false);
        tunedDelay(_rx_delay_centering);

        for (uint8_t i = 8; i > 0; --i) {
            tunedDelay(_rx_delay_intrabit);
            d >>= 1;
            if (rx_pin_read())
                d |= 0x80;
        }

        uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
        if (next != _receive_buffer_head) {
            _receive_buffer[_receive_buffer_tail] = d;
            _receive_buffer_tail = next;
        } else   {
            _buffer_overflow = true;
        }

        tunedDelay(_rx_delay_stopbit);
        setRxIntMsk(true);
    }
}

uint8_t
iBUSTelemetry::rx_pin_read()
{
    return *_receivePortRegister & _pinBitMask;
}

inline void
iBUSTelemetry::handle_interrupt()
{
    if (active_object) {
        active_object->recv();
    }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
    iBUSTelemetry::handle_interrupt();
}
#endif

iBUSTelemetry::iBUSTelemetry(uint8_t pin) :
    _rx_delay_centering(0),
    _rx_delay_intrabit(0),
    _rx_delay_stopbit(0),
    _tx_delay(0),
    _buffer_overflow(false)
{
    setPin(pin);
    sensorCount = 0;
}

iBUSTelemetry::~iBUSTelemetry()
{
    end();
}

void
iBUSTelemetry::setPin(uint8_t pin)
{
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
    _pin        = pin;
    _pinBitMask = digitalPinToBitMask(pin);
    uint8_t port = digitalPinToPort(pin);
    _transmitPortRegister = portOutputRegister(port);
    _receivePortRegister  = portInputRegister(port);
}

uint16_t
iBUSTelemetry::subtract_cap(uint16_t num, uint16_t sub)
{
    if (num > sub)
        return num - sub;
    else
        return 1;
}

void
iBUSTelemetry::begin()
{
    _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

    uint16_t bit_delay = (F_CPU / 115200) / 4;

    _tx_delay = subtract_cap(bit_delay, 15 / 4);

    if (digitalPinToPCICR(_pin)) {
        _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);
        _rx_delay_intrabit  = subtract_cap(bit_delay, 23 / 4);
        _rx_delay_stopbit   = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);

        *digitalPinToPCICR(_pin) |= _BV(digitalPinToPCICRbit(_pin));
        _pcint_maskreg   = digitalPinToPCMSK(_pin);
        _pcint_maskvalue = _BV(digitalPinToPCMSKbit(_pin));

        tunedDelay(_tx_delay);
    }
    listen();
}

void
iBUSTelemetry::setRxIntMsk(bool enable)
{
    if (enable)
        *_pcint_maskreg |= _pcint_maskvalue;
    else
        *_pcint_maskreg &= ~_pcint_maskvalue;
}

void
iBUSTelemetry::end()
{
    stopListening();
}

int
iBUSTelemetry::read()
{
    if (!isListening())
        return -1;

    if (_receive_buffer_head == _receive_buffer_tail)
        return -1;

    uint8_t d = _receive_buffer[_receive_buffer_head];
    _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
    return d;
}

int
iBUSTelemetry::available()
{
    if (!isListening())
        return 0;

    return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t
iBUSTelemetry::write(uint8_t b)
{
    if (_tx_delay == 0) {
        setWriteError();
        return 0;
    }

    volatile uint8_t * reg = _transmitPortRegister;
    uint8_t reg_mask       = _pinBitMask;
    uint8_t inv_mask       = ~_pinBitMask;
    uint8_t oldSREG        = SREG;
    uint16_t delay         = _tx_delay;

    cli();
    pinMode(_pin, OUTPUT);
    *reg &= inv_mask;
    tunedDelay(delay);

    for (uint8_t i = 8; i > 0; --i) {
        if (b & 1)
            *reg |= reg_mask;
        else
            *reg &= inv_mask;

        tunedDelay(delay);
        b >>= 1;
    }

    *reg |= reg_mask;
    pinMode(_pin, INPUT);
    *reg |= reg_mask; // ?

    SREG = oldSREG;
    tunedDelay(_tx_delay);

    return 1;
} // iBUSTelemetry::write

void
iBUSTelemetry::flush()
{
    if (!isListening())
        return;

    uint8_t oldSREG = SREG;
    cli();
    _receive_buffer_head = _receive_buffer_tail = 0;
    SREG = oldSREG;
}

int
iBUSTelemetry::peek()
{
    if (!isListening())
        return -1;

    if (_receive_buffer_head == _receive_buffer_tail)
        return -1;

    return _receive_buffer[_receive_buffer_head];
}

void
iBUSTelemetry::addSensor(uint8_t type)
{
    if (sensorCount < MAX_SENS_COUNT) {
        sensorArray[sensorCount].type  = type;
        sensorArray[sensorCount].value = 0;
        sensorCount++;
    }
}

void
iBUSTelemetry::setSensorValue(uint8_t adr, int32_t value)
{
    uint8_t sensAdrArr = adr - 1;

    if ((adr > 0) && (adr <= sensorCount)) {
        switch (sensorArray[sensAdrArr].type) {
            // Some empty cases to prevent setting these values by default case
            case 0x00:
                // IBUS_MEAS_TYPE_INTV
                break;
            case 0x01:
                sensorArray[sensAdrArr].value = value + 400;
                break;
            case 0x02:
                // IBUS_MEAS_TYPE_MOT
                break;
            case 0x7C:
                // IBUS_MEAS_TYPE_ODO1
                break;
            case 0x7D:
                // IBUS_MEAS_TYPE_ODO2
                break;
            case 0x7F:
                // IBUS_MEAS_TYPE_TX_V
                break;
            default:
                sensorArray[sensAdrArr].value = value;
        }
    }
}

void
iBUSTelemetry::setSensorValueFP(uint8_t adr, float value)
{
    uint8_t sensAdrArr = adr - 1;

    if ((adr > 0) && (adr <= sensorCount)) {
        switch (sensorArray[sensAdrArr].type) {
            case 0x01:
                setSensorValue(adr, (int32_t) (value * 10));
                break;
            case 0x03:
                sensorArray[sensAdrArr].value = (uint16_t) (value * 100);
                break;
            case 0x04:
                sensorArray[sensAdrArr].value = (uint16_t) (value * 100);
                break;
            case 0x05:
                sensorArray[sensAdrArr].value = (uint16_t) (value * 100);
                break;
            case 0x06:
                sensorArray[sensAdrArr].value = (uint16_t) value;
                break;
            case 0x07:
                sensorArray[sensAdrArr].value = (uint16_t) value;
                break;
            case 0x08:
                sensorArray[sensAdrArr].value = (uint16_t) value;
                break;
            case 0x09:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x0A:
                sensorArray[sensAdrArr].value = (uint16_t) value;
                break;
            case 0x0B:
                sensorArray[sensAdrArr].value = value;
                break;
            case 0x0C:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x0D:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x0E:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x0F:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x10:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x11:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x12:
                sensorArray[sensAdrArr].value = (int16_t) (value * 100);
                break;
            case 0x13:
                sensorArray[sensAdrArr].value = (uint16_t) (value * 100);
                break;
            case 0x14:
                sensorArray[sensAdrArr].value = (uint16_t) value;
                break;
            case 0x15:
                sensorArray[sensAdrArr].value = value;
                break;
            case 0x16:
                sensorArray[sensAdrArr].value = value;
                break;

            case 0x41:
                sensorArray[sensAdrArr].value = (uint16_t) (value * 100);
                break;
            case 0x7E:
                sensorArray[sensAdrArr].value = (uint16_t) (value * 10);
                break;

            case 0x80:
                sensorArray[sensAdrArr].value = (int32_t) (value * 10000000);
                break;
            case 0x81:
                sensorArray[sensAdrArr].value = (int32_t) (value * 10000000);
                break;
            case 0x82:
                sensorArray[sensAdrArr].value = (int32_t) (value * 100);
                break;
            case 0x83:
                sensorArray[sensAdrArr].value = (int32_t) (value * 100);
                break;
            case 0x84:
                sensorArray[sensAdrArr].value = (int32_t) (value * 100);
                break;

            case 0x85:
                sensorArray[sensAdrArr].value = (uint32_t) value;
                break;
            case 0x86:
                sensorArray[sensAdrArr].value = (uint32_t) value;
                break;
            case 0x87:
                sensorArray[sensAdrArr].value = (uint32_t) value;
                break;
            case 0x88:
                sensorArray[sensAdrArr].value = (uint32_t) value;
                break;
            case 0x89:
                sensorArray[sensAdrArr].value = (uint32_t) value;
                break;
            case 0x8A:
                sensorArray[sensAdrArr].value = (uint32_t) value;
                break;
        }
    }
} // iBUSTelemetry::setSensorValueFP

uint8_t
iBUSTelemetry::getSensorSize(uint8_t type)
{
    return (type < 0x80) ? 2 : 4;
}

uint16_t
iBUSTelemetry::gpsStateValues(uint8_t firstVal, uint8_t secondVal)
{
    return (secondVal << 8) | firstVal;
}

void
iBUSTelemetry::run()
{
    if ((available() >= 4) && (sensorCount > 0)) {
        bool msgReceived = 0;
        uint8_t len;
        uint8_t msgAdr;
        uint8_t msg;
        uint8_t sensAdr;
        uint8_t sensAdrArr;
        uint8_t sensSize;
        uint8_t pByte;
        uint16_t checksum;
        uint16_t checksumCalc;

        do {
            len          = _receive_buffer[_receive_buffer_head];
            msgAdr       = _receive_buffer[(_receive_buffer_head + 1) % _SS_MAX_RX_BUFF];
            checksum     = _receive_buffer[(_receive_buffer_head + 2) % _SS_MAX_RX_BUFF];
            checksum    |= _receive_buffer[(_receive_buffer_head + 3) % _SS_MAX_RX_BUFF] << 8;
            checksumCalc = 0xFFFF - (len + msgAdr);

            if (checksumCalc == checksum) {
                msgReceived = 1;
                _receive_buffer_head = (_receive_buffer_head + 4) % _SS_MAX_RX_BUFF;
            } else {
                msgReceived = 0;
                _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
            }
        } while ((!msgReceived) && (_receive_buffer_head != _receive_buffer_tail));

        if (msgReceived) {
            msg        = msgAdr >> 4;
            sensAdr    = msgAdr & 0b1111;
            sensAdrArr = sensAdr - 1;
            sensSize   = getSensorSize(sensorArray[sensAdrArr].type);

            switch (msg) {
                case 0b1000:
                    delayMicroseconds(100);
                    if (sensAdr <= sensorCount) {
                        write(0x04);
                        write((0b1000 << 4) + sensAdr);
                        checksum = 0xFFFF - (0x04 + ((0b1000 << 4) + sensAdr));
                        write(checksum);
                        write(checksum >> 8);
                    }
                    break;
                case 0b1001:
                    delayMicroseconds(100);
                    write(0x06);
                    write((0b1001 << 4) + sensAdr);
                    write(sensorArray[sensAdrArr].type);
                    write(sensSize);
                    checksum = 0xFFFF - (0x06 + ((0b1001 << 4) + sensAdr) + sensorArray[sensAdrArr].type + sensSize);
                    write(checksum);
                    write(checksum >> 8);
                    break;
                case 0b1010:
                    delayMicroseconds(100);
                    write(sensSize + 4);
                    write((0b1010 << 4) + sensAdr);
                    checksum = 0xFFFF - ((sensSize + 4) + ((0b1010 << 4) + sensAdr));

                    for (int i = 0; i < sensSize; i++) {
                        pByte     = (sensorArray[sensAdrArr].value >> (8 * i)) & 0xFF;
                        checksum -= pByte;
                        write(pByte);
                    }

                    write(checksum);
                    write(checksum >> 8);
                    break;
            }
        }
    }
} // iBUSTelemetry::run