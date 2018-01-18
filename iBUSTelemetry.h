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

#ifndef iBUSTelemetry_h
#define iBUSTelemetry_h_h

#include <inttypes.h>
#include <Stream.h>
#include <iBUSSensors.h>

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#ifndef GCC_VERSION
# define GCC_VERSION    (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class iBUSTelemetry : public Stream
{
private:
    uint8_t _pin;
    uint8_t _pinBitMask;
    volatile uint8_t * _receivePortRegister;
    volatile uint8_t * _transmitPortRegister;
    volatile uint8_t * _pcint_maskreg;
    uint8_t _pcint_maskvalue;

    uint16_t _rx_delay_centering;
    uint16_t _rx_delay_intrabit;
    uint16_t _rx_delay_stopbit;
    uint16_t _tx_delay;

    bool _buffer_overflow;

    static char _receive_buffer[_SS_MAX_RX_BUFF];
    static volatile uint8_t _receive_buffer_tail;
    static volatile uint8_t _receive_buffer_head;
    static iBUSTelemetry * active_object;

    void
    recv() __attribute__((__always_inline__));
    uint8_t
    rx_pin_read();

    void
    setPin(uint8_t pin);
    void
    setRxIntMsk(bool enable) __attribute__((__always_inline__));

    static uint16_t
    subtract_cap(uint16_t num, uint16_t sub);

    static inline void
    tunedDelay(uint16_t delay);

    uint8_t sensorCount;

    typedef struct iBUSSensor {
        uint8_t type;
        int32_t value;
    } iBUSSensor;

    iBUSSensor sensorArray[MAX_SENS_COUNT];

    uint8_t
    getSensorSize(uint8_t type);

public:
    iBUSTelemetry(uint8_t receivePin);
    ~iBUSTelemetry();
    void
    begin();
    void
    run();
    bool
    listen();
    void
    end();
    bool
    isListening(){ return this == active_object; }

    bool
    stopListening();
    bool
    overflow(){ bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }

    int
    peek();

    void
    addSensor(uint8_t type);
    void
    setSensorValue(uint8_t adr, int32_t value);
    void
    setSensorValueFP(uint8_t adr, float value);
    uint16_t
    gpsStateValues(uint8_t firstVal, uint8_t secondVal);

    virtual size_t
    write(uint8_t byte);
    virtual int
    read();
    virtual int
    available();
    virtual void
    flush();
    operator bool (){ return true; }

    using Print::write;

    static inline void
    handle_interrupt() __attribute__((__always_inline__));
};
#endif // ifndef iBUSTelemetry_h