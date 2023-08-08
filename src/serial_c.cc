/*!
 * \file serial/serial_c.cc
 * \author  CaffreySun <shw930913@gmail.com>
 */

#include "serial/serial_c.h"
#include "serial/serial.h"

#define SERIAL_PORT auto *serial_port = (serial::Serial *) serial_instance

struct serial_port_info *serial_list_ports(size_t *list_len) {
    auto ports = serial::list_ports();
    auto *ps = new struct serial_port_info[ports.size()];

    for (int i = 0; i < ports.size(); ++i) {
        auto &pi = ports[i];

        char *port = new char[pi.port.length() + 1];
        strcpy(port, pi.port.c_str());
        ps[i].port = port;

        char *desc = new char[pi.description.length() + 1];
        strcpy(desc, pi.description.c_str());
        ps[i].description = desc;

        char *hardware_id = new char[pi.hardware_id.length() + 1];
        strcpy(hardware_id, pi.hardware_id.c_str());
        ps[i].hardware_id = hardware_id;
    }

    *list_len = ports.size();

    return ps;
}

void serial_free_port_list(struct serial_port_info *ports, size_t list_len) {
    if (ports == nullptr) {
        return;
    }

    for (int i = 0; i < list_len; ++i) {
        const char *port = ports[i].port;
        delete[] port;

        const char *desc = ports[i].description;
        delete[] desc;

        const char *hardware_id = ports[i].hardware_id;
        delete[] hardware_id;
    }

    delete[] ports;
}

unsigned int serial_timeout_max_num(void) {
    return serial::Timeout::max();
}

struct serial_timeout serial_timeout_zero(void) {
    return serial_timeout{0, 0, 0, 0, 0};
}

struct serial_timeout serial_simple_timeout(unsigned int timeout) {
    auto to = serial::Timeout::simpleTimeout(timeout);
    return {
            to.inter_byte_timeout,
            to.read_timeout_constant,
            to.read_timeout_multiplier,
            to.write_timeout_constant,
            to.write_timeout_multiplier,
    };
}

void *serial_new(const char *port_name) {
    return new serial::Serial(port_name);
}

void serial_free(void *serial_instance) {
    delete (serial::Serial *) serial_instance;
}

int serial_open(void *serial_instance) {
    SERIAL_PORT;
    try {
        serial_port->open();
        return 0;
    } catch (std::invalid_argument &) {
        return -1;
    } catch (serial::SerialException &) {
        return -2;
    } catch (serial::IOException &) {
        return -3;
    } catch (std::exception &e) {
        return -4;
    }
}

bool serial_is_open(void *serial_instance) {
    SERIAL_PORT;

    return serial_port->isOpen();
}

void serial_close(void *serial_instance) {
    SERIAL_PORT;

    serial_port->close();
}

size_t serial_available(void *serial_instance) {
    SERIAL_PORT;

    return serial_port->available();
}

bool serial_wait_readable(void *serial_instance) {
    SERIAL_PORT;
    return serial_port->waitReadable();
}

void serial_wait_byte_times(void *serial_instance, size_t count) {
    SERIAL_PORT;

    serial_port->waitByteTimes(count);
}

int serial_read(void *serial_instance, void *buffer, size_t size) {
    SERIAL_PORT;

    try {
        auto len = serial_port->read((uint8_t *) buffer, size);
        return static_cast<int>(len);
    } catch (serial::PortNotOpenedException &) {
        return -1;
    } catch (serial::SerialException &) {
        return -2;
    } catch (std::exception &) {
        return -3;
    }
}

int serial_write(void *serial_instance, const void *data, size_t size) {
    SERIAL_PORT;

    try {
        auto len = serial_port->write((uint8_t *) data, size);
        return static_cast<int>(len);
    } catch (serial::PortNotOpenedException &) {
        return -1;
    } catch (serial::SerialException &) {
        return -2;
    } catch (serial::IOException &) {
        return -3;
    } catch (std::exception &) {
        return -4;
    }
}

void serial_set_timeout(void *serial_instance, struct serial_timeout timeout) {
    SERIAL_PORT;

    serial::Timeout t{};
    t.inter_byte_timeout = timeout.inter_byte_timeout;
    t.read_timeout_constant = timeout.read_timeout_constant;
    t.read_timeout_multiplier = timeout.read_timeout_multiplier;
    t.write_timeout_constant = timeout.write_timeout_constant;
    t.write_timeout_multiplier = timeout.write_timeout_multiplier;

    serial_port->setTimeout(t);
}

struct serial_timeout serial_get_timeout(void *serial_instance) {
    SERIAL_PORT;

    auto t = serial_port->getTimeout();
    struct serial_timeout timeout{};
    timeout.inter_byte_timeout = t.inter_byte_timeout;
    timeout.read_timeout_constant = t.read_timeout_constant;
    timeout.read_timeout_multiplier = t.read_timeout_multiplier;
    timeout.write_timeout_constant = t.write_timeout_constant;
    timeout.write_timeout_multiplier = t.write_timeout_multiplier;

    return timeout;
}

void serial_set_baudrate(void *serial_instance, unsigned int baudrate) {
    SERIAL_PORT;

    serial_port->setBaudrate(baudrate);
}

unsigned int serial_get_baudrate(void *serial_instance) {
    SERIAL_PORT;

    return serial_port->getBaudrate();
}

void serial_set_bits(void *serial_instance, unsigned int bits) {
    SERIAL_PORT;

    serial_port->setBytesize((serial::bytesize_t) bits);
}

unsigned int serial_get_bits(void *serial_instance) {
    SERIAL_PORT;
    return serial_port->getBytesize();
}

void serial_set_parity(void *serial_instance, enum serial_parity parity) {
    SERIAL_PORT;

    serial_port->setParity((serial::parity_t) parity);
}

enum serial_parity serial_get_parity(void *serial_instance) {
    SERIAL_PORT;
    return (enum serial_parity) (serial_port->getParity());
}

void serial_set_stopbits(void *serial_instance, enum serial_stopbits stopbits) {
    SERIAL_PORT;

    serial_port->setStopbits((serial::stopbits_t) stopbits);
}

enum serial_stopbits serial_get_stopbits(void *serial_instance) {
    SERIAL_PORT;
    return (enum serial_stopbits) (serial_port->getStopbits());
}

void serial_set_flowcontrol(void *serial_instance, enum serial_flowcontrol flowcontrol) {
    SERIAL_PORT;

    serial_port->setFlowcontrol((serial::flowcontrol_t) flowcontrol);
}

enum serial_flowcontrol serial_get_flowcontrol(void *serial_instance) {
    SERIAL_PORT;
    return (enum serial_flowcontrol) (serial_port->getFlowcontrol());
}

void serial_flush(void *serial_instance) {
    SERIAL_PORT;

    serial_port->flush();
}

void serial_flush_input(void *serial_instance) {
    SERIAL_PORT;

    serial_port->flushInput();
}

void serial_flush_output(void *serial_instance) {
    SERIAL_PORT;

    serial_port->flushOutput();
}

void serial_send_break(void *serial_instance, int duration) {
    SERIAL_PORT;

    serial_port->sendBreak(duration);
}

void serial_set_break(void *serial_instance, bool level) {
    SERIAL_PORT;

    serial_port->setBreak(level);
}

void serial_set_rts(void *serial_instance, bool level) {
    SERIAL_PORT;

    serial_port->setRTS(level);
}

void serial_dtr(void *serial_instance, bool level) {
    SERIAL_PORT;

    serial_port->setDTR(level);
}

int serial_wait_for_change(void *serial_instance) {
    SERIAL_PORT;

    try {
        return serial_port->waitForChange() ? 1 : 0;
    } catch (std::exception &e) {
        return -1;
    }
}

bool serial_get_cts(void *serial_instance) {
    SERIAL_PORT;

    return serial_port->getCTS();
}

bool serial_get_dsr(void *serial_instance) {
    SERIAL_PORT;

    return serial_port->getDSR();
}

bool serial_get_ri(void *serial_instance) {
    SERIAL_PORT;

    return serial_port->getRI();
}

bool serial_get_cd(void *serial_instance) {
    SERIAL_PORT;

    return serial_port->getCD();
}