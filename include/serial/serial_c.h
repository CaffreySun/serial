/*!
 * \file serial/serial_c.h
 * \author  CaffreySun <shw930913@gmail.com>
 * \version 0.1
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for interacting with Serial Ports.
 */

#ifndef SERIAL_SERIAL_C_H
#define SERIAL_SERIAL_C_H

#if !defined(LIBSERIAL_API)
#  if defined(_WIN32)
#    if !defined(LIBSERIAL_DYNAMIC)
#      define LIBSERIAL_API
#    else
#      if defined(LIBSERIAL_EXPORTS)
#        define LIBSERIAL_API __declspec(dllexport)
#      else
#        define LIBSERIAL_API __declspec(dllimport)
#      endif
#    endif
#  else
#    define LIBSERIAL_API
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

/*! Parity settings. */
enum serial_parity {
    /** No parity. */
    SERIAL_PARITY_NONE = 0,
    /** Odd parity. */
    SERIAL_PARITY_ODD = 1,
    /** Even parity. */
    SERIAL_PARITY_EVEN = 2,
    /** Mark parity. */
    SERIAL_PARITY_MARK = 3,
    /** Space parity. */
    SERIAL_PARITY_SPACE = 4
};

/*!
 * Enumeration defines the possible stopbit types for the serial port.
 */
enum serial_stopbits {
    SERIAL_STOPBITS_ONE = 1,
    SERIAL_STOPBITS_TWO = 2,
    SERIAL_STOPBITS_ONE_DOT_FIVE
};

/** Standard flow control combinations. */
enum serial_flowcontrol {
    /** No flow control. */
    SERIAL_FLOWCONTROL_NONE = 0,
    /** Software flow control using XON/XOFF characters. */
    SERIAL_FLOWCONTROL_XONXOFF = 1,
    /** Hardware flow control using RTS/CTS signals. */
    SERIAL_FLOWCONTROL_RTSCTS = 2,
};

/*!
 * Structure that describes a serial device.
 */
struct serial_port_info {
    /*! Address of the serial port (this can be passed to the constructor of Serial). */
    const char *port;

    /*! Human readable description of serial device if available. */
    const char *description;

    /*! Hardware ID (e.g. VID:PID of USB serial devices) or "n/a" if not available. */
    const char *hardware_id;
};

/*!
 * Structure for setting the timeout of the serial port, times are
 * in milliseconds.
 *
 * In order to disable the interbyte timeout, set it to serial_timeout_max_num().
 */
struct serial_timeout {
    /*! Number of milliseconds between bytes received to timeout on. */
    unsigned int inter_byte_timeout;
    /*! A constant number of milliseconds to wait after calling read. */
    unsigned int read_timeout_constant;
    /*! A multiplier against the number of requested bytes to wait after
     *  calling read.
     */
    unsigned int read_timeout_multiplier;
    /*! A constant number of milliseconds to wait after calling write. */
    unsigned int write_timeout_constant;
    /*! A multiplier against the number of requested bytes to wait after
     *  calling write.
     */
    unsigned int write_timeout_multiplier;
};

/*！
 * See serial::list_ports.
 *
 * Lists the serial ports available on the system
 *
 * The result should be freed after use by calling serial_free_port_list().
 *
 * \return NULL if no ports are available.
 */
LIBSERIAL_API struct serial_port_info *serial_list_ports(size_t *list_len);

/*!
 * Free a port list obtained from serial_list_ports().
 *
 * This will also free all the serial_port_info structures referred to from the list;
 *
 * \param ports Pointer to a list of port structures. Must not be NULL.
 */
LIBSERIAL_API void serial_free_port_list(struct serial_port_info *ports, size_t list_len);

/*!
 * serial::Timeout::max();
 */
LIBSERIAL_API unsigned int serial_timeout_max_num(void);

LIBSERIAL_API struct serial_timeout serial_timeout_zero(void);

/*! See serial::Timeout::simpleTimeout();
 *
 * Convenience function to generate timeout structs using a
 * single absolute timeout.
 *
 * \param timeout A long that defines the time in milliseconds until a
 * timeout occurs after a call to read or write is made.
 *
 * \return timeout struct that represents this simple timeout provided.
 */
LIBSERIAL_API struct serial_timeout serial_simple_timeout(unsigned int timeout);

/*! See serial::Serial();
 *
 * Creates a Serial object and opens the port if a port is specified,
 * otherwise it remains closed until serial_open is called.
 * The serial default params are:
 *    baudrate: 9600
 *    bits: 8
 *    parity: none
 *    stopbits: 1
 *    flowcontrol: none
 *    timeout: serial_timeout_zero()
 *
 * \param port A const char * containing the address of the serial port,
 *        which would be something like 'COM1' on Windows and '/dev/ttyS0'
 *        on Linux.
 */
LIBSERIAL_API void *serial_new(const char *port_name);

/*!
 * Free a serial instance obtained from serial_new().
 *
 * @param[in] serial_instance Pointer to a serial. Must not be NULL.
 */
LIBSERIAL_API void serial_free(void *serial_instance);

/*! See Serial::open();
 *
 * Opens the serial port as long as the port is set and the port isn't
 * already open.
 *
 * If the port is provided to the constructor then an explicit call to open
 * is not needed.
 *
 * \return error code, when the port is already open, the code is 0.
 *         if return -1, invalid_argument.
 *         if return -2, serial_exception.
 *         if return -3, io_exception.
 */
LIBSERIAL_API int serial_open(void *serial_instance);

/*! See Serial::isOpen();
 * Gets the open status of the serial port.
 *
 * \return Returns true if the port is open, false otherwise.
 */
LIBSERIAL_API bool serial_is_open(void *serial_instance);

/*! See Serial::close();
 *
 * Closes the serial port.
 */
LIBSERIAL_API void serial_close(void *serial_instance);

/*! See Serial::available();
 * Return the number of characters in the buffer.
 */
LIBSERIAL_API size_t serial_available(void *serial_instance);

/*! See Serial::waitReadable();
 *
 * Block until there is serial data to read or read_timeout_constant
 * number of milliseconds have elapsed. The return value is true when
 * the function exits with the port in a readable state, false otherwise
 * (due to timeout or select interruption).
 */
LIBSERIAL_API bool serial_wait_readable(void *serial_instance);

/*! See Serial::waitByteTimes();
 *
 * Block for a period of time corresponding to the transmission time of
 * count characters at present serial settings. This may be used in con-
 * junction with waitReadable to read larger blocks of data from the
 * port.
 */
LIBSERIAL_API void serial_wait_byte_times(void *serial_instance, size_t count);


/*! See Serial::read();
 * Read a given amount of bytes from the serial port into a given buffer.
 *
 * The read function will return in one of three cases:
 *  * The number of requested bytes was read.
 *    * In this case the number of bytes requested will match the size_t
 *      returned by read.
 *  * A timeout occurred, in this case the number of bytes read will not
 *    match the amount requested, but no exception will be thrown.  One of
 *    two possible timeouts occurred:
 *    * The inter byte timeout expired, this means that number of
 *      milliseconds elapsed between receiving bytes from the serial port
 *      exceeded the inter byte timeout.
 *    * The total timeout expired, which is calculated by multiplying the
 *      read timeout multiplier by the number of requested bytes and then
 *      added to the read timeout constant.  If that total number of
 *      milliseconds elapses after the initial call to read a timeout will
 *      occur.
 *
 * \param buffer An uint8_t array of at least the requested size.
 * \param size A size_t defining how many bytes to be read.
 *
 * \return A int representing the number of bytes read as a result of the
 *         call to read.
 *         if return -1, port not opened.
 *         if return -2, serial_exception.
 */
LIBSERIAL_API int serial_read(void *serial_instance, void *buffer, size_t size);

/*! See Serial::write();
 *
 * Write the data to the serial port.
 *
 * \param data A const reference containing the data to be written
 * to the serial port.
 *
 * \param size A size_t that indicates how many bytes should be written from
 * the given data buffer.
 *
 * \return A int representing the number of bytes actually written to
 *         the serial port.
 *         if return -1, port not opened.
 *         if return -2, serial_exception.
 *         if return -3, io_exception.
 */
LIBSERIAL_API int serial_write(void *serial_instance, const void *data, size_t size);

/*! See Serial::setTimeout();
 *
 * Sets the timeout for reads and writes using the Timeout struct.
 *
 * There are two timeout conditions described here:
 *  * The inter byte timeout:
 *    * The inter_byte_timeout component of serial_timeout defines the
 *      maximum amount of time, in milliseconds, between receiving bytes on
 *      the serial port that can pass before a timeout occurs.  Setting this
 *      to zero will prevent inter byte timeouts from occurring.
 *  * Total time timeout:
 *    * The constant and multiplier component of this timeout condition,
 *      for both read and write, are defined in serial_timeout.  This
 *      timeout occurs if the total time since the read or write call was
 *      made exceeds the specified time in milliseconds.
 *    * The limit is defined by multiplying the multiplier component by the
 *      number of requested bytes and adding that product to the constant
 *      component.  In this way if you want a read call, for example, to
 *      timeout after exactly one second regardless of the number of bytes
 *      you asked for then set the read_timeout_constant component of
 *      serial_timeout to 1000 and the read_timeout_multiplier to zero.
 *      This timeout condition can be used in conjunction with the inter
 *      byte timeout condition with out any problems, timeout will simply
 *      occur when one of the two timeout conditions is met.  This allows
 *      users to have maximum control over the trade-off between
 *      responsiveness and efficiency.
 *
 * Read and write functions will return in one of three cases.  When the
 * reading or writing is complete, when a timeout occurs, or when an
 * exception occurs.
 *
 * A timeout of 0 enables non-blocking mode.
 *
 * \param timeout A serial_timeout struct containing the inter byte
 * timeout, and the read and write timeout constants and multipliers.
 */
LIBSERIAL_API void serial_set_timeout(void *serial_instance, struct serial_timeout timeout);

/*! See Serial::getTimeout();
 *
 * Gets the timeout for reads in seconds.
 *
 * \return A Timeout struct containing the inter_byte_timeout, and read
 * and write timeout constants and multipliers.
 */
LIBSERIAL_API struct serial_timeout serial_get_timeout(void *serial_instance);

/*! See Serial::setBaudrate();
 *
 * Sets the baudrate for the serial port.
 *
 * Possible baudrates depends on the system but some safe baudrates include:
 * 110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000,
 * 57600, 115200
 * Some other baudrates that are supported by some comports:
 * 128000, 153600, 230400, 256000, 460800, 500000, 921600
 *
 * \param baudrate An integer that sets the baud rate for the serial port.
 */
LIBSERIAL_API void serial_set_baudrate(void *serial_instance, unsigned int baudrate);

/*! See Serial::getBaudrate();
 *
 * Gets the baudrate for the serial port.
 *
 * \return An integer that sets the baud rate for the serial port.
 */
LIBSERIAL_API unsigned int serial_get_baudrate(void *serial_instance);

/*! See Serial::setBits();
 *
 * Sets the bytesize for the serial port.
 *
 * \param bytesize Size of each byte in the serial transmission of data,
 * default is eightbits, possible values are: fivebits, sixbits, sevenbits,
 * eightbits
 */
LIBSERIAL_API void serial_set_bits(void *serial_instance, unsigned int bits);

/*! See Serial::getBits();
 * Gets the bytesize for the serial port.
 */
LIBSERIAL_API unsigned int serial_get_bits(void *serial_instance);

/*! See Serial::setParity();
 *
 * Sets the parity for the serial port.
 *
 * \param parity Method of parity, default is parity_none, possible values
 * are: parity_none, parity_odd, parity_even
 */
LIBSERIAL_API void serial_set_parity(void *serial_instance, enum serial_parity parity);

/*! See Serial::getParity();
 *
 * Gets the parity for the serial port.
 */
LIBSERIAL_API enum serial_parity serial_get_parity(void *serial_instance);

/*! See Serial::setStopbits();
 *
 * Sets the stopbits for the serial port.
 *
 * \param stopbits Number of stop bits used, default is SERIAL_STOPBITS_ONE.
 */
LIBSERIAL_API void serial_set_stopbits(void *serial_instance, enum serial_stopbits stopbits);

/*! See Serial::getStopbits();
 * Gets the stopbits for the serial port.
 */
LIBSERIAL_API enum serial_stopbits serial_get_stopbits(void *serial_instance);

/*! See Serial::setFlowcontrol();
 *
 * Sets the flow control for the serial port.
 *
 * \param flowcontrol Type of flowcontrol used, default is flowcontrol_none,
 * possible values are: NONE, XONXOFF, RTSCTS
 */
LIBSERIAL_API void serial_set_flowcontrol(void *serial_instance, enum serial_flowcontrol flowcontrol);

/*! See Serial::getFlowcontrol();
 *
 * Gets the flow control for the serial port.
 */
LIBSERIAL_API enum serial_flowcontrol serial_get_flowcontrol(void *serial_instance);

/*! See Serial::flush();
 *
 * Flush the input and output buffers
 */
LIBSERIAL_API void serial_flush(void *serial_instance);

/*! See Serial::flushInput();
 *
 * Flush only the input buffer
 */
LIBSERIAL_API void serial_flush_input(void *serial_instance);

/*! See Serial::flushOutput();
 *
 * Flush only the output buffer
 */
LIBSERIAL_API void serial_flush_output(void *serial_instance);

/*! See Serial::sendBreak();
 *
 * Sends the RS-232 break signal.  See tcsendbreak(3).
 * */
LIBSERIAL_API void serial_send_break(void *serial_instance, int duration);

/*! See Serial::setBreak();
 *
 * Set the break condition to a given level.
 */
LIBSERIAL_API void serial_set_break(void *serial_instance, bool level);

/*! See Serial::setRTS();
 *
 * Set the RTS handshaking line to the given level.
 */
LIBSERIAL_API void serial_set_rts(void *serial_instance, bool level);

/*! See Serial::setDTR();
 *
 * Set the DTR handshaking line to the given level.
 */
LIBSERIAL_API void serial_dtr(void *serial_instance, bool level);

/*! See Serial::waitForChange();
 *
 * Blocks until CTS, DSR, RI, CD changes or something interrupts it.
 *
 * Can throw an exception if an error occurs while waiting.
 * You can check the status of CTS, DSR, RI, and CD once this returns.
 * Uses TIOCMIWAIT via ioctl if available (mostly only on Linux) with a
 * resolution of less than +-1ms and as good as +-0.2ms.  Otherwise a
 * polling method is used which can give +-2ms.
 *
 * \return Returns 1 if one of the lines changed, 0 if something else
 * occurred, -1 if an error occurred.
 */
LIBSERIAL_API int serial_wait_for_change(void *serial_instance);

/*! See Serial::getCTS();
 *
 * Returns the current status of the CTS line.
 */
LIBSERIAL_API bool serial_get_cts(void *serial_instance);

/*! See Serial::getDSR();
 *
 * Returns the current status of the DSR line.
 */
LIBSERIAL_API bool serial_get_dsr(void *serial_instance);

/*! See Serial::getRI();
 *
 * Returns the current status of the RI line.
 */
LIBSERIAL_API bool serial_get_ri(void *serial_instance);

/*! See Serial::getCD();
 *
 * Returns the current status of the CD line.
 */
LIBSERIAL_API bool serial_get_cd(void *serial_instance);

#ifdef __cplusplus
}
#endif

#endif //SERIAL_SERIAL_C_H
