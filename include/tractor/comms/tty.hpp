#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace tractor {
    namespace comms {

        /**
         * @brief Parity configuration for serial communication
         */
        enum class Parity { None, Odd, Even, Mark, Space };

        /**
         * @brief Stop bits configuration
         */
        enum class StopBits { One, OnePointFive, Two };

        /**
         * @brief Data bits configuration
         */
        enum class DataBits { Five = 5, Six = 6, Seven = 7, Eight = 8 };

        /**
         * @brief Flow control configuration
         */
        enum class FlowControl { None, Hardware, Software };

        /**
         * @brief Serial port configuration structure
         */
        struct SerialConfig {
            uint32_t baud_rate = 9600;
            DataBits data_bits = DataBits::Eight;
            Parity parity = Parity::None;
            StopBits stop_bits = StopBits::One;
            FlowControl flow_control = FlowControl::None;
            uint32_t read_timeout_ms = 1000;
            uint32_t write_timeout_ms = 1000;
            bool vmin_mode = false; // If true, uses VMIN/VTIME instead of timeout
            uint8_t vmin = 1;       // Minimum characters to read
            uint8_t vtime = 0;      // Time between characters (in deciseconds)
        };

        /**
         * @brief Comprehensive serial port abstraction class
         *
         * Supports communication over ttyUSB, ttyS, ttyACM, and other serial devices.
         * Provides both blocking and non-blocking I/O operations with configurable timeouts.
         */
        class Tty {
          public:
            /**
             * @brief Default constructor
             */
            Tty();

            /**
             * @brief Constructor with port and baud rate
             * @param port Serial port device path (e.g., "/dev/ttyUSB0")
             * @param baud_rate Communication speed in bits per second
             */
            Tty(const std::string &port, uint32_t baud_rate = 9600);

            /**
             * @brief Constructor with full configuration
             * @param port Serial port device path
             * @param config Serial port configuration
             */
            Tty(const std::string &port, const SerialConfig &config);

            /**
             * @brief Destructor - automatically closes the port
             */
            ~Tty();

            // Delete copy constructor and assignment
            Tty(const Tty &) = delete;
            Tty &operator=(const Tty &) = delete;

            // Allow move semantics
            Tty(Tty &&other) noexcept;
            Tty &operator=(Tty &&other) noexcept;

            /**
             * @brief Open the serial port
             * @return true if successful, false otherwise
             */
            bool open();

            /**
             * @brief Open a specific serial port
             * @param port Serial port device path
             * @return true if successful, false otherwise
             */
            bool open(const std::string &port);

            /**
             * @brief Open with specific configuration
             * @param port Serial port device path
             * @param config Serial port configuration
             * @return true if successful, false otherwise
             */
            bool open(const std::string &port, const SerialConfig &config);

            /**
             * @brief Close the serial port
             */
            void close();

            /**
             * @brief Check if the port is currently open
             * @return true if open, false otherwise
             */
            bool is_open() const;

            /**
             * @brief Write data to the serial port
             * @param data Data buffer to write
             * @param size Number of bytes to write
             * @return Number of bytes actually written, -1 on error
             */
            ssize_t write(const uint8_t *data, size_t size);

            /**
             * @brief Write data to the serial port
             * @param data Vector of bytes to write
             * @return Number of bytes actually written, -1 on error
             */
            ssize_t write(const std::vector<uint8_t> &data);

            /**
             * @brief Write string to the serial port
             * @param data String to write
             * @return Number of bytes actually written, -1 on error
             */
            ssize_t write(const std::string &data);

            /**
             * @brief Read data from the serial port
             * @param buffer Buffer to store read data
             * @param size Maximum number of bytes to read
             * @return Number of bytes actually read, -1 on error, 0 on timeout
             */
            ssize_t read(uint8_t *buffer, size_t size);

            /**
             * @brief Read data into a vector
             * @param buffer Vector to store read data
             * @param size Maximum number of bytes to read
             * @return Number of bytes actually read, -1 on error, 0 on timeout
             */
            ssize_t read(std::vector<uint8_t> &buffer, size_t size);

            /**
             * @brief Read until a delimiter is found
             * @param delimiter Byte to use as delimiter
             * @param max_size Maximum bytes to read
             * @return Vector containing read data including delimiter, empty on error/timeout
             */
            std::vector<uint8_t> read_until(uint8_t delimiter, size_t max_size = 1024);

            /**
             * @brief Read a line (until '\n')
             * @param max_size Maximum bytes to read
             * @return String containing the line (without '\n'), empty on error/timeout
             */
            std::string read_line(size_t max_size = 1024);

            /**
             * @brief Read exactly the specified number of bytes
             * @param buffer Buffer to store read data
             * @param size Exact number of bytes to read
             * @param timeout_ms Timeout in milliseconds (0 = use default)
             * @return Number of bytes read (equals size on success), less on timeout/error
             */
            ssize_t read_exact(uint8_t *buffer, size_t size, uint32_t timeout_ms = 0);

            /**
             * @brief Read exactly the specified number of bytes into a vector
             * @param size Exact number of bytes to read
             * @param timeout_ms Timeout in milliseconds (0 = use default)
             * @return Vector containing read data (size equals requested on success)
             */
            std::vector<uint8_t> read_exact(size_t size, uint32_t timeout_ms = 0);

            /**
             * @brief Get the number of bytes available to read
             * @return Number of bytes available, -1 on error
             */
            ssize_t available() const;

            /**
             * @brief Flush input buffer (discard all received data)
             */
            void flush_input();

            /**
             * @brief Flush output buffer (wait for all data to be transmitted)
             */
            void flush_output();

            /**
             * @brief Flush both input and output buffers
             */
            void flush();

            /**
             * @brief Set the baud rate
             * @param baud_rate Baud rate in bits per second
             * @return true if successful, false otherwise
             */
            bool set_baud_rate(uint32_t baud_rate);

            /**
             * @brief Get current baud rate
             * @return Current baud rate
             */
            uint32_t get_baud_rate() const;

            /**
             * @brief Set read timeout
             * @param timeout_ms Timeout in milliseconds
             */
            void set_read_timeout(uint32_t timeout_ms);

            /**
             * @brief Set write timeout
             * @param timeout_ms Timeout in milliseconds
             */
            void set_write_timeout(uint32_t timeout_ms);

            /**
             * @brief Set data bits
             * @param data_bits Number of data bits (5-8)
             * @return true if successful, false otherwise
             */
            bool set_data_bits(DataBits data_bits);

            /**
             * @brief Set parity
             * @param parity Parity mode
             * @return true if successful, false otherwise
             */
            bool set_parity(Parity parity);

            /**
             * @brief Set stop bits
             * @param stop_bits Number of stop bits
             * @return true if successful, false otherwise
             */
            bool set_stop_bits(StopBits stop_bits);

            /**
             * @brief Set flow control
             * @param flow_control Flow control mode
             * @return true if successful, false otherwise
             */
            bool set_flow_control(FlowControl flow_control);

            /**
             * @brief Apply a complete configuration
             * @param config Configuration to apply
             * @return true if successful, false otherwise
             */
            bool apply_config(const SerialConfig &config);

            /**
             * @brief Get current configuration
             * @return Current serial configuration
             */
            SerialConfig get_config() const;

            /**
             * @brief Get the port device path
             * @return Port path string
             */
            std::string get_port() const;

            /**
             * @brief Get the last error message
             * @return Error message string
             */
            std::string get_last_error() const;

            /**
             * @brief List available serial ports
             * @return Vector of available port paths
             */
            static std::vector<std::string> list_ports();

            /**
             * @brief Check if a port exists
             * @param port Port path to check
             * @return true if port exists, false otherwise
             */
            static bool port_exists(const std::string &port);

            /**
             * @brief Set DTR (Data Terminal Ready) line
             * @param state true to set high, false to set low
             * @return true if successful, false otherwise
             */
            bool set_dtr(bool state);

            /**
             * @brief Set RTS (Request To Send) line
             * @param state true to set high, false to set low
             * @return true if successful, false otherwise
             */
            bool set_rts(bool state);

            /**
             * @brief Get CTS (Clear To Send) line state
             * @return true if high, false if low or on error
             */
            bool get_cts() const;

            /**
             * @brief Get DSR (Data Set Ready) line state
             * @return true if high, false if low or on error
             */
            bool get_dsr() const;

            /**
             * @brief Get RI (Ring Indicator) line state
             * @return true if high, false if low or on error
             */
            bool get_ri() const;

            /**
             * @brief Get CD (Carrier Detect) line state
             * @return true if high, false if low or on error
             */
            bool get_cd() const;

            /**
             * @brief Send a break signal
             * @param duration_ms Duration in milliseconds (0 = default ~250ms)
             */
            void send_break(uint32_t duration_ms = 0);

            /**
             * @brief Get native file descriptor (for advanced usage)
             * @return File descriptor or -1 if not open
             */
            int get_fd() const;

          private:
            class Impl;
            std::unique_ptr<Impl> pimpl_;

            bool configure_port();
            bool set_termios_baud(uint32_t baud_rate);
        };

    } // namespace comms
} // namespace tractor
