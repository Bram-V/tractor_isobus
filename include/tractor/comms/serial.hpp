#pragma once

#include "tractor/comms/tty.hpp"
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace tractor {
    namespace comms {

        /**
         * @brief Callback function for received lines
         * @param line The received line (without delimiter)
         */
        using LineCallback = std::function<void(const std::string &line)>;

        /**
         * @brief Callback function for received binary data
         * @param data The received data
         */
        using DataCallback = std::function<void(const std::vector<uint8_t> &data)>;

        /**
         * @brief Callback function for connection events
         * @param connected true if connected, false if disconnected
         */
        using ConnectionCallback = std::function<void(bool connected)>;

        /**
         * @brief Callback function for errors
         * @param error_message Description of the error
         */
        using ErrorCallback = std::function<void(const std::string &error_message)>;

        /**
         * @brief Message framing mode for serial communication
         */
        enum class FramingMode {
            LineDelimited,  // Messages separated by line delimiter (\n, \r\n, etc.)
            FixedLength,    // Fixed-length messages
            LengthPrefixed, // Length prefix followed by data
            Custom          // Custom delimiter byte
        };

        /**
         * @brief Configuration for high-level serial communication
         */
        struct SerialOptions {
            std::string port;
            SerialConfig tty_config;

            FramingMode framing = FramingMode::LineDelimited;
            char line_delimiter = '\n';   // For LineDelimited mode
            size_t fixed_length = 0;      // For FixedLength mode
            uint8_t custom_delimiter = 0; // For Custom mode

            bool auto_reconnect = false;
            uint32_t reconnect_delay_ms = 1000;
            size_t max_line_length = 4096;
            bool strip_line_endings = true; // Remove \r\n from lines
        };

        /**
         * @brief High-level serial port communication class
         *
         * Provides an event-driven, thread-safe interface for serial communication with:
         * - Automatic message framing (line-based, length-prefixed, etc.)
         * - Callback-based event handling
         * - Thread-safe write operations
         * - Optional automatic reconnection
         * - Background reading thread
         */
        class Serial {
          public:
            /**
             * @brief Constructor
             * @param options Serial communication options
             */
            explicit Serial(const SerialOptions &options);

            /**
             * @brief Simplified constructor for line-based communication
             * @param port Serial port path
             * @param baud_rate Baud rate
             */
            Serial(const std::string &port, uint32_t baud_rate = 115200);

            /**
             * @brief Destructor
             */
            ~Serial();

            // Delete copy
            Serial(const Serial &) = delete;
            Serial &operator=(const Serial &) = delete;

            /**
             * @brief Start serial communication
             * @return true if successfully started, false otherwise
             */
            bool start();

            /**
             * @brief Stop serial communication
             */
            void stop();

            /**
             * @brief Check if serial communication is active
             * @return true if running, false otherwise
             */
            bool is_running() const;

            /**
             * @brief Check if currently connected to the port
             * @return true if connected, false otherwise
             */
            bool is_connected() const;

            /**
             * @brief Write a line (automatically appends line ending)
             * @param line Line to write
             * @return true if successful, false otherwise
             */
            bool write_line(const std::string &line);

            /**
             * @brief Write raw data
             * @param data Data to write
             * @return true if successful, false otherwise
             */
            bool write(const std::vector<uint8_t> &data);

            /**
             * @brief Write string data
             * @param data String to write
             * @return true if successful, false otherwise
             */
            bool write(const std::string &data);

            /**
             * @brief Set callback for received lines (LineDelimited mode)
             * @param callback Function to call when a line is received
             */
            void on_line(LineCallback callback);

            /**
             * @brief Set callback for received data (other framing modes)
             * @param callback Function to call when data is received
             */
            void on_data(DataCallback callback);

            /**
             * @brief Set callback for connection state changes
             * @param callback Function to call on connect/disconnect
             */
            void on_connection(ConnectionCallback callback);

            /**
             * @brief Set callback for errors
             * @param callback Function to call on errors
             */
            void on_error(ErrorCallback callback);

            /**
             * @brief Get statistics
             */
            struct Statistics {
                size_t lines_received = 0;
                size_t bytes_received = 0;
                size_t bytes_sent = 0;
                size_t errors = 0;
                size_t reconnects = 0;
            };

            Statistics get_statistics() const;

            /**
             * @brief Reset statistics counters
             */
            void reset_statistics();

            /**
             * @brief Get current options
             * @return Current serial options
             */
            SerialOptions get_options() const;

            /**
             * @brief Get the underlying TTY device (for advanced usage)
             * @return Pointer to TTY device, or nullptr if not connected
             */
            Tty *get_tty();

          private:
            struct Impl;
            std::unique_ptr<Impl> pimpl_;

            void reader_thread();
            bool connect();
            void disconnect();
            void process_line_delimited();
            void process_fixed_length();
            void process_length_prefixed();
            void process_custom();
        };

    } // namespace comms
} // namespace tractor
