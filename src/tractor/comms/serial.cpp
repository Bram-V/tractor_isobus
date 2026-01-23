#include "tractor/comms/serial.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

namespace tractor {
    namespace comms {

        struct Serial::Impl {
            SerialOptions options;
            std::unique_ptr<Tty> tty;

            std::atomic<bool> running{false};
            std::atomic<bool> connected{false};
            std::thread reader_thread;

            mutable std::mutex write_mutex;
            mutable std::mutex callback_mutex;
            mutable std::mutex stats_mutex;

            LineCallback line_callback;
            DataCallback data_callback;
            ConnectionCallback connection_callback;
            ErrorCallback error_callback;

            Statistics stats;

            std::string read_buffer;

            ~Impl() {
                if (reader_thread.joinable()) {
                    reader_thread.join();
                }
            }
        };

        Serial::Serial(const SerialOptions &options) : pimpl_(std::make_unique<Impl>()) { pimpl_->options = options; }

        Serial::Serial(const std::string &port, uint32_t baud_rate) : pimpl_(std::make_unique<Impl>()) {
            pimpl_->options.port = port;
            pimpl_->options.tty_config.baud_rate = baud_rate;
            pimpl_->options.tty_config.read_timeout_ms = 100;
            pimpl_->options.framing = FramingMode::LineDelimited;
            pimpl_->options.line_delimiter = '\n';
            pimpl_->options.strip_line_endings = true;
        }

        Serial::~Serial() { stop(); }

        bool Serial::start() {
            if (pimpl_->running) {
                return true;
            }

            if (!connect()) {
                return false;
            }

            pimpl_->running = true;
            pimpl_->reader_thread = std::thread(&Serial::reader_thread, this);

            return true;
        }

        void Serial::stop() {
            if (!pimpl_->running) {
                return;
            }

            pimpl_->running = false;

            if (pimpl_->reader_thread.joinable()) {
                pimpl_->reader_thread.join();
            }

            disconnect();
        }

        bool Serial::is_running() const { return pimpl_->running; }

        bool Serial::is_connected() const { return pimpl_->connected; }

        bool Serial::write_line(const std::string &line) {
            std::string data = line;
            if (pimpl_->options.framing == FramingMode::LineDelimited) {
                data += pimpl_->options.line_delimiter;
            }
            return write(data);
        }

        bool Serial::write(const std::vector<uint8_t> &data) {
            std::lock_guard<std::mutex> lock(pimpl_->write_mutex);

            if (!pimpl_->connected || !pimpl_->tty) {
                return false;
            }

            ssize_t written = pimpl_->tty->write(data);
            if (written < 0) {
                std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                if (pimpl_->error_callback) {
                    pimpl_->error_callback("Write error: " + pimpl_->tty->get_last_error());
                }
                return false;
            }

            {
                std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                pimpl_->stats.bytes_sent += written;
            }

            return written == static_cast<ssize_t>(data.size());
        }

        bool Serial::write(const std::string &data) {
            std::vector<uint8_t> bytes(data.begin(), data.end());
            return write(bytes);
        }

        void Serial::on_line(LineCallback callback) {
            std::lock_guard<std::mutex> lock(pimpl_->callback_mutex);
            pimpl_->line_callback = callback;
        }

        void Serial::on_data(DataCallback callback) {
            std::lock_guard<std::mutex> lock(pimpl_->callback_mutex);
            pimpl_->data_callback = callback;
        }

        void Serial::on_connection(ConnectionCallback callback) {
            std::lock_guard<std::mutex> lock(pimpl_->callback_mutex);
            pimpl_->connection_callback = callback;
        }

        void Serial::on_error(ErrorCallback callback) {
            std::lock_guard<std::mutex> lock(pimpl_->callback_mutex);
            pimpl_->error_callback = callback;
        }

        Serial::Statistics Serial::get_statistics() const {
            std::lock_guard<std::mutex> lock(pimpl_->stats_mutex);
            return pimpl_->stats;
        }

        void Serial::reset_statistics() {
            std::lock_guard<std::mutex> lock(pimpl_->stats_mutex);
            pimpl_->stats = Statistics{};
        }

        SerialOptions Serial::get_options() const { return pimpl_->options; }

        Tty *Serial::get_tty() { return pimpl_->tty.get(); }

        bool Serial::connect() {
            pimpl_->tty = std::make_unique<Tty>(pimpl_->options.port, pimpl_->options.tty_config);

            if (!pimpl_->tty->open()) {
                std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                if (pimpl_->error_callback) {
                    pimpl_->error_callback("Failed to open port: " + pimpl_->tty->get_last_error());
                }
                pimpl_->tty.reset();
                return false;
            }

            pimpl_->connected = true;
            pimpl_->read_buffer.clear();

            {
                std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                if (pimpl_->connection_callback) {
                    pimpl_->connection_callback(true);
                }
            }

            return true;
        }

        void Serial::disconnect() {
            if (!pimpl_->connected) {
                return;
            }

            pimpl_->connected = false;

            if (pimpl_->tty) {
                pimpl_->tty->close();
                pimpl_->tty.reset();
            }

            {
                std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                if (pimpl_->connection_callback) {
                    pimpl_->connection_callback(false);
                }
            }
        }

        void Serial::reader_thread() {
            while (pimpl_->running) {
                if (!pimpl_->connected) {
                    if (pimpl_->options.auto_reconnect) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(pimpl_->options.reconnect_delay_ms));

                        if (connect()) {
                            std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                            pimpl_->stats.reconnects++;
                        }
                    } else {
                        break;
                    }
                    continue;
                }

                // Process data based on framing mode
                switch (pimpl_->options.framing) {
                case FramingMode::LineDelimited:
                    process_line_delimited();
                    break;
                case FramingMode::FixedLength:
                    process_fixed_length();
                    break;
                case FramingMode::LengthPrefixed:
                    process_length_prefixed();
                    break;
                case FramingMode::Custom:
                    process_custom();
                    break;
                }
            }
        }

        void Serial::process_line_delimited() {
            uint8_t ch;
            ssize_t bytes_read = pimpl_->tty->read(&ch, 1);

            if (bytes_read < 0) {
                // Error
                std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                pimpl_->stats.errors++;
                disconnect();
                return;
            }

            if (bytes_read == 0) {
                // Timeout, continue
                return;
            }

            {
                std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                pimpl_->stats.bytes_received++;
            }

            // Check for line delimiter
            if (ch == pimpl_->options.line_delimiter) {
                if (!pimpl_->read_buffer.empty()) {
                    std::string line = pimpl_->read_buffer;

                    // Strip trailing \r if present
                    if (pimpl_->options.strip_line_endings && !line.empty() && line.back() == '\r') {
                        line.pop_back();
                    }

                    {
                        std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                        pimpl_->stats.lines_received++;
                    }

                    // Call callback
                    {
                        std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                        if (pimpl_->line_callback) {
                            pimpl_->line_callback(line);
                        }
                    }

                    pimpl_->read_buffer.clear();
                }
            } else if (ch == '\r' && pimpl_->options.strip_line_endings) {
                // Skip \r if we're stripping line endings (will be handled above)
                pimpl_->read_buffer += static_cast<char>(ch);
            } else {
                // Append to buffer
                pimpl_->read_buffer += static_cast<char>(ch);

                // Check for overflow
                if (pimpl_->read_buffer.size() > pimpl_->options.max_line_length) {
                    std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                    if (pimpl_->error_callback) {
                        pimpl_->error_callback("Line exceeds maximum length");
                    }
                    pimpl_->read_buffer.clear();

                    std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                    pimpl_->stats.errors++;
                }
            }
        }

        void Serial::process_fixed_length() {
            if (pimpl_->options.fixed_length == 0) {
                return;
            }

            std::vector<uint8_t> buffer(pimpl_->options.fixed_length);
            ssize_t bytes_read = pimpl_->tty->read_exact(buffer.data(), buffer.size());

            if (bytes_read < 0) {
                std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                pimpl_->stats.errors++;
                disconnect();
                return;
            }

            if (bytes_read == static_cast<ssize_t>(buffer.size())) {
                {
                    std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                    pimpl_->stats.bytes_received += bytes_read;
                }

                {
                    std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                    if (pimpl_->data_callback) {
                        pimpl_->data_callback(buffer);
                    }
                }
            }
        }

        void Serial::process_length_prefixed() {
            // Read length prefix (1 byte)
            uint8_t length;
            ssize_t bytes_read = pimpl_->tty->read(&length, 1);

            if (bytes_read <= 0) {
                if (bytes_read < 0) {
                    std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                    pimpl_->stats.errors++;
                    disconnect();
                }
                return;
            }

            if (length == 0) {
                return;
            }

            // Read data
            std::vector<uint8_t> buffer(length);
            bytes_read = pimpl_->tty->read_exact(buffer.data(), length);

            if (bytes_read == length) {
                {
                    std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                    pimpl_->stats.bytes_received += bytes_read + 1; // +1 for length byte
                }

                {
                    std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                    if (pimpl_->data_callback) {
                        pimpl_->data_callback(buffer);
                    }
                }
            } else {
                std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                pimpl_->stats.errors++;
            }
        }

        void Serial::process_custom() {
            uint8_t ch;
            ssize_t bytes_read = pimpl_->tty->read(&ch, 1);

            if (bytes_read < 0) {
                std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                pimpl_->stats.errors++;
                disconnect();
                return;
            }

            if (bytes_read == 0) {
                return;
            }

            {
                std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                pimpl_->stats.bytes_received++;
            }

            // Check for custom delimiter
            if (ch == pimpl_->options.custom_delimiter) {
                if (!pimpl_->read_buffer.empty()) {
                    std::vector<uint8_t> data(pimpl_->read_buffer.begin(), pimpl_->read_buffer.end());

                    {
                        std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                        if (pimpl_->data_callback) {
                            pimpl_->data_callback(data);
                        }
                    }

                    pimpl_->read_buffer.clear();
                }
            } else {
                pimpl_->read_buffer += static_cast<char>(ch);

                if (pimpl_->read_buffer.size() > pimpl_->options.max_line_length) {
                    std::lock_guard<std::mutex> cb_lock(pimpl_->callback_mutex);
                    if (pimpl_->error_callback) {
                        pimpl_->error_callback("Message exceeds maximum length");
                    }
                    pimpl_->read_buffer.clear();

                    std::lock_guard<std::mutex> stats_lock(pimpl_->stats_mutex);
                    pimpl_->stats.errors++;
                }
            }
        }

    } // namespace comms
} // namespace tractor
