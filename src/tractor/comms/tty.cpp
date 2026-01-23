#include "tractor/comms/tty.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <system_error>
#include <termios.h>
#include <unistd.h>

namespace tractor {
    namespace comms {

        // PIMPL implementation class
        class Tty::Impl {
          public:
            int fd = -1;
            std::string port;
            SerialConfig config;
            std::string last_error;
            struct termios original_termios;
            bool has_original_termios = false;

            ~Impl() {
                if (fd >= 0) {
                    if (has_original_termios) {
                        tcsetattr(fd, TCSANOW, &original_termios);
                    }
                    ::close(fd);
                }
            }
        };

        // Helper function to convert baud rate to termios constant
        static speed_t baud_rate_to_speed(uint32_t baud_rate) {
            switch (baud_rate) {
            case 50:
                return B50;
            case 75:
                return B75;
            case 110:
                return B110;
            case 134:
                return B134;
            case 150:
                return B150;
            case 200:
                return B200;
            case 300:
                return B300;
            case 600:
                return B600;
            case 1200:
                return B1200;
            case 1800:
                return B1800;
            case 2400:
                return B2400;
            case 4800:
                return B4800;
            case 9600:
                return B9600;
            case 19200:
                return B19200;
            case 38400:
                return B38400;
            case 57600:
                return B57600;
            case 115200:
                return B115200;
            case 230400:
                return B230400;
            case 460800:
                return B460800;
            case 500000:
                return B500000;
            case 576000:
                return B576000;
            case 921600:
                return B921600;
            case 1000000:
                return B1000000;
            case 1152000:
                return B1152000;
            case 1500000:
                return B1500000;
            case 2000000:
                return B2000000;
            case 2500000:
                return B2500000;
            case 3000000:
                return B3000000;
            case 3500000:
                return B3500000;
            case 4000000:
                return B4000000;
            default:
                return B0;
            }
        }

        static uint32_t speed_to_baud_rate(speed_t speed) {
            switch (speed) {
            case B50:
                return 50;
            case B75:
                return 75;
            case B110:
                return 110;
            case B134:
                return 134;
            case B150:
                return 150;
            case B200:
                return 200;
            case B300:
                return 300;
            case B600:
                return 600;
            case B1200:
                return 1200;
            case B1800:
                return 1800;
            case B2400:
                return 2400;
            case B4800:
                return 4800;
            case B9600:
                return 9600;
            case B19200:
                return 19200;
            case B38400:
                return 38400;
            case B57600:
                return 57600;
            case B115200:
                return 115200;
            case B230400:
                return 230400;
            case B460800:
                return 460800;
            case B500000:
                return 500000;
            case B576000:
                return 576000;
            case B921600:
                return 921600;
            case B1000000:
                return 1000000;
            case B1152000:
                return 1152000;
            case B1500000:
                return 1500000;
            case B2000000:
                return 2000000;
            case B2500000:
                return 2500000;
            case B3000000:
                return 3000000;
            case B3500000:
                return 3500000;
            case B4000000:
                return 4000000;
            default:
                return 0;
            }
        }

        // Constructors and destructor
        Tty::Tty() : pimpl_(std::make_unique<Impl>()) {}

        Tty::Tty(const std::string &port, uint32_t baud_rate) : pimpl_(std::make_unique<Impl>()) {
            pimpl_->port = port;
            pimpl_->config.baud_rate = baud_rate;
        }

        Tty::Tty(const std::string &port, const SerialConfig &config) : pimpl_(std::make_unique<Impl>()) {
            pimpl_->port = port;
            pimpl_->config = config;
        }

        Tty::~Tty() = default;

        Tty::Tty(Tty &&other) noexcept = default;
        Tty &Tty::operator=(Tty &&other) noexcept = default;

        // Open/Close operations
        bool Tty::open() {
            if (pimpl_->port.empty()) {
                pimpl_->last_error = "No port specified";
                return false;
            }
            return open(pimpl_->port, pimpl_->config);
        }

        bool Tty::open(const std::string &port) {
            pimpl_->port = port;
            return open(port, pimpl_->config);
        }

        bool Tty::open(const std::string &port, const SerialConfig &config) {
            if (pimpl_->fd >= 0) {
                close();
            }

            pimpl_->port = port;
            pimpl_->config = config;

            // Open the port with non-blocking flag initially
            pimpl_->fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

            if (pimpl_->fd < 0) {
                pimpl_->last_error = std::string("Failed to open port: ") + std::strerror(errno);
                return false;
            }

            // Check if it's a terminal device
            if (!isatty(pimpl_->fd)) {
                pimpl_->last_error = "Device is not a TTY";
                ::close(pimpl_->fd);
                pimpl_->fd = -1;
                return false;
            }

            // Get current settings to restore later if needed
            if (tcgetattr(pimpl_->fd, &pimpl_->original_termios) == 0) {
                pimpl_->has_original_termios = true;
            }

            // Configure the port
            if (!configure_port()) {
                ::close(pimpl_->fd);
                pimpl_->fd = -1;
                return false;
            }

            // Remove O_NONBLOCK flag for blocking I/O
            int flags = fcntl(pimpl_->fd, F_GETFL, 0);
            if (flags != -1) {
                fcntl(pimpl_->fd, F_SETFL, flags & ~O_NONBLOCK);
            }

            pimpl_->last_error.clear();
            return true;
        }

        void Tty::close() {
            if (pimpl_->fd >= 0) {
                // Restore original settings if available
                if (pimpl_->has_original_termios) {
                    tcsetattr(pimpl_->fd, TCSANOW, &pimpl_->original_termios);
                }
                ::close(pimpl_->fd);
                pimpl_->fd = -1;
            }
        }

        bool Tty::is_open() const { return pimpl_->fd >= 0; }

        // Configuration
        bool Tty::configure_port() {
            struct termios tty;
            std::memset(&tty, 0, sizeof(tty));

            if (tcgetattr(pimpl_->fd, &tty) != 0) {
                pimpl_->last_error = std::string("Failed to get attributes: ") + std::strerror(errno);
                return false;
            }

            // Set baud rate
            speed_t speed = baud_rate_to_speed(pimpl_->config.baud_rate);
            if (speed == B0) {
                pimpl_->last_error = "Unsupported baud rate";
                return false;
            }
            cfsetispeed(&tty, speed);
            cfsetospeed(&tty, speed);

            // Control modes
            tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines

            // Data bits
            tty.c_cflag &= ~CSIZE;
            switch (pimpl_->config.data_bits) {
            case DataBits::Five:
                tty.c_cflag |= CS5;
                break;
            case DataBits::Six:
                tty.c_cflag |= CS6;
                break;
            case DataBits::Seven:
                tty.c_cflag |= CS7;
                break;
            case DataBits::Eight:
                tty.c_cflag |= CS8;
                break;
            }

            // Parity
            switch (pimpl_->config.parity) {
            case Parity::None:
                tty.c_cflag &= ~PARENB;
                break;
            case Parity::Odd:
                tty.c_cflag |= PARENB;
                tty.c_cflag |= PARODD;
                break;
            case Parity::Even:
                tty.c_cflag |= PARENB;
                tty.c_cflag &= ~PARODD;
                break;
            case Parity::Mark:
            case Parity::Space:
                // Mark and Space parity require special handling on Linux
                tty.c_cflag |= PARENB;
                tty.c_cflag |= CMSPAR;
                if (pimpl_->config.parity == Parity::Mark) {
                    tty.c_cflag |= PARODD;
                } else {
                    tty.c_cflag &= ~PARODD;
                }
                break;
            }

            // Stop bits
            switch (pimpl_->config.stop_bits) {
            case StopBits::One:
                tty.c_cflag &= ~CSTOPB;
                break;
            case StopBits::Two:
                tty.c_cflag |= CSTOPB;
                break;
            case StopBits::OnePointFive:
                // Not directly supported, use two stop bits
                tty.c_cflag |= CSTOPB;
                break;
            }

            // Flow control
            switch (pimpl_->config.flow_control) {
            case FlowControl::None:
                tty.c_cflag &= ~CRTSCTS;
                tty.c_iflag &= ~(IXON | IXOFF | IXANY);
                break;
            case FlowControl::Hardware:
                tty.c_cflag |= CRTSCTS;
                tty.c_iflag &= ~(IXON | IXOFF | IXANY);
                break;
            case FlowControl::Software:
                tty.c_cflag &= ~CRTSCTS;
                tty.c_iflag |= (IXON | IXOFF);
                break;
            }

            // Input modes - disable software flow control, no special processing
            tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

            // Output modes - raw output
            tty.c_oflag &= ~OPOST;

            // Local modes - raw mode, no echo, no signals
            tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

            // Timeout configuration
            if (pimpl_->config.vmin_mode) {
                // VMIN/VTIME mode
                tty.c_cc[VMIN] = pimpl_->config.vmin;
                tty.c_cc[VTIME] = pimpl_->config.vtime;
            } else {
                // Timeout-based mode
                tty.c_cc[VMIN] = 0;
                tty.c_cc[VTIME] = (pimpl_->config.read_timeout_ms + 99) / 100; // Convert ms to deciseconds
            }

            // Apply settings
            if (tcsetattr(pimpl_->fd, TCSANOW, &tty) != 0) {
                pimpl_->last_error = std::string("Failed to set attributes: ") + std::strerror(errno);
                return false;
            }

            // Flush any existing data
            tcflush(pimpl_->fd, TCIOFLUSH);

            return true;
        }

        // Write operations
        ssize_t Tty::write(const uint8_t *data, size_t size) {
            if (data == nullptr || size == 0) {
                return 0;
            }

            if (!is_open()) {
                pimpl_->last_error = "Port is not open";
                return -1;
            }

            ssize_t total_written = 0;
            const uint8_t *ptr = data;
            size_t remaining = size;

            // Use select for write timeout
            struct timeval timeout;
            timeout.tv_sec = pimpl_->config.write_timeout_ms / 1000;
            timeout.tv_usec = (pimpl_->config.write_timeout_ms % 1000) * 1000;

            while (remaining > 0) {
                fd_set write_fds;
                FD_ZERO(&write_fds);
                FD_SET(pimpl_->fd, &write_fds);

                int ret = select(pimpl_->fd + 1, nullptr, &write_fds, nullptr, &timeout);
                if (ret < 0) {
                    pimpl_->last_error = std::string("Select error: ") + std::strerror(errno);
                    return -1;
                } else if (ret == 0) {
                    pimpl_->last_error = "Write timeout";
                    return total_written;
                }

                ssize_t written = ::write(pimpl_->fd, ptr, remaining);
                if (written < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        continue;
                    }
                    pimpl_->last_error = std::string("Write error: ") + std::strerror(errno);
                    return -1;
                }

                total_written += written;
                ptr += written;
                remaining -= written;
            }

            return total_written;
        }

        ssize_t Tty::write(const std::vector<uint8_t> &data) { return write(data.data(), data.size()); }

        ssize_t Tty::write(const std::string &data) {
            return write(reinterpret_cast<const uint8_t *>(data.data()), data.size());
        }

        // Read operations
        ssize_t Tty::read(uint8_t *buffer, size_t size) {
            if (buffer == nullptr || size == 0) {
                return 0;
            }

            if (!is_open()) {
                pimpl_->last_error = "Port is not open";
                return -1;
            }

            // Use select for read timeout
            struct timeval timeout;
            timeout.tv_sec = pimpl_->config.read_timeout_ms / 1000;
            timeout.tv_usec = (pimpl_->config.read_timeout_ms % 1000) * 1000;

            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(pimpl_->fd, &read_fds);

            int ret = select(pimpl_->fd + 1, &read_fds, nullptr, nullptr, &timeout);
            if (ret < 0) {
                pimpl_->last_error = std::string("Select error: ") + std::strerror(errno);
                return -1;
            } else if (ret == 0) {
                // Timeout
                return 0;
            }

            ssize_t bytes_read = ::read(pimpl_->fd, buffer, size);
            if (bytes_read < 0) {
                pimpl_->last_error = std::string("Read error: ") + std::strerror(errno);
                return -1;
            }

            return bytes_read;
        }

        ssize_t Tty::read(std::vector<uint8_t> &buffer, size_t size) {
            buffer.resize(size);
            ssize_t bytes_read = read(buffer.data(), size);
            if (bytes_read > 0) {
                buffer.resize(bytes_read);
            } else {
                buffer.clear();
            }
            return bytes_read;
        }

        std::vector<uint8_t> Tty::read_until(uint8_t delimiter, size_t max_size) {
            std::vector<uint8_t> result;
            result.reserve(256);

            uint8_t byte;
            while (result.size() < max_size) {
                ssize_t bytes_read = read(&byte, 1);
                if (bytes_read <= 0) {
                    break;
                }
                result.push_back(byte);
                if (byte == delimiter) {
                    break;
                }
            }

            return result;
        }

        std::string Tty::read_line(size_t max_size) {
            auto data = read_until('\n', max_size);
            if (data.empty()) {
                return "";
            }

            // Remove trailing '\n' and '\r'
            while (!data.empty() && (data.back() == '\n' || data.back() == '\r')) {
                data.pop_back();
            }

            return std::string(data.begin(), data.end());
        }

        ssize_t Tty::read_exact(uint8_t *buffer, size_t size, uint32_t timeout_ms) {
            if (buffer == nullptr || size == 0) {
                return 0;
            }

            if (!is_open()) {
                pimpl_->last_error = "Port is not open";
                return -1;
            }

            uint32_t timeout = timeout_ms > 0 ? timeout_ms : pimpl_->config.read_timeout_ms;

            auto start = std::chrono::steady_clock::now();
            size_t total_read = 0;

            while (total_read < size) {
                auto elapsed =
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start)
                        .count();

                if (elapsed >= timeout) {
                    pimpl_->last_error = "Timeout reading exact amount";
                    return total_read;
                }

                uint32_t remaining_timeout = timeout - elapsed;

                struct timeval tv;
                tv.tv_sec = remaining_timeout / 1000;
                tv.tv_usec = (remaining_timeout % 1000) * 1000;

                fd_set read_fds;
                FD_ZERO(&read_fds);
                FD_SET(pimpl_->fd, &read_fds);

                int ret = select(pimpl_->fd + 1, &read_fds, nullptr, nullptr, &tv);
                if (ret < 0) {
                    pimpl_->last_error = std::string("Select error: ") + std::strerror(errno);
                    return -1;
                } else if (ret == 0) {
                    continue; // Timeout on this iteration, try again
                }

                ssize_t bytes_read = ::read(pimpl_->fd, buffer + total_read, size - total_read);
                if (bytes_read < 0) {
                    pimpl_->last_error = std::string("Read error: ") + std::strerror(errno);
                    return -1;
                } else if (bytes_read == 0) {
                    continue;
                }

                total_read += bytes_read;
            }

            return total_read;
        }

        std::vector<uint8_t> Tty::read_exact(size_t size, uint32_t timeout_ms) {
            std::vector<uint8_t> buffer(size);
            ssize_t bytes_read = read_exact(buffer.data(), size, timeout_ms);
            if (bytes_read > 0) {
                buffer.resize(bytes_read);
            } else {
                buffer.clear();
            }
            return buffer;
        }

        // Buffer operations
        ssize_t Tty::available() const {
            if (!is_open()) {
                return -1;
            }

            int bytes = 0;
            if (ioctl(pimpl_->fd, FIONREAD, &bytes) < 0) {
                return -1;
            }

            return bytes;
        }

        void Tty::flush_input() {
            if (is_open()) {
                tcflush(pimpl_->fd, TCIFLUSH);
            }
        }

        void Tty::flush_output() {
            if (is_open()) {
                tcdrain(pimpl_->fd);
            }
        }

        void Tty::flush() {
            if (is_open()) {
                tcflush(pimpl_->fd, TCIOFLUSH);
            }
        }

        // Configuration setters
        bool Tty::set_baud_rate(uint32_t baud_rate) {
            if (!is_open()) {
                pimpl_->config.baud_rate = baud_rate;
                return true;
            }

            pimpl_->config.baud_rate = baud_rate;
            return set_termios_baud(baud_rate);
        }

        bool Tty::set_termios_baud(uint32_t baud_rate) {
            struct termios tty;
            if (tcgetattr(pimpl_->fd, &tty) != 0) {
                pimpl_->last_error = std::string("Failed to get attributes: ") + std::strerror(errno);
                return false;
            }

            speed_t speed = baud_rate_to_speed(baud_rate);
            if (speed == B0) {
                pimpl_->last_error = "Unsupported baud rate";
                return false;
            }

            cfsetispeed(&tty, speed);
            cfsetospeed(&tty, speed);

            if (tcsetattr(pimpl_->fd, TCSANOW, &tty) != 0) {
                pimpl_->last_error = std::string("Failed to set attributes: ") + std::strerror(errno);
                return false;
            }

            return true;
        }

        uint32_t Tty::get_baud_rate() const {
            if (!is_open()) {
                return pimpl_->config.baud_rate;
            }

            struct termios tty;
            if (tcgetattr(pimpl_->fd, &tty) != 0) {
                return pimpl_->config.baud_rate;
            }

            return speed_to_baud_rate(cfgetospeed(&tty));
        }

        void Tty::set_read_timeout(uint32_t timeout_ms) { pimpl_->config.read_timeout_ms = timeout_ms; }

        void Tty::set_write_timeout(uint32_t timeout_ms) { pimpl_->config.write_timeout_ms = timeout_ms; }

        bool Tty::set_data_bits(DataBits data_bits) {
            pimpl_->config.data_bits = data_bits;
            return is_open() ? configure_port() : true;
        }

        bool Tty::set_parity(Parity parity) {
            pimpl_->config.parity = parity;
            return is_open() ? configure_port() : true;
        }

        bool Tty::set_stop_bits(StopBits stop_bits) {
            pimpl_->config.stop_bits = stop_bits;
            return is_open() ? configure_port() : true;
        }

        bool Tty::set_flow_control(FlowControl flow_control) {
            pimpl_->config.flow_control = flow_control;
            return is_open() ? configure_port() : true;
        }

        bool Tty::apply_config(const SerialConfig &config) {
            pimpl_->config = config;
            return is_open() ? configure_port() : true;
        }

        SerialConfig Tty::get_config() const { return pimpl_->config; }

        std::string Tty::get_port() const { return pimpl_->port; }

        std::string Tty::get_last_error() const { return pimpl_->last_error; }

        // Static methods
        std::vector<std::string> Tty::list_ports() {
            std::vector<std::string> ports;

            // Common serial port patterns on Linux
            const std::vector<std::string> patterns = {"/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS*", "/dev/ttyAMA*",
                                                       "/dev/rfcomm*"};

            for (const auto &pattern : patterns) {
                std::filesystem::path base_path = std::filesystem::path(pattern).parent_path();
                std::string prefix = std::filesystem::path(pattern).filename().string();
                prefix = prefix.substr(0, prefix.find('*'));

                try {
                    if (std::filesystem::exists(base_path)) {
                        for (const auto &entry : std::filesystem::directory_iterator(base_path)) {
                            if (entry.is_character_file() || entry.is_symlink()) {
                                std::string filename = entry.path().filename().string();
                                if (filename.find(prefix) == 0) {
                                    ports.push_back(entry.path().string());
                                }
                            }
                        }
                    }
                } catch (const std::filesystem::filesystem_error &) {
                    // Ignore errors, continue with other patterns
                }
            }

            std::sort(ports.begin(), ports.end());
            return ports;
        }

        bool Tty::port_exists(const std::string &port) {
            try {
                std::filesystem::path p(port);
                return std::filesystem::exists(p) &&
                       (std::filesystem::is_character_file(p) || std::filesystem::is_symlink(p));
            } catch (const std::filesystem::filesystem_error &) {
                return false;
            }
        }

        // Control line operations
        bool Tty::set_dtr(bool state) {
            if (!is_open()) {
                pimpl_->last_error = "Port is not open";
                return false;
            }

            int status;
            if (ioctl(pimpl_->fd, TIOCMGET, &status) < 0) {
                pimpl_->last_error = std::string("Failed to get modem status: ") + std::strerror(errno);
                return false;
            }

            if (state) {
                status |= TIOCM_DTR;
            } else {
                status &= ~TIOCM_DTR;
            }

            if (ioctl(pimpl_->fd, TIOCMSET, &status) < 0) {
                pimpl_->last_error = std::string("Failed to set DTR: ") + std::strerror(errno);
                return false;
            }

            return true;
        }

        bool Tty::set_rts(bool state) {
            if (!is_open()) {
                pimpl_->last_error = "Port is not open";
                return false;
            }

            int status;
            if (ioctl(pimpl_->fd, TIOCMGET, &status) < 0) {
                pimpl_->last_error = std::string("Failed to get modem status: ") + std::strerror(errno);
                return false;
            }

            if (state) {
                status |= TIOCM_RTS;
            } else {
                status &= ~TIOCM_RTS;
            }

            if (ioctl(pimpl_->fd, TIOCMSET, &status) < 0) {
                pimpl_->last_error = std::string("Failed to set RTS: ") + std::strerror(errno);
                return false;
            }

            return true;
        }

        bool Tty::get_cts() const {
            if (!is_open()) {
                return false;
            }

            int status;
            if (ioctl(pimpl_->fd, TIOCMGET, &status) < 0) {
                return false;
            }

            return (status & TIOCM_CTS) != 0;
        }

        bool Tty::get_dsr() const {
            if (!is_open()) {
                return false;
            }

            int status;
            if (ioctl(pimpl_->fd, TIOCMGET, &status) < 0) {
                return false;
            }

            return (status & TIOCM_DSR) != 0;
        }

        bool Tty::get_ri() const {
            if (!is_open()) {
                return false;
            }

            int status;
            if (ioctl(pimpl_->fd, TIOCMGET, &status) < 0) {
                return false;
            }

            return (status & TIOCM_RI) != 0;
        }

        bool Tty::get_cd() const {
            if (!is_open()) {
                return false;
            }

            int status;
            if (ioctl(pimpl_->fd, TIOCMGET, &status) < 0) {
                return false;
            }

            return (status & TIOCM_CD) != 0;
        }

        void Tty::send_break(uint32_t duration_ms) {
            if (!is_open()) {
                return;
            }

            int duration = duration_ms > 0 ? duration_ms : 250;
            tcsendbreak(pimpl_->fd, duration);
        }

        int Tty::get_fd() const { return pimpl_->fd; }

    } // namespace comms
} // namespace tractor
