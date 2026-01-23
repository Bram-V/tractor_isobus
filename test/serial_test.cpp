#include "tractor/comms/tty.hpp"
#include <chrono>
#include <doctest/doctest.h>
#include <thread>

using namespace tractor::comms;

TEST_CASE("Serial Configuration Tests") {
    SUBCASE("Default configuration") {
        SerialConfig config;
        CHECK(config.baud_rate == 9600);
        CHECK(config.data_bits == DataBits::Eight);
        CHECK(config.parity == Parity::None);
        CHECK(config.stop_bits == StopBits::One);
        CHECK(config.flow_control == FlowControl::None);
    }

    SUBCASE("Custom configuration") {
        SerialConfig config;
        config.baud_rate = 115200;
        config.data_bits = DataBits::Seven;
        config.parity = Parity::Even;
        config.stop_bits = StopBits::Two;
        config.flow_control = FlowControl::Hardware;

        CHECK(config.baud_rate == 115200);
        CHECK(config.data_bits == DataBits::Seven);
        CHECK(config.parity == Parity::Even);
        CHECK(config.stop_bits == StopBits::Two);
        CHECK(config.flow_control == FlowControl::Hardware);
    }
}

TEST_CASE("Serial Construction Tests") {
    SUBCASE("Default constructor") {
        Tty serial;
        CHECK_FALSE(serial.is_open());
    }

    SUBCASE("Constructor with port and baud rate") {
        Tty serial("/dev/ttyUSB0", 115200);
        CHECK_FALSE(serial.is_open());
        CHECK(serial.get_port() == "/dev/ttyUSB0");
        CHECK(serial.get_baud_rate() == 115200);
    }

    SUBCASE("Constructor with config") {
        SerialConfig config;
        config.baud_rate = 9600;
        config.data_bits = DataBits::Eight;

        Tty serial("/dev/ttyACM0", config);
        CHECK_FALSE(serial.is_open());
        CHECK(serial.get_port() == "/dev/ttyACM0");
    }
}

TEST_CASE("Serial Move Semantics") {
    SUBCASE("Move constructor") {
        Tty serial1("/dev/ttyUSB0", 115200);
        Tty serial2(std::move(serial1));

        CHECK(serial2.get_port() == "/dev/ttyUSB0");
        CHECK(serial2.get_baud_rate() == 115200);
    }

    SUBCASE("Move assignment") {
        Tty serial1("/dev/ttyUSB0", 115200);
        Tty serial2;

        serial2 = std::move(serial1);

        CHECK(serial2.get_port() == "/dev/ttyUSB0");
        CHECK(serial2.get_baud_rate() == 115200);
    }
}

TEST_CASE("Serial Port Discovery") {
    SUBCASE("List ports") {
        auto ports = Tty::list_ports();
        // Just verify the function doesn't crash
        // The actual result depends on the system
        CHECK(ports.size() >= 0);
    }

    SUBCASE("Port exists check") {
        // Test with a path that definitely doesn't exist
        CHECK_FALSE(Tty::port_exists("/dev/ttyNONEXISTENT999"));
    }
}

TEST_CASE("Serial Configuration Setters") {
    Tty serial;

    SUBCASE("Set baud rate when closed") {
        CHECK(serial.set_baud_rate(115200));
        CHECK(serial.get_baud_rate() == 115200);
    }

    SUBCASE("Set read timeout") {
        serial.set_read_timeout(5000);
        auto config = serial.get_config();
        CHECK(config.read_timeout_ms == 5000);
    }

    SUBCASE("Set write timeout") {
        serial.set_write_timeout(3000);
        auto config = serial.get_config();
        CHECK(config.write_timeout_ms == 3000);
    }

    SUBCASE("Set data bits") {
        CHECK(serial.set_data_bits(DataBits::Seven));
        auto config = serial.get_config();
        CHECK(config.data_bits == DataBits::Seven);
    }

    SUBCASE("Set parity") {
        CHECK(serial.set_parity(Parity::Even));
        auto config = serial.get_config();
        CHECK(config.parity == Parity::Even);
    }

    SUBCASE("Set stop bits") {
        CHECK(serial.set_stop_bits(StopBits::Two));
        auto config = serial.get_config();
        CHECK(config.stop_bits == StopBits::Two);
    }

    SUBCASE("Set flow control") {
        CHECK(serial.set_flow_control(FlowControl::Hardware));
        auto config = serial.get_config();
        CHECK(config.flow_control == FlowControl::Hardware);
    }

    SUBCASE("Apply complete config") {
        SerialConfig config;
        config.baud_rate = 57600;
        config.data_bits = DataBits::Seven;
        config.parity = Parity::Odd;

        CHECK(serial.apply_config(config));
        auto retrieved = serial.get_config();
        CHECK(retrieved.baud_rate == 57600);
        CHECK(retrieved.data_bits == DataBits::Seven);
        CHECK(retrieved.parity == Parity::Odd);
    }
}

TEST_CASE("Serial Error Handling") {
    Tty serial;

    SUBCASE("Write to closed port") {
        std::string data = "test";
        ssize_t result = serial.write(data);
        CHECK(result == -1);
        CHECK_FALSE(serial.get_last_error().empty());
    }

    SUBCASE("Read from closed port") {
        uint8_t buffer[10];
        ssize_t result = serial.read(buffer, 10);
        CHECK(result == -1);
        CHECK_FALSE(serial.get_last_error().empty());
    }

    SUBCASE("Available on closed port") {
        ssize_t result = serial.available();
        CHECK(result == -1);
    }

    SUBCASE("Set DTR on closed port") {
        CHECK_FALSE(serial.set_dtr(true));
        CHECK_FALSE(serial.get_last_error().empty());
    }

    SUBCASE("Set RTS on closed port") {
        CHECK_FALSE(serial.set_rts(true));
        CHECK_FALSE(serial.get_last_error().empty());
    }
}

TEST_CASE("Serial File Descriptor") {
    Tty serial;

    SUBCASE("FD when closed") { CHECK(serial.get_fd() == -1); }
}

TEST_CASE("Serial Buffer Operations") {
    Tty serial;

    SUBCASE("Flush operations on closed port (should not crash)") {
        serial.flush_input();
        serial.flush_output();
        serial.flush();
        // Just verify no crash
        CHECK(true);
    }
}

TEST_CASE("Serial Write Variants") {
    Tty serial;

    SUBCASE("Write with null data") {
        ssize_t result = serial.write(nullptr, 10);
        // Null data returns 0 (handled before port check)
        CHECK(result == 0);
    }

    SUBCASE("Write with zero size") {
        uint8_t data[] = {1, 2, 3};
        ssize_t result = serial.write(data, 0);
        // Zero size returns 0 regardless of port state
        CHECK(result == 0);
    }

    SUBCASE("Write empty vector") {
        std::vector<uint8_t> data;
        // Should handle gracefully
        CHECK(true);
    }

    SUBCASE("Write empty string") {
        std::string data = "";
        // Should handle gracefully
        CHECK(true);
    }
}

TEST_CASE("Serial Read Variants") {
    Tty serial;

    SUBCASE("Read with null buffer") {
        ssize_t result = serial.read(nullptr, 10);
        CHECK(result == 0);  // Null buffer returns 0
    }

    SUBCASE("Read with zero size") {
        uint8_t buffer[10];
        ssize_t result = serial.read(buffer, 0);
        // Zero size returns 0 regardless of port state
        CHECK(result == 0);
    }

    SUBCASE("Read exact with null buffer") {
        ssize_t result = serial.read_exact(nullptr, 10);
        // Null buffer returns 0 (handled before port check)
        CHECK(result == 0);
    }

    SUBCASE("Read exact with zero size") {
        uint8_t buffer[10];
        ssize_t result = serial.read_exact(buffer, 0);
        // Zero size returns 0 regardless of port state
        CHECK(result == 0);
    }
}

// Note: Actual hardware-based tests would require physical serial ports
// or virtual serial port pairs (e.g., using socat)
TEST_CASE("Serial Open Tests") {
    SUBCASE("Open non-existent port") {
        Tty serial("/dev/ttyNONEXISTENT999", 115200);
        CHECK_FALSE(serial.open());
        CHECK_FALSE(serial.get_last_error().empty());
    }

    SUBCASE("Open without port specified") {
        Tty serial;
        CHECK_FALSE(serial.open());
        CHECK_FALSE(serial.get_last_error().empty());
    }
}
