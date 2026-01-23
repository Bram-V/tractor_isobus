#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"
#include "tractor/comms/serial.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

// Global state
static std::atomic_bool running = true;
static std::atomic<std::int32_t> gnss_auth_status = 0; // 0=unauthenticated, 1=authenticated, 2=degraded
static std::atomic<std::int32_t> gnss_warning = 0;

// GPS position data (for TC-GEO)
static std::atomic<double> gnss_latitude = 0.0;
static std::atomic<double> gnss_longitude = 0.0;
static std::atomic<double> gnss_altitude = 0.0;
static std::atomic<bool> gnss_position_valid = false;

// Pointer to TC client (needed for GPS updates)
static std::shared_ptr<isobus::TaskControllerClient> g_tc_client = nullptr;

void signal_handler(int) { running = false; }

// Proprietary DDI codes (ISO 11783-10: 50000-65535 are safe for proprietary use)
constexpr std::uint16_t DDI_GNSS_AUTH_STATE = 50000; // 0=unauthenticated, 1=authenticated, 2=degraded
constexpr std::uint16_t DDI_GNSS_WARNING = 50001;    // Warning status flags

enum class DDOPObjectIDs : std::uint16_t {
    Device = 0,
    MainDeviceElement = 1,
    Connector = 3,
    ConnectorXOffset = 4,
    ConnectorYOffset = 5,
    Implement = 6,
    GNSSAuthStatusPD = 7, // Process Data for Auth Status (DDI 50000)
    GNSSWarningPD = 8,    // Process Data for Warning (DDI 50001)
    AreaPresentation = 100,
    WidthPresentation = 101
};

// PHTG (Proprietary GNSS Authentication) sentence structure
struct PHTGData {
    std::string date;    // Field 0: Date (DD:MM:YYYY)
    std::string time;    // Field 1: Time (HH:MM:SS.SS)
    std::string system;  // Field 2: GNSS System
    std::string service; // Field 3: Service (e.g., HAS, OSNMA)
    int auth_result;     // Field 4: Authentication result (0=fail, 1=pass)
    int warning;         // Field 5: Warning status
};

// GGA (GPS Fix Data) sentence structure
struct GGAData {
    double latitude;  // Decimal degrees
    double longitude; // Decimal degrees
    double altitude;  // Meters above sea level
    int fix_quality;  // 0=invalid, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
    int num_sats;     // Number of satellites
    double hdop;      // Horizontal dilution of precision
};

bool validate_checksum(const std::string &sentence) {
    size_t star_pos = sentence.find('*');
    if (star_pos == std::string::npos || star_pos + 2 >= sentence.length()) {
        return false;
    }

    std::uint8_t calc_cs = 0;
    for (size_t i = 1; i < star_pos; i++) {
        calc_cs ^= static_cast<std::uint8_t>(sentence[i]);
    }

    std::string cs_str = sentence.substr(star_pos + 1, 2);
    int recv_cs = std::stoi(cs_str, nullptr, 16);

    return calc_cs == recv_cs;
}

// Convert NMEA coordinate format (DDMM.MMMM) to decimal degrees
double nmea_to_decimal(const std::string &coord, const std::string &direction) {
    if (coord.empty())
        return 0.0;

    // Find decimal point
    size_t dot_pos = coord.find('.');
    if (dot_pos == std::string::npos || dot_pos < 2)
        return 0.0;

    // Extract degrees and minutes
    int degrees = std::stoi(coord.substr(0, dot_pos - 2));
    double minutes = std::stod(coord.substr(dot_pos - 2));

    double decimal = degrees + (minutes / 60.0);

    // Apply direction (S and W are negative)
    if (direction == "S" || direction == "W") {
        decimal = -decimal;
    }

    return decimal;
}

bool parse_phtg(const std::string &sentence, PHTGData &data) {
    if (sentence.substr(0, 5) != "$PHTG") {
        return false;
    }

    if (!validate_checksum(sentence)) {
        return false;
    }

    size_t star_pos = sentence.find('*');
    std::string body = sentence.substr(6, star_pos - 6);

    std::stringstream ss(body);
    std::string token;
    int field = 0;

    while (std::getline(ss, token, ',')) {
        switch (field) {
        case 0:
            data.date = token;
            break;
        case 1:
            data.time = token;
            break;
        case 2:
            data.system = token;
            break;
        case 3:
            data.service = token;
            break;
        case 4:
            data.auth_result = token.empty() ? 0 : std::stoi(token);
            break;
        case 5:
            data.warning = token.empty() ? 0 : std::stoi(token);
            break;
        }
        field++;
    }

    return field >= 6;
}

bool parse_gga(const std::string &sentence, GGAData &data) {
    if (sentence.substr(0, 6) != "$GPGGA" && sentence.substr(0, 6) != "$GNGGA") {
        return false;
    }

    if (!validate_checksum(sentence)) {
        return false;
    }

    size_t star_pos = sentence.find('*');
    std::string body = sentence.substr(7, star_pos - 7);

    std::stringstream ss(body);
    std::string token;
    int field = 0;

    std::string lat_str, lat_dir, lon_str, lon_dir, alt_str;

    while (std::getline(ss, token, ',')) {
        switch (field) {
        case 0: /* time */
            break;
        case 1:
            lat_str = token;
            break;
        case 2:
            lat_dir = token;
            break;
        case 3:
            lon_str = token;
            break;
        case 4:
            lon_dir = token;
            break;
        case 5:
            data.fix_quality = token.empty() ? 0 : std::stoi(token);
            break;
        case 6:
            data.num_sats = token.empty() ? 0 : std::stoi(token);
            break;
        case 7:
            data.hdop = token.empty() ? 0.0 : std::stod(token);
            break;
        case 8:
            alt_str = token;
            break;
        }
        field++;
    }

    if (field < 9)
        return false;

    data.latitude = nmea_to_decimal(lat_str, lat_dir);
    data.longitude = nmea_to_decimal(lon_str, lon_dir);
    data.altitude = alt_str.empty() ? 0.0 : std::stod(alt_str);

    return data.fix_quality > 0;
}

void process_nmea_line(const std::string &line) {
    // Process PHTG (Authentication) messages
    if (line.substr(0, 5) == "$PHTG") {
        PHTGData phtg;
        if (parse_phtg(line, phtg)) {
            std::cout << "PHTG: [" << phtg.date << " " << phtg.time << "] " << phtg.system << "/" << phtg.service
                      << " Auth=" << phtg.auth_result << " Warn=" << phtg.warning << "\n";

            gnss_auth_status.store(phtg.auth_result);
            gnss_warning.store(phtg.warning);
        }
    }
    // Process GPGGA (GPS position) messages
    else if (line.substr(0, 6) == "$GPGGA" || line.substr(0, 6) == "$GNGGA") {
        GGAData gga;
        if (parse_gga(line, gga)) {
            gnss_latitude.store(gga.latitude);
            gnss_longitude.store(gga.longitude);
            gnss_altitude.store(gga.altitude);
            gnss_position_valid.store(true);

            std::cout << "GGA: Lat=" << gga.latitude << " Lon=" << gga.longitude << " Alt=" << gga.altitude
                      << " Fix=" << gga.fix_quality << " Sats=" << gga.num_sats << "\n";

            // Send position to Task Controller via TC-GEO mechanism
            if (g_tc_client != nullptr) {
                // TC-GEO expects position in specific format
                // The library will handle PGN 0x1FDD generation
                // Note: Check your AgIsoStack++ version for exact API
                // Some versions use: set_gps_position() or update_position()
                // For now, we'll store it and let the TC client handle it in its update cycle
            }
        }
    }
}

// Create ISOBUS-TC compliant DDOP with proprietary DDIs
bool create_simple_ddop(std::shared_ptr<isobus::DeviceDescriptorObjectPool> ddop, isobus::NAME clientName) {
    bool success = true;
    ddop->clear();

    // Device localization: English (en)
    std::array<std::uint8_t, 7> localizationData = {'e', 'n', 0x50, 0x00, 0x55, 0x55, 0xFF};

    // Add Device object
    success &= ddop->add_device("GNSS_Auth_Device", "1.0.0", "001", "GAD1", localizationData,
                                std::vector<std::uint8_t>(), clientName.get_full_name());

    // Add Main Device Element (required by ISOBUS)
    success &=
        ddop->add_device_element("MainDevice", 0, 0, isobus::task_controller_object::DeviceElementObject::Type::Device,
                                 static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement));

    // Add Connector element (required for implements)
    success &= ddop->add_device_element("Connector", 1, static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement),
                                        isobus::task_controller_object::DeviceElementObject::Type::Connector,
                                        static_cast<std::uint16_t>(DDOPObjectIDs::Connector));

    // Connector offset process data (X and Y)
    success &= ddop->add_device_process_data(
        "ConnectorX", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable), 0,
        static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorXOffset));

    success &= ddop->add_device_process_data(
        "ConnectorY", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable), 0,
        static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorYOffset));

    // Add Function/Implement element
    success &=
        ddop->add_device_element("GNSS_Function", 2, static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement),
                                 isobus::task_controller_object::DeviceElementObject::Type::Function,
                                 static_cast<std::uint16_t>(DDOPObjectIDs::Implement));

    // ✅ PROPRIETARY PROCESS DATA: GNSS Authentication Status
    // Using proprietary DDI 50000 (ISO 11783-10 compliant)
    // Values: 0=unauthenticated, 1=authenticated, 2=degraded/OSNMA
    success &= ddop->add_device_process_data(
        "GNSS_Auth_Status", DDI_GNSS_AUTH_STATE, // Proprietary DDI 50000
        isobus::NULL_OBJECT_ID,                  // No presentation needed for simple integer
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
        static_cast<std::uint16_t>(DDOPObjectIDs::GNSSAuthStatusPD));

    // ✅ PROPRIETARY PROCESS DATA: GNSS Warning Status
    // Using proprietary DDI 50001
    success &= ddop->add_device_process_data(
        "GNSS_Warning", DDI_GNSS_WARNING, // Proprietary DDI 50001
        isobus::NULL_OBJECT_ID,           // No presentation needed
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
        static_cast<std::uint16_t>(DDOPObjectIDs::GNSSWarningPD));

    // Add presentation objects
    success &= ddop->add_device_value_presentation("mm", 0, 1.0f, 0,
                                                   static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation));

    success &= ddop->add_device_value_presentation("m^2", 0, 1.0f, 0,
                                                   static_cast<std::uint16_t>(DDOPObjectIDs::AreaPresentation));

    // Link child objects to parent elements (matching main.cpp pattern)
    if (success) {
        auto mainElement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement)));
        auto connector = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::Connector)));
        auto implement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::Implement)));

        // Link connector offsets to connector element
        connector->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorXOffset));
        connector->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorYOffset));

        // Link proprietary process data to implement element
        implement->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::GNSSAuthStatusPD));
        implement->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::GNSSWarningPD));
    }

    return success;
}

int main(int argc, char **argv) {
    const char *serial_device = "/tmp/ttyV0";
    int serial_baud = 9600;

    if (argc > 1) {
        serial_device = argv[1];
    }
    if (argc > 2) {
        serial_baud = std::atoi(argv[2]);
    }

    std::cout << "========================================\n";
    std::cout << "NMEA to ISOBUS TC-Compliant Bridge\n";
    std::cout << "========================================\n";
    std::cout << "Serial: " << serial_device << " @ " << serial_baud << " baud\n";
    std::cout << "Proprietary DDIs:\n";
    std::cout << "  - 50000: GNSS_AUTH_STATE (0=unauth, 1=auth, 2=degraded)\n";
    std::cout << "  - 50001: GNSS_WARNING\n";
    std::cout << "========================================\n\n";

    // Initialize CAN interface
    auto can_driver = std::make_shared<isobus::SocketCANInterface>("can0");
    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, can_driver);

    if ((!isobus::CANHardwareInterface::start()) || (!can_driver->get_is_valid())) {
        std::cerr << "❌ Failed to start CAN interface\n";
        return -1;
    }

    std::signal(SIGINT, signal_handler);

    // Create ISOBUS NAME
    isobus::NAME my_name(0);
    my_name.set_arbitrary_address_capable(true);
    my_name.set_industry_group(2); // Agricultural
    my_name.set_device_class(6);
    my_name.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
    my_name.set_identity_number(43);
    my_name.set_ecu_instance(0);
    my_name.set_function_instance(0);
    my_name.set_device_class_instance(0);
    my_name.set_manufacturer_code(1407);

    // Create internal control function
    auto my_ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(my_name, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    // Create partnered control function (Task Controller)
    const isobus::NAMEFilter filterTaskController(isobus::NAME::NAMEParameters::FunctionCode,
                                                  static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));
    const isobus::NAMEFilter filterTaskControllerInstance(isobus::NAME::NAMEParameters::FunctionInstance, 0);
    const isobus::NAMEFilter filterTaskControllerIndustryGroup(
        isobus::NAME::NAMEParameters::IndustryGroup,
        static_cast<std::uint8_t>(isobus::NAME::IndustryGroup::AgriculturalAndForestryEquipment));
    const isobus::NAMEFilter filterTaskControllerDeviceClass(
        isobus::NAME::NAMEParameters::DeviceClass, static_cast<std::uint8_t>(isobus::NAME::DeviceClass::NonSpecific));

    const std::vector<isobus::NAMEFilter> tcNameFilters = {filterTaskController, filterTaskControllerInstance,
                                                           filterTaskControllerIndustryGroup,
                                                           filterTaskControllerDeviceClass};

    auto tc_partner = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, tcNameFilters);

    // Create Task Controller Client
    auto tc = std::make_shared<isobus::TaskControllerClient>(tc_partner, my_ecu, nullptr);
    g_tc_client = tc; // Store globally for GPS updates

    // Create DDOP with proprietary DDIs
    auto ddop = std::make_shared<isobus::DeviceDescriptorObjectPool>();
    if (!create_simple_ddop(ddop, my_ecu->get_NAME())) {
        std::cerr << "❌ Failed to create DDOP\n";
        return -1;
    }
    std::cout << "✅ DDOP created with proprietary DDIs\n";

    // ✅ Register callback for TC value requests (PGN 0xEF00)
    // This is how the TC will request and log our proprietary values
    tc->add_request_value_callback(
        [](std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t &outValue, void *parent) -> bool {
            switch (ddi) {
            case DDI_GNSS_AUTH_STATE:
                outValue = gnss_auth_status.load();
                std::cout << "📡 TC requests GNSS_AUTH_STATE (DDI 50000): " << outValue << "\n";
                return true;

            case DDI_GNSS_WARNING:
                outValue = gnss_warning.load();
                std::cout << "📡 TC requests GNSS_WARNING (DDI 50001): " << outValue << "\n";
                return true;

            default:
                return false;
            }
        },
        nullptr);

    // Value command callback (not used, but required)
    tc->add_value_command_callback([](std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t processVariableValue,
                                      void *parent) -> bool { return false; },
                                   nullptr);

    // Configure and initialize TC client
    tc->configure(ddop, 1, 0, 0, true, false, false, false, false);
    tc->initialize(true);
    std::cout << "✅ Task Controller Client initialized\n";

    // Set up serial communication for NMEA data
    auto nmea_serial = std::make_shared<tractor::comms::Serial>(serial_device, serial_baud);

    nmea_serial->on_line([](const std::string &line) { process_nmea_line(line); });

    nmea_serial->on_connection([](bool connected) {
        if (connected) {
            std::cout << "✅ Serial port connected\n";
        } else {
            std::cout << "⚠️  Serial port disconnected\n";
        }
    });

    nmea_serial->on_error([](const std::string &error) { std::cerr << "❌ Serial error: " << error << "\n"; });

    if (!nmea_serial->start()) {
        std::cerr << "❌ Failed to start serial communication\n";
        return -1;
    }

    std::cout << "\n✅ System running... Press Ctrl+C to exit\n";
    std::cout << "========================================\n\n";

    // Main loop - TC client updates and GPS position transmission
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // TODO: Send GPS position via TC-GEO
        // The exact API depends on AgIsoStack++ version
        // Common methods:
        //   tc->set_gps_position(lat, lon, alt);
        //   tc->update_position(...);
        // This generates TC-GEO messages (PGN 0x1FDD) for the Task Controller

        if (gnss_position_valid.load()) {
            // Position is ready to be sent
            // Uncomment and adapt based on your library version:
            // tc->set_gps_position(gnss_latitude.load(), gnss_longitude.load(), gnss_altitude.load());
        }
    }

    std::cout << "\n========================================\n";
    std::cout << "Shutting down...\n";
    std::cout << "========================================\n";

    nmea_serial->stop();
    tc->terminate();
    isobus::CANHardwareInterface::stop();

    return 0;
}
