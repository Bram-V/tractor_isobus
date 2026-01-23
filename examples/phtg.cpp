
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"
#include "tractor/comms/serial.hpp"

#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

static std::atomic_bool running = true;
static std::atomic<std::int32_t> current_work_state = 0;
static std::atomic<std::int32_t> gnss_auth_status = 0;
static std::atomic<std::int32_t> gnss_warning = 0;

void signal_handler(int) { running = false; }

struct PHTGData {
    std::string date;
    std::string time;
    std::string system;
    std::string service;
    int auth_result;
    int warning;
};

bool validate_checksum(const std::string &sentence) {
    try {
        size_t star_pos = sentence.find('*');
        if (star_pos == std::string::npos || star_pos + 2 >= sentence.length()) {
            return false;
        }

        std::uint8_t calc_cs = 0;
        for (size_t i = 1; i < star_pos; i++) {
            calc_cs ^= static_cast<std::uint8_t>(sentence[i]);
        }

        std::string cs_str = sentence.substr(star_pos + 1, 2);
        if (cs_str.empty() || cs_str.length() < 2) {
            return false;
        }
        int recv_cs = std::stoi(cs_str, nullptr, 16);

        return calc_cs == recv_cs;
    } catch (...) {
        return false;
    }
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
            if (!token.empty()) {
                try {
                    data.auth_result = std::stoi(token);
                } catch (...) {
                    data.auth_result = 0;
                }
            } else {
                data.auth_result = 0;
            }
            break;
        case 5:
            if (!token.empty()) {
                try {
                    data.warning = std::stoi(token);
                } catch (...) {
                    data.warning = 0;
                }
            } else {
                data.warning = 0;
            }
            break;
        }
        field++;
    }

    return field >= 6;
}

void process_nmea_line(const std::string &line) {
    try {
        if (line.length() >= 5 && line.substr(0, 5) == "$PHTG") {
            PHTGData phtg;
            if (parse_phtg(line, phtg)) {
                gnss_auth_status.store(phtg.auth_result);
                gnss_warning.store(phtg.warning);
                std::cout << "📡 PHTG: Auth=" << phtg.auth_result << " Warning=" << phtg.warning << "\n";
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "⚠ NMEA parse error: " << e.what() << "\n";
    } catch (...) {
        std::cerr << "⚠ Unknown NMEA parse error\n";
    }
}

enum class DDOPObjectIDs : std::uint16_t {
    Device = 0,

    MainDeviceElement = 1, // The main implement
    Connector = 3,         // Hitch/attachment point
    Boom = 6,              // Boom/tool

    DeviceActualWorkState = 2, // Is device working? (0=off, 1=on)
    ConnectorXOffset = 4,      // Fore/aft position from reference
    ConnectorYOffset = 5,      // Left/right position from reference

    ActualWorkingWidth = 8,
    ActualCondensedWorkState1To16 = 9,
    SetpointCondensedWorkState1To16 = 10,

    SectionLeft = 11,
    SectionLeftXOffset = 12,
    SectionLeftYOffset = 13,
    SectionLeftWidth = 14,

    SectionRight = 15,
    SectionRightXOffset = 16,
    SectionRightYOffset = 17,
    SectionRightWidth = 18,

    WidthPresentation = 101 // For width/distance (mm)
};

bool create_simple_ddop(std::shared_ptr<isobus::DeviceDescriptorObjectPool> ddop, isobus::NAME clientName) {
    bool success = true;

    ddop->clear();

    std::array<std::uint8_t, 7> localizationData = {'e', 'n', 0x50, 0x00, 0x55, 0x55, 0xFF};

    success &= ddop->add_device("SimpleImplement", "1.0.0", "001", "SMP1.0", localizationData,
                                std::vector<std::uint8_t>(), clientName.get_full_name());

    success &=
        ddop->add_device_element("Implement", 0, 0, isobus::task_controller_object::DeviceElementObject::Type::Device,
                                 static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement));

    success &= ddop->add_device_process_data(
        "Actual Work State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState),
        isobus::NULL_OBJECT_ID,
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
        static_cast<std::uint16_t>(DDOPObjectIDs::DeviceActualWorkState));

    success &= ddop->add_device_element("Connector", 1, static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement),
                                        isobus::task_controller_object::DeviceElementObject::Type::Connector,
                                        static_cast<std::uint16_t>(DDOPObjectIDs::Connector));

    success &= ddop->add_device_process_data(
        "Connector X", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable), 0,
        static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorXOffset));

    success &= ddop->add_device_process_data(
        "Connector Y", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable), 0,
        static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorYOffset));

    // Boom
    success &= ddop->add_device_element("Boom", 2, static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement),
                                        isobus::task_controller_object::DeviceElementObject::Type::Function,
                                        static_cast<std::uint16_t>(DDOPObjectIDs::Boom));

    success &= ddop->add_device_process_data(
        "Working Width", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
        static_cast<std::uint16_t>(DDOPObjectIDs::ActualWorkingWidth));

    success &= ddop->add_device_process_data(
        "Actual Work State 1-16",
        static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16), isobus::NULL_OBJECT_ID,
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
        static_cast<std::uint16_t>(DDOPObjectIDs::ActualCondensedWorkState1To16));

    success &= ddop->add_device_process_data(
        "Setpoint Work State 1-16",
        static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16),
        isobus::NULL_OBJECT_ID,
        static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable) |
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
        static_cast<std::uint16_t>(DDOPObjectIDs::SetpointCondensedWorkState1To16));

    // Section Left
    constexpr std::int32_t BOOM_WIDTH = 12000;
    constexpr std::int32_t SECTION_WIDTH = BOOM_WIDTH / 2;

    success &= ddop->add_device_element("Section Left", 3, static_cast<std::uint16_t>(DDOPObjectIDs::Boom),
                                        isobus::task_controller_object::DeviceElementObject::Type::Section,
                                        static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeft));

    success &= ddop->add_device_property("Offset X", 0,
                                         static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeftXOffset));

    success &= ddop->add_device_property("Offset Y", -SECTION_WIDTH / 2,
                                         static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeftYOffset));

    success &= ddop->add_device_property("Width", SECTION_WIDTH,
                                         static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeftWidth));

    // Section Right
    success &= ddop->add_device_element("Section Right", 4, static_cast<std::uint16_t>(DDOPObjectIDs::Boom),
                                        isobus::task_controller_object::DeviceElementObject::Type::Section,
                                        static_cast<std::uint16_t>(DDOPObjectIDs::SectionRight));

    success &= ddop->add_device_property("Offset X", 0,
                                         static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::SectionRightXOffset));

    success &= ddop->add_device_property("Offset Y", SECTION_WIDTH / 2,
                                         static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::SectionRightYOffset));

    success &= ddop->add_device_property("Width", SECTION_WIDTH,
                                         static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
                                         static_cast<std::uint16_t>(DDOPObjectIDs::SectionRightWidth));

    success &= ddop->add_device_value_presentation("mm", 0, 1.0f, 0,
                                                   static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation));

    if (success) {
        auto mainElement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement)));
        auto connector = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::Connector)));
        auto boom = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::Boom)));
        auto sectionLeft = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeft)));
        auto sectionRight = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::SectionRight)));

        mainElement->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::DeviceActualWorkState));

        connector->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorXOffset));
        connector->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorYOffset));

        boom->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ActualWorkingWidth));
        boom->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ActualCondensedWorkState1To16));
        boom->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::SetpointCondensedWorkState1To16));

        sectionLeft->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeftXOffset));
        sectionLeft->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeftYOffset));
        sectionLeft->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::SectionLeftWidth));

        sectionRight->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::SectionRightXOffset));
        sectionRight->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::SectionRightYOffset));
        sectionRight->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::SectionRightWidth));
    }

    return success;
}

int main(int argc, char **argv) {
    const char *serial_device = "/tmp/ttyV0";
    int serial_baud = 115200;

    if (argc > 1) {
        serial_device = argv[1];
    }
    if (argc > 2) {
        serial_baud = std::atoi(argv[2]);
    }

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       ISOBUS TASK CONTROLLER CLIENT EXAMPLE                 ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "[1/9] Initializing Serial interface...\n";
    std::cout << "Serial: " << serial_device << " @ " << serial_baud << " baud\n";

    auto nmea_serial = std::make_shared<tractor::comms::Serial>(serial_device, serial_baud);

    nmea_serial->on_line([](const std::string &line) { process_nmea_line(line); });

    nmea_serial->on_connection([](bool connected) {
        if (connected) {
            std::cout << "✓ Serial port connected\n";
        } else {
            std::cout << "⚠  Serial port disconnected\n";
        }
    });

    nmea_serial->on_error([](const std::string &error) { std::cerr << "❌ Serial error: " << error << "\n"; });

    if (!nmea_serial->start()) {
        std::cerr << "❌ Failed to start serial communication\n";
        return -1;
    }

    std::cout << "✓ Serial interface started successfully\n\n";

    std::cout << "[2/9] Initializing CAN interface...\n";

    auto can_driver = std::make_shared<isobus::SocketCANInterface>("can0");

    isobus::CANHardwareInterface::set_number_of_can_channels(1);

    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, can_driver);

    if ((!isobus::CANHardwareInterface::start()) || (!can_driver->get_is_valid())) {
        std::cout << "❌ Failed to start CAN interface!\n";
        std::cout << "   Make sure:\n";
        std::cout << "   1. can0 interface exists: ip link show can0\n";
        std::cout << "   2. can0 is up: sudo ip link set can0 up type can bitrate 250000\n";
        return -1;
    }

    std::cout << "✓ CAN interface started successfully\n\n";

    std::signal(SIGINT, signal_handler);

    std::cout << "[3/9] Creating ISO NAME...\n";

    isobus::NAME my_name(0);

    my_name.set_arbitrary_address_capable(true); // We can change address if conflict
    my_name.set_industry_group(2);               // Agriculture and Forestry
    my_name.set_device_class(6);                 // Rate Control
    my_name.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
    my_name.set_identity_number(42);      // Our unique serial number
    my_name.set_ecu_instance(0);          // First ECU
    my_name.set_function_instance(0);     // First function instance
    my_name.set_device_class_instance(0); // First device class instance
    my_name.set_manufacturer_code(1407);  // Open-Agriculture manufacturer code

    std::cout << "✓ ISO NAME created: 0x" << std::hex << my_name.get_full_name() << std::dec << "\n\n";

    std::cout << "[4/9] Creating Internal Control Function (our ECU)...\n";

    auto my_ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(my_name, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    std::cout << "✓ Internal CF created and claimed address: 0x" << std::hex << static_cast<int>(my_ecu->get_address())
              << std::dec << "\n\n";

    std::cout << "[5/9] Creating Partnered Control Function (TC Server finder)...\n";

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

    std::cout << "✓ Partnered CF created (will auto-connect when TC appears)\n\n";

    std::cout << "[6/9] Creating Task Controller Client...\n";

    auto tc = std::make_shared<isobus::TaskControllerClient>(tc_partner, // TC server (auto-discovered)
                                                             my_ecu,     // Our ECU
                                                             nullptr);   // No VT for this example

    std::cout << "✓ TC Client created\n\n";

    std::cout << "[7/9] Creating Device Descriptor Object Pool (DDOP)...\n";

    auto ddop = std::make_shared<isobus::DeviceDescriptorObjectPool>();

    if (!create_simple_ddop(ddop, my_ecu->get_NAME())) {
        std::cout << "❌ Failed to create DDOP!\n";
        return -1;
    }

    std::cout << "✓ DDOP created with " << ddop->size() << " objects\n\n";

    std::cout << "[8/9] Setting up callbacks...\n";

    tc->add_request_value_callback(
        [](std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t &outValue, void *parent) -> bool {
            std::cout << "   [->CALLBACK] TC requests value:\n";
            std::cout << "              Element #" << elementNumber << ", DDI=0x" << std::hex << ddi << std::dec
                      << "\n";

            switch (ddi) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
                outValue = current_work_state.load();
                std::cout << "              → Device Work State: " << (outValue ? "Working (1)" : "Not Working (0)")
                          << "\n";
                return true;

            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth):
                outValue = gnss_auth_status.load() ? 12000 : 0; // 12m when authenticated
                std::cout << "              → Boom Width: " << outValue << " mm\n";
                return true;

            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16): {
                // Both sections controlled by PHTG auth: bit 0-1 = section 0, bit 2-3 = section 1
                int auth = gnss_auth_status.load();
                if (auth) {
                    outValue = 0x05; // 0b00000101 = both sections ON
                } else {
                    outValue = 0x00; // both sections OFF
                }
                std::cout << "              → Sections: " << (auth ? "BOTH ON" : "BOTH OFF") << "\n";
                return true;
            }

            default:
                std::cout << "              → Not supported\n";
                outValue = 0;
                return false;
            }
        },
        nullptr); // No parent pointer needed

    tc->add_value_command_callback(
        [](std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t processVariableValue, void *parent) -> bool {
            std::cout << "   [CALLBACK] TC sends command:\n";
            std::cout << "              Element #" << elementNumber << ", DDI=0x" << std::hex << ddi << std::dec
                      << ", Value=" << processVariableValue << "\n";

            switch (ddi) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState):
                std::cout << "              → Work State command: " << (processVariableValue ? "ON" : "OFF") << "\n";
                return true;

            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16):
                std::cout << "              → Section control (ignored - PHTG controlled)\n";
                return true;

            default:
                std::cout << "              → Command not supported\n";
                return false;
            }
        },
        nullptr); // No parent pointer

    std::cout << "✓ Callbacks registered\n\n";

    std::cout << "[9/9] Configuring and initializing TC Client...\n";

    tc->configure(ddop,  // Our DDOP
                  1,     // Boot time: 1 second
                  2,     // 2 sections
                  0,     // Localization pool: unlimited
                  true,  // Report to TC: YES
                  false, // TC-GEO: NO
                  false, // Documentation: NO
                  false, // TC-BBS-A: NO
                  true); // TC-BBS-B: YES (boom section control)

    tc->initialize(true); // Use internal thread for automatic operation

    std::cout << "✓ TC Client initialized\n\n";

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                  TC CLIENT IS RUNNING                        ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "The CAN stack is running in background threads.\n";
    std::cout << "Watch above for TC communication events.\n";
    std::cout << "Press Ctrl+C to exit cleanly.\n\n";

    while (running) {
        static int counter = 0;
        if (counter % 10 == 0) {
            std::cout << "────────────────────────────────────────────────────────\n";
            std::cout << "[Status] TC Client Running\n";

            if (tc_partner->get_address_valid()) {
                std::cout << "         ✓ TC Server FOUND at address 0x" << std::hex
                          << static_cast<int>(tc_partner->get_address()) << std::dec << "\n";
                std::cout << "         ✓ Communication active\n";
            } else {
                std::cout << "         ⏳ Waiting for TC Server to appear...\n";
                std::cout << "         (Make sure TC is on the bus and powered)\n";
            }
            std::cout << "────────────────────────────────────────────────────────\n\n";
        }
        counter++;

        if (counter % 5 == 0) {
            current_work_state.store(1 - current_work_state.load());
            std::cout << "🔄 [AUTO-TOGGLE] Work state changed to: "
                      << (current_work_state.load() ? "WORKING" : "NOT WORKING") << "\n\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "\n╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    SHUTTING DOWN                             ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Stopping Serial interface...\n";
    nmea_serial->stop();

    std::cout << "Terminating TC client...\n";
    tc->terminate();

    std::cout << "Stopping CAN interface...\n";
    isobus::CANHardwareInterface::stop();

    std::cout << "\n✓ Shutdown complete. Goodbye!\n\n";
    return 0;
}
