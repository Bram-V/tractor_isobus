/*******************************************************************************
 * ISOBUS TASK CONTROLLER CLIENT EXAMPLE
 *******************************************************************************
 *
 * This example demonstrates how to create a Task Controller (TC) CLIENT.
 * A TC CLIENT represents an IMPLEMENT (like a sprayer, planter, etc.) that
 * communicates with a TC SERVER (typically on a tractor terminal).
 *
 * ARCHITECTURE OVERVIEW:
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *     ┌─────────────────────┐         CAN Bus         ┌─────────────────────┐
 *     │   TC CLIENT         │◄──────────────────────►│   TC SERVER         │
 *     │   (This Code)       │                         │   (Tractor/VT)      │
 *     │                     │                         │                     │
 *     │  - Implement ECU    │   1) Address Claim      │  - Task Controller  │
 *     │  - Rate Controller  │   2) Partner Discovery  │  - Farm Management  │
 *     │  - DDOP Provider    │   3) DDOP Upload        │  - Data Logging     │
 *     │  - Data Reporter    │   4) Process Data       │  - Section Control  │
 *     └─────────────────────┘   5) Commands/Values    └─────────────────────┘
 *
 *
 * COMMUNICATION FLOW:
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   CLIENT                                SERVER
 *     │                                     │
 *     ├──[1] Address Claim (NAME)──────────►
 *     │                                     │
 *     ◄─────[2] TC Server Announces────────┤
 *     │        (Partner Discovery)          │
 *     │                                     │
 *     ├──[3] Working Set Master────────────►
 *     │      (I want to connect!)           │
 *     │                                     │
 *     ◄─────[4] Request DDOP───────────────┤
 *     │                                     │
 *     ├──[5] Upload DDOP────────────────────►
 *     │      (Device Structure)             │
 *     │                                     │
 *     ◄─────[6] Request Process Data───────┤
 *     │                                     │
 *     ├──[7] Report Values──────────────────►
 *     │      (Work State, Width, Rate)      │
 *     │                                     │
 *     ◄─────[8] Set Commands───────────────┤
 *     │      (Turn On/Off, Set Rate)        │
 *     │                                     │
 *     ├──[9] Confirm & Report───────────────►
 *     │      (Continuous Operation)         │
 *     └─────────────────────────────────────┘
 *
 ******************************************************************************/

#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"

#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

#include <echo/echo.hpp>
#include <echo/format.hpp>
#include <echo/widget.hpp>

// Signal handler for clean shutdown (Ctrl+C)
static std::atomic_bool running = true;
// Current work state: 0 = not working, 1 = working
static std::atomic<std::int32_t> current_work_state = 0;
// Auto mode: true = TC controls sections, false = manual control
static std::atomic_bool is_auto_mode = true;
// Section control state: 0 = manual, 1 = auto
static std::atomic<std::int32_t> section_control_state = 1;
// Hashtag auth result: 0 = auth failed/invalid, 1 = auth success/valid
static std::atomic<std::int32_t> hashtag_auth_result = 0;
// Data sending frequency in milliseconds (how often to report/update)
static std::atomic<std::uint32_t> send_frequency_ms = 1000;

void signal_handler(int) { running = false; }

static constexpr std::uint16_t DDI_AUTH_RESULT = 65432;

bool export_ddop_to_xml(std::shared_ptr<isobus::DeviceDescriptorObjectPool> ddop, const std::string &filename) {
    if (!ddop) {
        std::cerr << "Error: DDOP is null\n";
        return false;
    }

    std::string xmlContent;
    if (!ddop->generate_task_data_iso_xml(xmlContent)) {
        std::cerr << "Error: Failed to generate ISOXML from DDOP\n";
        return false;
    }

    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing\n";
        return false;
    }

    outFile << xmlContent;
    outFile.close();

    std::cout << "DDOP exported successfully to " << filename << "\n";
    return true;
}

/*******************************************************************************
 * DDOP OBJECT IDs
 *******************************************************************************
 *
 * The Device Descriptor Object Pool (DDOP) describes the structure of your
 * implement. Each object in the DDOP needs a unique ID. Think of these as
 * "handles" or "names" for different parts of your device.
 *
 * DDOP HIERARCHY VISUALIZED:
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *                           Device (0)
 *                              │
 *                 ┌────────────┼────────────┐
 *                 │            │            │
 *          MainElement(1)  Connector(3)  Implement(6)
 *                 │            │            │
 *         WorkState(2)  ├──X-Offset(4) WorkingWidth(8)
 *                       └──Y-Offset(5)
 *
 * Additional Objects:
 *   - Presentations (100, 101): Define units (mm, m², etc.)
 *
 * OBJECT TYPES:
 *   - Device:         Root object containing device info
 *   - DeviceElement:  Physical/logical parts (connector, boom, etc.)
 *   - ProcessData:    Measurable values (width, rate, state)
 *   - Property:       Fixed characteristics (offsets, types)
 *   - Presentation:   How to display values (units, decimals)
 *
 ******************************************************************************/
enum class DDOPObjectIDs : std::uint16_t {
    // Root device object (ID 0 is reserved for the device itself)
    Device = 0,

    // Device Elements (physical/logical parts)
    MainDeviceElement = 1, // The main implement
    Connector = 3,         // Hitch/attachment point
    Implement = 6,         // Working function (boom/tool)

    // Process Data (dynamic values that change during operation)
    DeviceActualWorkState = 2, // Is device working? (0=off, 1=on)
    ConnectorXOffset = 4,      // Fore/aft position from reference
    ConnectorYOffset = 5,      // Left/right position from reference
    ImplementWorkingWidth = 8, // Active working width in mm

    HashtagAuthResult = 10, // Auth result from hashtag sensor

    // Presentations (how to display values)
    AreaPresentation = 100,  // For area values (m²)
    WidthPresentation = 101, // For width/distance (mm)
    RawPresentation = 102,   // For raw values (no formatting)
};

/*******************************************************************************
 * FUNCTION: create_simple_ddop()
 *******************************************************************************
 *
 * Creates a Device Descriptor Object Pool (DDOP) that describes our implement
 * to the Task Controller. The DDOP is like a blueprint or schema that tells
 * the TC:
 *   - What kind of device we are
 *   - What physical parts we have (elements)
 *   - What data we can provide (process data)
 *   - How to display that data (presentations)
 *
 * PARAMETERS:
 *   ddop       - Shared pointer to the DDOP object to populate
 *   clientName - ISO NAME of our device (64-bit identifier)
 *
 * RETURNS:
 *   true if DDOP was created successfully, false otherwise
 *
 * DDOP STRUCTURE FOR THIS EXAMPLE:
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *   Device Object
 *   └─ "SimpleImplement" v1.0.0
 *      │
 *      ├─ MainDeviceElement (Type: Device)
 *      │  └─ DeviceActualWorkState (DDI: ActualWorkState)
 *      │     → Reports if device is working or not
 *      │
 *      ├─ Connector (Type: Connector)
 *      │  ├─ ConnectorXOffset (DDI: DeviceElementOffsetX)
 *      │  │  → Where connector is (fore/aft) in mm
 *      │  └─ ConnectorYOffset (DDI: DeviceElementOffsetY)
 *      │     → Where connector is (left/right) in mm
 *      │
 *      └─ Implement (Type: Function)
 *         └─ ImplementWorkingWidth (DDI: ActualWorkingWidth)
 *            → How wide is the working area in mm
 *
 ******************************************************************************/
bool create_simple_ddop(std::shared_ptr<isobus::DeviceDescriptorObjectPool> ddop, isobus::NAME clientName) {
    bool success = true;

    // Clear any existing objects
    ddop->clear();

    //==========================================================================
    // LOCALIZATION DATA
    //==========================================================================
    // This 7-byte array defines language and formatting preferences:
    //   Byte 0-1: Language code ('e', 'n' = English)
    //   Byte 2:   0x50 = Decimal point format, 24hr time
    //   Byte 3:   0x00 = Date format (DDMMYYYY)
    //   Byte 4:   0x55 = Units: metric length, metric area
    //   Byte 5:   0x55 = Units: metric mass, metric temperature
    //   Byte 6:   0xFF = Reserved
    std::array<std::uint8_t, 7> localizationData = {'e', 'n', 0x50, 0x00, 0x55, 0x55, 0xFF};

    //==========================================================================
    // ADD DEVICE OBJECT (ID 0 - Root)
    //==========================================================================
    // The device object is the root of the DDOP tree and contains metadata
    // about the entire implement.
    //
    // Parameters:
    //   - designator:        "SimpleImplement" (device name)
    //   - softwareVersion:   "1.0.0"
    //   - serialNumber:      "001"
    //   - structureLabel:    "SMP1.0" (unique identifier for this DDOP structure)
    //   - localizationLabel: (array defined above)
    //   - extendedStructure: (empty - used for binary DDOP data)
    //   - clientIsoName:     Our 64-bit ISO NAME
    success &= ddop->add_device("HASHTAG", "1.0.11", "001", "SMP1.0", localizationData, std::vector<std::uint8_t>(),
                                clientName.get_full_name());

    //==========================================================================
    // ADD MAIN DEVICE ELEMENT (ID 1)
    //==========================================================================
    // This represents the main implement itself. Every DDOP must have at least
    // one device element.
    //
    // Parameters:
    //   - designator:        "Implement" (name shown to operator)
    //   - elementNumber:     0 (first element, used in messages)
    //   - parentObject:      0 (parent is Device object)
    //   - type:              Device (this is the main device)
    //   - objectId:          1 (unique ID for this element)
    success &=
        ddop->add_device_element("Implement", 0, 0, isobus::task_controller_object::DeviceElementObject::Type::Device,
                                 static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement));

    //==========================================================================
    // ADD WORK STATE PROCESS DATA (ID 2)
    //==========================================================================
    // This process data object reports whether the device is working or not.
    // The TC can request this value and we respond via callbacks.
    //
    // Parameters:
    //   - designator:     "Actual Work State"
    //   - ddi:            ActualWorkState (standard DDI from ISO 11783-11)
    //   - presentation:   NULL_OBJECT_ID (no special formatting)
    //   - properties:     MemberOfDefaultSet (TC will request this automatically)
    //   - triggerMethods: OnChange (report when value changes)
    //   - objectId:       2 (unique ID)
    //
    // WORK STATE VALUES:
    //   0 = Not working / Off
    //   1 = Working / On
    success &= ddop->add_device_process_data(
        "Actual Work State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState),
        isobus::NULL_OBJECT_ID,
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
        static_cast<std::uint16_t>(DDOPObjectIDs::DeviceActualWorkState));

    //==========================================================================
    // ADD CONNECTOR ELEMENT (ID 3)
    //==========================================================================
    // The connector represents the physical attachment point (hitch) where
    // the implement connects to the tractor. This is important for TC-GEO
    // applications to calculate positions accurately.
    //
    // Parameters:
    //   - designator:    "Connector"
    //   - elementNumber: 1 (second element)
    //   - parentObject:  1 (parent is MainDeviceElement)
    //   - type:          Connector (special type for hitches)
    //   - objectId:      3 (unique ID)
    success &= ddop->add_device_element("Connector", 1, static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement),
                                        isobus::task_controller_object::DeviceElementObject::Type::Connector,
                                        static_cast<std::uint16_t>(DDOPObjectIDs::Connector));

    //==========================================================================
    // ADD CONNECTOR X OFFSET (ID 4)
    //==========================================================================
    // This describes the fore/aft position of the connector relative to a
    // reference point (usually the rear axle of the tractor).
    //
    // X-AXIS: Positive = Forward, Negative = Backward
    //
    // Parameters:
    //   - designator:     "Connector X"
    //   - ddi:            DeviceElementOffsetX
    //   - presentation:   WidthPresentation (display in mm)
    //   - properties:     Settable (TC can set this value)
    //   - triggerMethods: 0 (no automatic reporting)
    //   - objectId:       4
    success &= ddop->add_device_process_data(
        "Connector X", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable), 0,
        static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorXOffset));

    //==========================================================================
    // ADD CONNECTOR Y OFFSET (ID 5)
    //==========================================================================
    // This describes the left/right position of the connector.
    //
    // Y-AXIS: Positive = Right, Negative = Left (driver's perspective)
    //
    // Parameters: Similar to ConnectorXOffset above
    success &= ddop->add_device_process_data(
        "Connector Y", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable), 0,
        static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorYOffset));

    //==========================================================================
    // ADD IMPLEMENT FUNCTION ELEMENT (ID 6)
    //==========================================================================
    // This represents the working function of the implement (e.g., boom, tool,
    // applicator). This is where the actual work happens.
    //
    // Parameters:
    //   - designator:    "Function"
    //   - elementNumber: 2 (third element)
    //   - parentObject:  1 (parent is MainDeviceElement)
    //   - type:          Function (working function)
    //   - objectId:      6
    success &= ddop->add_device_element("Function", 2, static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement),
                                        isobus::task_controller_object::DeviceElementObject::Type::Function,
                                        static_cast<std::uint16_t>(DDOPObjectIDs::Implement));

    //==========================================================================
    // ADD WORKING WIDTH PROCESS DATA (ID 8)
    //==========================================================================
    // This reports the current active working width of the implement in
    // millimeters. For a sprayer, this would be the boom width. For a planter,
    // the total width of all rows.
    //
    // Parameters:
    //   - designator:     "Working Width"
    //   - ddi:            ActualWorkingWidth (DDI 0x0046)
    //   - presentation:   WidthPresentation (show in mm)
    //   - properties:     MemberOfDefaultSet (report automatically)
    //   - triggerMethods: OnChange (report when width changes)
    //   - objectId:       8
    //
    // NOTE: Value is in millimeters (e.g., 3000 = 3 meters)
    success &= ddop->add_device_process_data(
        "Working Width", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth),
        static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
        static_cast<std::uint8_t>(
            isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval),
        static_cast<std::uint16_t>(DDOPObjectIDs::ImplementWorkingWidth));

    // =========================================================================
    // ADD HASHTAG SENSOR PROCESS DATA (ID 10)
    // =========================================================================
    // This process data object reports whether the device is working or not.
    // The TC can request this value and we respond via callbacks.
    //
    // Parameters:
    //   - designator:     "Hashtag Auth Result"
    //   - ddi:            HashtagAuthResult (standard DDI from ISO 11783-11)
    //   - presentation:   NULL_OBJECT_ID (no special formatting)
    //   - properties:     MemberOfDefaultSet (TC will request this automatically)
    //   - triggerMethods: OnChange (report when value changes)
    //   - objectId:       10 (unique ID)
    //
    // AUTH RESULT VALUES:
    //   0 = Not working / Off
    //   1 = Working / On
    success &= ddop->add_device_process_data("Hashtag DDI #1", DDI_AUTH_RESULT,
                                             static_cast<std::uint16_t>(DDOPObjectIDs::RawPresentation),
                                             static_cast<std::uint8_t>(3), static_cast<std::uint8_t>(9),
                                             static_cast<std::uint16_t>(DDOPObjectIDs::HashtagAuthResult));

    //==========================================================================
    // ADD PRESENTATIONS
    //==========================================================================
    // Presentations define how values should be displayed to the operator.
    // They specify units, decimal places, and scale factors.
    //
    // ADD WIDTH PRESENTATION (ID 101):
    //   - unit:       "mm" (millimeters)
    //   - offset:     0
    //   - scale:      1.0 (no scaling)
    //   - decimals:   0 (show whole numbers)
    //   - objectId:   101
    success &= ddop->add_device_value_presentation("mm", 0, 1.0f, 0,
                                                   static_cast<std::uint16_t>(DDOPObjectIDs::WidthPresentation));

    // ADD AREA PRESENTATION (ID 100):
    //   - unit:       "m^2" (square meters)
    //   - offset:     0
    //   - scale:      1.0
    //   - decimals:   0
    //   - objectId:   100
    success &= ddop->add_device_value_presentation("m^2", 0, 1.0f, 0,
                                                   static_cast<std::uint16_t>(DDOPObjectIDs::AreaPresentation));

    // ADD RAW PRESENTATION (ID 102):
    //   - unit:       "raw" (no formatting)
    //   - offset:     0
    //   - scale:      1.0
    //   - decimals:   0
    //   - objectId:   102
    success &= ddop->add_device_value_presentation("raw", 0, 1.0f, 0,
                                                   static_cast<std::uint16_t>(DDOPObjectIDs::RawPresentation));

    //==========================================================================
    // LINK CHILD OBJECTS TO PARENTS
    //==========================================================================
    // This step creates the parent-child relationships in the DDOP tree.
    // Each DeviceElement needs to reference its child ProcessData objects.
    //
    // WHY? The TC needs to know which data belongs to which element. For
    // example, "WorkingWidth" belongs to the "Implement" element.
    //
    // RELATIONSHIP TREE:
    //
    //   MainElement (ID 1)
    //   └─ DeviceActualWorkState (ID 2)
    //
    //   Connector (ID 3)
    //   ├─ ConnectorXOffset (ID 4)
    //   └─ ConnectorYOffset (ID 5)
    //
    //   Implement (ID 6)
    //   └─ ImplementWorkingWidth (ID 8)
    //
    if (success) {
        // Get pointers to the device elements
        auto mainElement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::MainDeviceElement)));
        auto connector = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::Connector)));
        auto implement = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
            ddop->get_object_by_id(static_cast<std::uint16_t>(DDOPObjectIDs::Implement)));

        // Link children to MainElement
        mainElement->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::DeviceActualWorkState));

        // Link children to MainElement
38448 .rw-rw-r--      1  465      4.1k bresilla 17 Dec  2025   -N devbox.json
21639092 .rw-rw-r--      1  935      4.1k bresilla 16 Jan 16:21   -N hashtag.xml
21660637 .rw-rw-r--      1  636      4.1k bresilla 20 Jan 13:53   -N hashtag_fromcode.xml
21660599 .rw-rw-r--      1  188      4.1k bresilla 16 Jan 11:13   -N imgui.ini
21638445 .rw-rw-r--      1 5.3k      8.2k bresilla 17 Dec  2025   -N Makefile
21638446 .rw-rw-r--      1    0         0 bresilla 17 Dec  2025   -N README.md
21660700 .rw-rw-r--      1  691      4.1k bresilla 23 Jan 12:25   -N tag_fromcode.xml
21638465 .rw-rw-r--      1 6.8k      8.2k bresilla  5 Jan 15:47   -N xmake.lua

═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════════[ 1 ]══

 // ceol   bresilla ▓| 1 |                                                                                        |  main ?  tractor

        mainElement->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::HashtagAuthResult));

        // Link children to Connector
        connector->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorXOffset));
        connector->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ConnectorYOffset));

        // Link children to Implement
        implement->add_reference_to_child_object(static_cast<std::uint16_t>(DDOPObjectIDs::ImplementWorkingWidth));
    }

    return success;
}

/*******************************************************************************
 * MAIN FUNCTION
 ******************************************************************************/
int main() {

    //==========================================================================
    // STEP 1: CAN BUS SETUP
    //==========================================================================
    // Initialize the CAN hardware interface. This sets up communication with
    // the physical CAN bus hardware (in this case, SocketCAN on Linux).
    //
    // SocketCAN is a Linux kernel module that provides CAN bus access through
    // standard network interfaces (like can0, can1, etc.)
    //
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║       ISOBUS TASK CONTROLLER CLIENT EXAMPLE                  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "[1/8] Initializing CAN interface...\n";

    // Create CAN driver for "can0" interface
    auto can_driver = std::make_shared<isobus::SocketCANInterface>("can0");

    // Tell the hardware interface we have 1 CAN channel
    isobus::CANHardwareInterface::set_number_of_can_channels(1);

    // Assign our CAN driver to channel 0
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, can_driver);

    // Start the CAN interface
    if ((!isobus::CANHardwareInterface::start()) || (!can_driver->get_is_valid())) {
        std::cout << "❌ Failed to start CAN interface!\n";
        std::cout << "   Make sure:\n";
        std::cout << "   1. can0 interface exists: ip link show can0\n";
        std::cout << "   2. can0 is up: sudo ip link set can0 up type can bitrate 250000\n";
        return -1;
    }

    std::cout << "✓ CAN interface started successfully\n\n";

    // Setup signal handler for graceful shutdown
    std::signal(SIGINT, signal_handler);

    //==========================================================================
    // STEP 2: CREATE ISO NAME
    //==========================================================================
    // The ISO NAME is a 64-bit identifier that uniquely identifies our ECU
    // on the ISOBUS network. It's used during the address claim process.
    //
    // ISO NAME STRUCTURE (64 bits):
    // ┌──────────────────────────────────────────────────────────────────┐
    // │ Bit │ Field                    │ Description                      │
    // ├─────┼──────────────────────────┼──────────────────────────────────┤
    // │ 0   │ Arbitrary Address Capable│ Can we change our address?       │
    // │ 1-3 │ Industry Group           │ 2 = Agriculture/Forestry         │
    // │ 4-7 │ Device Class Instance    │ Instance of this device class    │
    // │ 8-11│ Device Class             │ 6 = Rate Control                 │
    // │12-15│ Reserved                 │ Always 0                         │
    // │16-23│ Function                 │ 128 = RateControl                │
    // │24-27│ Function Instance        │ Which instance of this function  │
    // │28-30│ ECU Instance             │ Which ECU on this device         │
    // │31-42│ Manufacturer Code        │ 1407 = Open-Agriculture          │
    // │43-63│ Identity Number          │ Serial number (42 in this case)  │
    // └──────────────────────────────────────────────────────────────────┘
    //
    std::cout << "[2/8] Creating ISO NAME...\n";

    isobus::NAME my_name(0);

    // Set NAME fields according to ISO 11783-5
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

    //==========================================================================
    // STEP 3: CREATE INTERNAL CONTROL FUNCTION
    //==========================================================================
    // An Internal Control Function (ICF) represents OUR ECU on the network.
    // It handles address claiming and maintains our presence on the bus.
    //
    // WHAT IT DOES:
    //   - Participates in address claim process (ISO 11783-5)
    //   - Claims a unique address (0-253)
    //   - Defends our address against conflicts
    //   - Allows us to send/receive messages
    //
    std::cout << "[3/8] Creating Internal Control Function (our ECU)...\n";

    auto my_ecu = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(my_name, 0);

    // Wait for address claim to complete
    // The address claim process involves:
    //   1. Requesting an address
    //   2. Sending "cannot claim" if waiting
    //   3. Claiming address after delay
    //   4. Defending against conflicts
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    std::cout << "✓ Internal CF created and claimed address: 0x" << std::hex << static_cast<int>(my_ecu->get_address())
              << std::dec << "\n\n";

    //==========================================================================
    // STEP 4: CREATE PARTNERED CONTROL FUNCTION FOR TC SERVER
    //==========================================================================
    // A Partnered Control Function (PCF) represents another ECU we want to
    // talk to - in this case, the Task Controller SERVER.
    //
    // We use NAME FILTERS to automatically find the TC server when it appears
    // on the bus. Think of it like a query: "Find me an ECU that matches these
    // characteristics"
    //
    // NAME FILTER EXPLANATION:
    // ┌────────────────────────────────────────────────────────────────────┐
    // │ Filter                  │ Value │ Meaning                          │
    // ├─────────────────────────┼───────┼──────────────────────────────────┤
    // │ FunctionCode            │  25   │ Must be Task Controller          │
    // │ FunctionInstance        │   0   │ First TC (instance 0)            │
    // │ IndustryGroup           │   2   │ Agriculture/Forestry             │
    // │ DeviceClass             │   0   │ Non-Specific (typical for TC)    │
    // └────────────────────────────────────────────────────────────────────┘
    //
    // When an ECU with matching NAME appears, our PCF becomes "valid" and
    // we can start communicating with it.
    //
    std::cout << "[4/8] Creating Partnered Control Function (TC Server finder)...\n";

    // Filter 1: Must have Function Code = TaskController (25)
    const isobus::NAMEFilter filterTaskController(isobus::NAME::NAMEParameters::FunctionCode,
                                                  static_cast<std::uint8_t>(isobus::NAME::Function::TaskController));

    // Filter 2: Must be instance 0 (first TC)
    const isobus::NAMEFilter filterTaskControllerInstance(isobus::NAME::NAMEParameters::FunctionInstance, 0);

    // Filter 3: Must be Agriculture/Forestry industry
    const isobus::NAMEFilter filterTaskControllerIndustryGroup(
        isobus::NAME::NAMEParameters::IndustryGroup,
        static_cast<std::uint8_t>(isobus::NAME::IndustryGroup::AgriculturalAndForestryEquipment));

    // Filter 4: Device class should be Non-Specific
    const isobus::NAMEFilter filterTaskControllerDeviceClass(
        isobus::NAME::NAMEParameters::DeviceClass, static_cast<std::uint8_t>(isobus::NAME::DeviceClass::NonSpecific));

    // Combine all filters
    const std::vector<isobus::NAMEFilter> tcNameFilters = {filterTaskController, filterTaskControllerInstance,
                                                           filterTaskControllerIndustryGroup,
                                                           filterTaskControllerDeviceClass};

    // Create partnered CF that automatically finds matching TC
    auto tc_partner = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, tcNameFilters);

    std::cout << "✓ Partnered CF created (will auto-connect when TC appears)\n\n";

    //==========================================================================
    // STEP 5: CREATE TASK CONTROLLER CLIENT
    //==========================================================================
    // The TaskControllerClient manages all TC client operations:
    //   - Connection management
    //   - DDOP upload
    //   - Process data handling
    //   - Command processing
    //   - State machine management
    //
    // CONSTRUCTOR PARAMETERS:
    //   - partner:   The TC server we'll talk to (can be nullptr initially)
    //   - clientECU: Our Internal Control Function
    //   - primaryVT: Optional Virtual Terminal (nullptr if no VT)
    //
    std::cout << "[5/8] Creating Task Controller Client...\n";

    auto tc = std::make_shared<isobus::TaskControllerClient>(tc_partner, // TC server (auto-discovered)
                                                             my_ecu,     // Our ECU
                                                             nullptr);   // No VT for this example

    std::cout << "✓ TC Client created\n\n";

    //==========================================================================
    // STEP 6: CREATE DEVICE DESCRIPTOR OBJECT POOL (DDOP)
    //==========================================================================
    // The DDOP is a data structure that describes our implement to the TC.
    // It's like an XML schema or database schema that defines:
    //   - What our device is
    //   - What parts it has
    //   - What data we can provide
    //   - How that data should be displayed
    //
    // The TC needs this information to:
    //   - Understand our capabilities
    //   - Request appropriate data
    //   - Log data correctly
    //   - Display information to the operator
    //
    std::cout << "[6/8] Creating Device Descriptor Object Pool (DDOP)...\n";

    auto ddop = std::make_shared<isobus::DeviceDescriptorObjectPool>();

    if (!create_simple_ddop(ddop, my_ecu->get_NAME())) {
        std::cout << "❌ Failed to create DDOP!\n";
        return -1;
    }

    std::string filenameee = "hashtag_oroginal.xml";
    const char *xml_export_filename = filenameee.c_str();
    export_ddop_to_xml(ddop, xml_export_filename);

    std::cout << "✓ DDOP created with " << ddop->size() << " objects\n\n";

    //==========================================================================
    // STEP 7: SETUP CALLBACKS
    //==========================================================================
    // Callbacks are functions that get called when the TC requests data or
    // sends commands. Think of them as event handlers.
    //
    // TWO MAIN CALLBACK TYPES:
    //
    // 1. REQUEST VALUE CALLBACK:
    //    Called when TC asks: "What is your current working width?"
    //    We must respond with the requested value.
    //
    // 2. VALUE COMMAND CALLBACK:
    //    Called when TC commands: "Turn on section 3"
    //    We must execute the command and respond with success/failure.
    //
    std::cout << "[7/8] Setting up callbacks...\n";

    //--------------------------------------------------------------------------
    // REQUEST VALUE CALLBACK - DATA REPORTER/READER
    //--------------------------------------------------------------------------
    // CALLBACK SIGNATURE:
    //   bool callback(uint16_t elementNumber,    // Which element (0, 1, 2...)
    //                 uint16_t ddi,               // Which DDI (data type)
    //                 int32_t& outValue,          // Output: value to return
    //                 void* parentPointer)        // Optional context pointer
    //
    // PRACTICAL USAGE: This callback handles data REQUESTS FROM the Task Controller.
    // Think of it as a dashboard or sensor display:
    //   - TC asks: "What's your current working width?" → You respond: "3000mm"
    //   - TC asks: "Are you currently working?" → You respond: current_work_state
    //   - TC asks: "What's your section control state?" → You respond: section_control_state
    //
    // This is READ-ONLY - you simply report current sensor values or status.
    // Common use cases: temperature sensors, position data, current speed, operational state.
    //
    // PARAMETERS EXPLAINED:
    //   elementNumber: The device element being queried (from DDOP)
    //                  Element 0 = MainDeviceElement
    //                  Element 1 = Connector
    //                  Element 2 = Implement
    //
    //   ddi: Data Description Index - defines what data is requested
    //        Example DDIs:
    //          0x00A1 = ActualWorkState (is device on/off?)
    //          0x0046 = ActualWorkingWidth (how wide?)
    //          0x0049 = SetpointWorkState (desired on/off state)
    //
    //   outValue: Reference where we store the requested value
    //             Must be set before returning true
    //
    //   parentPointer: Optional pointer to context/class instance
    //                  nullptr in this example (using lambda capture instead)
    //
    // RETURN VALUE:
    //   true  = We handled this request, outValue is valid
    //   false = We don't have this data, try next callback
    //
    // PRACTICAL USAGE: This callback handles data REQUESTS FROM the Task Controller.
    // Think of it as a dashboard or sensor display:
    //   - TC asks: "What's your current working width?" → You respond: "3000mm"
    //   - TC asks: "Are you currently working?" → You respond: current_work_state
    //   - TC asks: "What's your section control state?" → You respond: section_control_state
    //
    // This is READ-ONLY - you simply report current sensor values or status.
    // Common use cases: temperature sensors, position data, current speed, operational state.
    //
    // PARAMETERS EXPLAINED:
    //   elementNumber: The device element being queried (from DDOP)
    //                  Element 0 = MainDeviceElement
    //                  Element 1 = Connector
    //                  Element 2 = Implement
    //
    //   ddi: Data Description Index - defines what data is requested
    //        Example DDIs:
    //          0x00A1 = ActualWorkState (is device on/off?)
    //          0x0046 = ActualWorkingWidth (how wide?)
    //          0x0049 = SetpointWorkState (desired on/off state)
    //
    //   outValue: Reference where we store the requested value
    //             Must be set before returning true
    //
    //   parentPointer: Optional pointer to context/class instance
    //                  nullptr in this example (using lambda capture instead)
    //
    // RETURN VALUE:
    //   true  = We handled this request, outValue is valid
    //   false = We don't have this data, try next callback
    //
    tc->add_request_value_callback(
        [](std::uint16_t, std::uint16_t ddi, std::int32_t &outValue, void *) -> bool {
            switch (ddi) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
                outValue = current_work_state.load();
                return true;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth):
                outValue = 3000;
                return true;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
                outValue = section_control_state.load();
                return true;
            case DDI_AUTH_RESULT:
                // Report the current hashtag authentication result
                // 0 = auth failed/invalid hashtag detected
                // 1 = auth success/valid hashtag detected
                outValue = hashtag_auth_result.load();
                return true;
            default:
                outValue = 0;
                return false;
            }
        },
        nullptr);

    //--------------------------------------------------------------------------
    // VALUE COMMAND CALLBACK - COMMAND EXECUTOR/ACTUATOR
    //--------------------------------------------------------------------------
    // CALLBACK SIGNATURE:
    //   bool callback(uint16_t elementNumber,         // Which element
    //                 uint16_t ddi,                    // Which DDI (command type)
    //                 int32_t processVariableValue,    // Commanded value
    //                 void* parentPointer)             // Optional context
    //
    // PRACTICAL USAGE: This callback handles commands FROM the Task Controller.
    // Think of it as actuators or control systems:
    //   - TC commands: "Turn on!" → You execute: current_work_state = ON
    //   - TC commands: "Set rate to 50%" → You execute: application_rate = 50
    //   - TC commands: "Enable auto mode!" → You execute: is_auto_mode = true
    //
    // This is WRITE OPERATION - you change device state based on TC commands.
    // Common use cases: turning sections on/off, setting application rates, changing operational modes.
    //
    // PARAMETERS EXPLAINED:
    //   elementNumber: Which element to command (0, 1, 2...)
    //
    //   ddi: The type of command
    //        Example DDIs:
    //          0x0049 = SetpointWorkState (turn on/off)
    //          0x003E = SetpointRate (set application rate)
    //          0x006E = SetpointCondensedWorkState (section control)
    //
    //   processVariableValue: The commanded value
    //                         For WorkState: 0=off, 1=on
    //                         For Rate: value in DDI units
    //
    //   parentPointer: Optional context (nullptr here)
    //
    // RETURN VALUE:
    //   true  = Command accepted and executed
    //   false = Command rejected or not supported
    //
    tc->add_value_command_callback(
        [](std::uint16_t, std::uint16_t ddi, std::int32_t processVariableValue, void *) -> bool {
            switch (ddi) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState):
                current_work_state.store(processVariableValue);
                return true;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::PrescriptionControlState):
                section_control_state.store(processVariableValue != 0 ? 1 : 0);
                is_auto_mode.store(processVariableValue != 0);
                return true;
            default:
                return false;
            }
        },
        nullptr);

    std::cout << "✓ Callbacks registered\n\n";

    //==========================================================================
    // STEP 8: CONFIGURE AND INITIALIZE TC CLIENT
    //==========================================================================
    // Before we can use the TC client, we must:
    //   1. Configure it with our DDOP and capabilities
    //   2. Initialize the state machine
    //
    std::cout << "[8/8] Configuring and initializing TC Client...\n";

    //--------------------------------------------------------------------------
    // CONFIGURE PARAMETERS EXPLAINED:
    //--------------------------------------------------------------------------
    // configure(ddop, maxBootTime, maxStructure, maxLocalization,
    //           reportToTC, supportGEO, supportDoc, supportTCBBS_A, supportTCBBS_B)
    //
    // Parameter 1: ddop (DeviceDescriptorObjectPool)
    //   The DDOP we created earlier describing our implement
    //
    // Parameter 2: maxBootTime (uint32_t, seconds)
    //   How long the TC should wait for us to boot up before requesting DDOP
    //   Typical values: 1-10 seconds
    //   We use 1 second since we're ready immediately
    //
    // Parameter 3: maxStructurePoolSize (uint32_t, bytes)
    //   Maximum size of DDOP structure in bytes
    //   0 = unlimited (let library calculate)
    //   Typical: 1000-32000 bytes depending on complexity
    //
    // Parameter 4: maxLocalizationPoolSize (uint32_t, bytes)
    //   Maximum size of localization label in bytes
    //   0 = unlimited
    //   Only needed if supporting multiple languages
    //
    // Parameter 5: reportToTC (bool)
    //   true  = We will actively report data to TC (normal operation)
    //   false = TC must poll us for data (not recommended)
    //
    // Parameter 6: supportTC-GEO (bool)
    //   true  = We support TC-GEO for position-based logging
    //   false = No position support (simpler)
    //   TC-GEO requires GPS and more complex implementation
    //
    // Parameter 7: supportDocumentation (bool)
    //   true  = We support peer-to-peer documentation transfer
    //   false = No documentation support
    //   Allows transferring manuals, calibration data, etc.
    //
    // Parameter 8: supportTC-BBS version A (bool)
    //   true  = Support TC Basic Boom Section Control version A
    //   false = No TC-BBS-A support
    //   Version A is older section control protocol
    //
    // Parameter 9: supportTC-BBS version B (bool)
    //   true  = Support TC Basic Boom Section Control version B
    //   false = No TC-BBS-B support
    //   Version B is newer, more advanced section control
    //
    tc->configure(ddop,   // Our DDOP
                  1,      // Boot time: 1 second (we're ready quickly)
                  0,      // Structure pool: unlimited (auto-calculate)
                  0,      // Localization pool: unlimited
                  true,   // Report to TC: YES (we actively send data)
                  false,  // TC-GEO: NO (no position support for now)
                  false,  // Documentation: NO (not needed for basic operation)
                  false,  // TC-BBS-A: NO (no section control)
                  false); // TC-BBS-B: NO (no section control)

    //--------------------------------------------------------------------------
    // INITIALIZE THE STATE MACHINE
    //--------------------------------------------------------------------------
    // initialize(useInternalThread)
    //
    // Parameter: useInternalThread (bool)
    //   true  = TC client runs in background thread (recommended)
    //   false = We must call update() regularly from our main loop
    //
    // When true, the TC client:
    //   - Manages connection automatically
    //   - Uploads DDOP when TC requests
    //   - Handles state transitions
    //   - Calls our callbacks as needed
    //   - All happens in background, no work needed in main loop
    //
    tc->initialize(true); // Use internal thread for automatic operation

    std::cout << "✓ TC Client initialized\n\n";

    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                  TC CLIENT IS RUNNING                        ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "The CAN stack is running in background threads.\n";
    std::cout << "Watch above for TC communication events.\n";
    std::cout << "Press Ctrl+C to exit cleanly.\n\n";

    //==========================================================================
    // MAIN LOOP - MONITORING
    //==========================================================================
    // The TC client runs in background threads, so our main loop just monitors
    // status and keeps the program alive.
    //
    // WHAT HAPPENS IN THE BACKGROUND:
    //   1. Partner discovery (waiting for TC to appear)
    //   2. Connection establishment
    //   3. DDOP upload
    //   4. Process data exchange
    //   5. Command handling
    //   6. All managed automatically by state machine!
    //
    int counter = 0;
    auto last_toggle_time = std::chrono::steady_clock::now();
    const auto toggle_interval = std::chrono::seconds(5); // Toggle work state every 5 seconds

    while (running) {
        auto now = std::chrono::steady_clock::now();

        // Toggle work state every 5 seconds (independent of send frequency)
        if (now - last_toggle_time >= toggle_interval) {
            current_work_state.store(1 - current_work_state.load());
            last_toggle_time = now;
        }

        std::cout << "\r  Work State: [" << (current_work_state.load() ? " ON " : "OFF ") << "]  " << std::flush;

        // echo("WORK STATE
        echo::format::String pretty_string;
        if (current_work_state.load()) {
            pretty_string = echo::format::String(" [ ON  ] ").bg(0, 255, 0).black().bold();
        } else {
            pretty_string = echo::format::String(" [ OFF ] ").bg(255, 0, 0).black().bold();
        }

        echo("WORK STATE ", pretty_string).inplace();

        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(send_frequency_ms.load()));
    }

    //==========================================================================
    // CLEANUP AND SHUTDOWN
    //==========================================================================
    // When user presses Ctrl+C, we need to clean up gracefully:
    //   1. Terminate TC client (stops background threads)
    //   2. Stop CAN hardware interface
    //   3. Release resources
    //
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    SHUTTING DOWN                             ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";

    std::cout << "Terminating TC client...\n";
    tc->terminate(); // Stop TC client threads and disconnect

    std::cout << "Stopping CAN interface...\n";
    isobus::CANHardwareInterface::stop(); // Stop CAN hardware

    std::cout << "\n✓ Shutdown complete. Goodbye!\n\n";
    return 0;
}
