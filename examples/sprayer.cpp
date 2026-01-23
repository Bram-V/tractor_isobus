#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/socket_can_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_client.hpp"
#include "tractor/comms/serial.hpp"

#include <atomic>
#include <cassert>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

static std::atomic_bool running = true;
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

void process_nmea_line(const std::string &line) {
    if (line.substr(0, 5) == "$PHTG") {
        PHTGData phtg;
        if (parse_phtg(line, phtg)) {
            gnss_auth_status.store(phtg.auth_result);
            gnss_warning.store(phtg.warning);
        }
    }
}

class SectionControlImplementSimulator {
  public:
    static constexpr std::uint16_t MAX_NUMBER_SECTIONS_SUPPORTED = 256;
    static constexpr std::uint8_t NUMBER_SECTIONS_PER_CONDENSED_MESSAGE = 16;

    enum class ImplementDDOPObjectIDs : std::uint16_t {
        Device = 0,
        MainDeviceElement,
        DeviceActualWorkState,
        RequestDefaultProcessData,
        DeviceTotalTime,
        Connector,
        ConnectorXOffset,
        ConnectorYOffset,
        ConnectorType,
        SprayBoom,
        ActualWorkState,
        ActualWorkingWidth,
        AreaTotal,
        SetpointWorkState,
        SectionControlState,
        BoomXOffset,
        BoomYOffset,
        BoomZOffset,
        Section1,
        SectionMax = Section1 + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        Section1XOffset,
        SectionXOffsetMax = Section1XOffset + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        Section1YOffset,
        SectionYOffsetMax = Section1YOffset + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        Section1Width,
        SectionWidthMax = Section1Width + (MAX_NUMBER_SECTIONS_SUPPORTED - 1),
        ActualCondensedWorkingState1To16,
        ActualCondensedWorkingState17To32,
        ActualCondensedWorkingState33To48,
        ActualCondensedWorkingState49To64,
        ActualCondensedWorkingState65To80,
        ActualCondensedWorkingState81To96,
        ActualCondensedWorkingState97To112,
        ActualCondensedWorkingState113To128,
        ActualCondensedWorkingState129To144,
        ActualCondensedWorkingState145To160,
        ActualCondensedWorkingState161To176,
        ActualCondensedWorkingState177To192,
        ActualCondensedWorkingState193To208,
        ActualCondensedWorkingState209To224,
        ActualCondensedWorkingState225To240,
        ActualCondensedWorkingState241To256,
        SetpointCondensedWorkingState1To16,
        SetpointCondensedWorkingState17To32,
        SetpointCondensedWorkingState33To48,
        SetpointCondensedWorkingState49To64,
        SetpointCondensedWorkingState65To80,
        SetpointCondensedWorkingState81To96,
        SetpointCondensedWorkingState97To112,
        SetpointCondensedWorkingState113To128,
        SetpointCondensedWorkingState129To144,
        SetpointCondensedWorkingState145To160,
        SetpointCondensedWorkingState161To176,
        SetpointCondensedWorkingState177To192,
        SetpointCondensedWorkingState193To208,
        SetpointCondensedWorkingState209To224,
        SetpointCondensedWorkingState225To240,
        SetpointCondensedWorkingState241To256,
        LiquidProduct,
        TankCapacity,
        TankVolume,
        LifetimeApplicationVolumeTotal,
        PrescriptionControlState,
        ActualCulturalPractice,
        TargetRate,
        ActualRate,
        AreaPresentation,
        TimePresentation,
        ShortWidthPresentation,
        LongWidthPresentation,
        VolumePresentation,
        VolumePerAreaPresentation
    };

    SectionControlImplementSimulator() = default;

    void set_number_of_sections(std::uint8_t value) {
        sectionSetpointStates.resize(value, false);
        sectionSwitchStates.resize(value, true);
    }

    std::uint8_t get_number_of_sections() const { return sectionSetpointStates.size(); }

    void set_section_setpoint_state(std::uint8_t index, bool value) {
        if (index < sectionSetpointStates.size()) {
            sectionSetpointStates.at(index) = value;
        }
    }

    bool get_section_setpoint_state(std::uint8_t index) const {
        if (index < sectionSetpointStates.size()) {
            return sectionSetpointStates.at(index);
        }
        return false;
    }

    void set_section_switch_state(std::uint8_t index, bool value) {
        if (index < sectionSwitchStates.size()) {
            sectionSwitchStates.at(index) = value;
        }
    }

    bool get_section_switch_state(std::uint8_t index) const {
        if (index < sectionSwitchStates.size()) {
            return sectionSwitchStates.at(index);
        }
        return false;
    }

    bool get_section_actual_state(std::uint8_t index) const {
        if (index >= get_number_of_sections()) {
            return false;
        }
        if (isAutoMode) {
            return sectionSetpointStates.at(index);
        } else {
            return sectionSwitchStates.at(index);
        }
    }

    std::uint8_t get_actual_number_of_sections_on() const {
        std::uint8_t count = 0;
        for (std::uint8_t i = 0; i < get_number_of_sections(); i++) {
            if (get_section_actual_state(i)) {
                count++;
            }
        }
        return count;
    }

    std::uint32_t get_actual_rate() const { return targetRate * (get_actual_number_of_sections_on() > 0 ? 1 : 0); }

    void set_target_rate(std::uint32_t value) { targetRate = value; }

    bool get_actual_work_state() const { return get_actual_number_of_sections_on() > 0; }

    bool get_setpoint_work_state() const { return setpointWorkState; }

    void set_setpoint_work_state(bool value) { setpointWorkState = value; }

    void set_is_mode_auto(bool isAuto) { isAutoMode = isAuto; }

    bool get_is_mode_auto() const { return isAutoMode; }

    std::uint32_t get_prescription_control_state() const { return static_cast<std::uint32_t>(isAutoMode); }

    std::uint32_t get_section_control_state() const { return static_cast<std::uint32_t>(isAutoMode); }

    bool create_ddop(std::shared_ptr<isobus::DeviceDescriptorObjectPool> poolToPopulate,
                     isobus::NAME clientName) const {
        bool retVal = true;
        std::uint16_t elementCounter = 0;
        constexpr std::int32_t BOOM_WIDTH = 12000; // 12m boom width in mm
        assert(0 != get_number_of_sections());
        const std::int32_t SECTION_WIDTH = (BOOM_WIDTH / get_number_of_sections());
        poolToPopulate->clear();

        constexpr std::array<std::uint8_t, 7> localizationData = {'e', 'n', 0x50, 0x00, 0x55, 0x55, 0xFF};

        retVal &= poolToPopulate->add_device("HASHTAG", "0.0.11", "124", "HT0.0.11", localizationData,
                                             std::vector<std::uint8_t>(), clientName.get_full_name());
        retVal &= poolToPopulate->add_device_element(
            "Sprayer", elementCounter++, 0, isobus::task_controller_object::DeviceElementObject::Type::Device,
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::MainDeviceElement));
        retVal &= poolToPopulate->add_device_process_data(
            "Actual Work State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState),
            isobus::NULL_OBJECT_ID,
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceActualWorkState));
        retVal &= poolToPopulate->add_device_process_data(
            "Request Default PD", static_cast<std::uint16_t>(ImplementDDOPObjectIDs::RequestDefaultProcessData),
            isobus::NULL_OBJECT_ID, 0,
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::Total),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::RequestDefaultProcessData));
        retVal &= poolToPopulate->add_device_process_data(
            "Total Time", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::EffectiveTotalTime),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TimePresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::Total),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceTotalTime));
        retVal &= poolToPopulate->add_device_element(
            "Connector", elementCounter++, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::MainDeviceElement),
            isobus::task_controller_object::DeviceElementObject::Type::Connector,
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Connector));
        retVal &= poolToPopulate->add_device_process_data(
            "Connector X", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorXOffset));
        retVal &= poolToPopulate->add_device_process_data(
            "Connector Y", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint8_t>(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorYOffset));
        retVal &= poolToPopulate->add_device_property(
            "Type", 9, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ConnectorType), isobus::NULL_OBJECT_ID,
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorType));

        retVal &= poolToPopulate->add_device_element(
            "Boom", elementCounter++, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::MainDeviceElement),
            isobus::task_controller_object::DeviceElementObject::Type::Function,
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SprayBoom));
        retVal &= poolToPopulate->add_device_property(
            "Offset X", 0, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomXOffset));
        retVal &= poolToPopulate->add_device_property(
            "Offset Y", 0, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomYOffset));
        retVal &= poolToPopulate->add_device_property(
            "Offset Z", 0, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetZ),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomZOffset));
        retVal &= poolToPopulate->add_device_process_data(
            "Actual Working Width", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LongWidthPresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualWorkingWidth));
        retVal &= poolToPopulate->add_device_process_data(
            "Setpoint Work State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState),
            isobus::NULL_OBJECT_ID,
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SetpointWorkState));
        retVal &= poolToPopulate->add_device_process_data(
            "Area Total", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::TotalArea),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::AreaPresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::Total),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::AreaTotal));
        retVal &= poolToPopulate->add_device_process_data(
            "Section Control State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState),
            isobus::NULL_OBJECT_ID,
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SectionControlState));

        retVal &= poolToPopulate->add_device_element("Product", elementCounter++,
                                                     static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SprayBoom),
                                                     isobus::task_controller_object::DeviceElementObject::Type::Bin,
                                                     static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LiquidProduct));
        retVal &= poolToPopulate->add_device_process_data(
            "Tank Capacity", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::MaximumVolumeContent),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::VolumePresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TankCapacity));
        retVal &= poolToPopulate->add_device_process_data(
            "Tank Volume", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualVolumeContent),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::VolumePresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TankVolume));
        retVal &= poolToPopulate->add_device_process_data(
            "Lifetime Total Volume",
            static_cast<std::uint16_t>(isobus::DataDescriptionIndex::LifetimeApplicationTotalVolume),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::VolumePresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::Total),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LifetimeApplicationVolumeTotal));
        retVal &= poolToPopulate->add_device_process_data(
            "Rx Control State", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::PrescriptionControlState),
            isobus::NULL_OBJECT_ID,
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::PrescriptionControlState));
        retVal &= poolToPopulate->add_device_process_data(
            "Target Rate",
            static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointVolumePerAreaApplicationRate),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::VolumePerAreaPresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TargetRate));
        retVal &= poolToPopulate->add_device_process_data(
            "Actual Rate", static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualVolumePerAreaApplicationRate),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::VolumePerAreaPresentation),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
            static_cast<std::uint8_t>(
                isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange) |
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval),
            static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualRate));
        retVal &= poolToPopulate->add_device_property(
            "Operation Type", 3, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCulturalPractice),
            isobus::NULL_OBJECT_ID, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualCulturalPractice));

        for (std::uint_fast8_t i = 0; i < get_number_of_sections(); i++) {
            std::int32_t individualSectionWidth = BOOM_WIDTH / get_number_of_sections();
            std::ostringstream ss;
            ss << "Section " << static_cast<int>(i);
            retVal &= poolToPopulate->add_device_element(
                ss.str(), elementCounter++, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SprayBoom),
                isobus::task_controller_object::DeviceElementObject::Type::Section,
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1) + i);
            retVal &= poolToPopulate->add_device_property(
                "Offset X", -20, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetX),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LongWidthPresentation),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1XOffset) + i);
            retVal &= poolToPopulate->add_device_property(
                "Offset Y", ((-BOOM_WIDTH) / 2) + (i * SECTION_WIDTH) + (SECTION_WIDTH / 2),
                static_cast<std::uint16_t>(isobus::DataDescriptionIndex::DeviceElementOffsetY),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LongWidthPresentation),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1YOffset) + i);
            retVal &= poolToPopulate->add_device_property(
                "Width", individualSectionWidth,
                static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LongWidthPresentation),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1Width) + i);
            auto section = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(i + static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1)));
            section->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1YOffset) +
                                                   i);
            section->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1XOffset) +
                                                   i);
            section->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Section1Width) +
                                                   i);
        }

        std::uint16_t sectionCounter = 0;
        while (sectionCounter < get_number_of_sections()) {
            retVal &= poolToPopulate->add_device_process_data(
                "Actual Work State 1-16",
                static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) +
                    (sectionCounter / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE),
                isobus::NULL_OBJECT_ID,
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualCondensedWorkingState1To16) +
                    (sectionCounter / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE));
            retVal &= poolToPopulate->add_device_process_data(
                "Setpoint Work State 1-16",
                static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) +
                    (sectionCounter / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE),
                isobus::NULL_OBJECT_ID,
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable) |
                    static_cast<std::uint8_t>(
                        isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::MemberOfDefaultSet),
                static_cast<std::uint8_t>(
                    isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange),
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SetpointCondensedWorkingState1To16) +
                    (sectionCounter / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE));
            sectionCounter += NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
        }

        retVal &= poolToPopulate->add_device_value_presentation(
            "mm", 0, 1.0f, 0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ShortWidthPresentation));
        retVal &= poolToPopulate->add_device_value_presentation(
            "m", 0, 0.001f, 0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LongWidthPresentation));
        retVal &= poolToPopulate->add_device_value_presentation(
            "m^2", 0, 1.0f, 0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::AreaPresentation));
        retVal &= poolToPopulate->add_device_value_presentation(
            "L", 0, 0.001f, 0, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::VolumePresentation));
        retVal &= poolToPopulate->add_device_value_presentation(
            "minutes", 0, 1.0f, 1, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TimePresentation));
        retVal &= poolToPopulate->add_device_value_presentation(
            "L/ha", 0, 0.001f, 1, static_cast<std::uint16_t>(ImplementDDOPObjectIDs::VolumePerAreaPresentation));

        if (retVal) {
            auto sprayer = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(
                    static_cast<std::uint16_t>(ImplementDDOPObjectIDs::MainDeviceElement)));
            auto connector = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::Connector)));
            auto boom = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SprayBoom)));
            auto product = std::static_pointer_cast<isobus::task_controller_object::DeviceElementObject>(
                poolToPopulate->get_object_by_id(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LiquidProduct)));

            sprayer->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceActualWorkState));
            sprayer->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::DeviceTotalTime));

            connector->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorXOffset));
            connector->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorYOffset));
            connector->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ConnectorType));

            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomXOffset));
            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomYOffset));
            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::BoomZOffset));
            boom->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualWorkingWidth));
            boom->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SectionControlState));

            sectionCounter = 0;
            while (sectionCounter < get_number_of_sections()) {
                boom->add_reference_to_child_object(
                    static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualCondensedWorkingState1To16) +
                    (sectionCounter / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE));
                boom->add_reference_to_child_object(
                    static_cast<std::uint16_t>(ImplementDDOPObjectIDs::SetpointCondensedWorkingState1To16) +
                    (sectionCounter / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE));
                sectionCounter += NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
            }

            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TankCapacity));
            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TankVolume));
            product->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::LifetimeApplicationVolumeTotal));
            product->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::PrescriptionControlState));
            product->add_reference_to_child_object(
                static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualCulturalPractice));
            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::TargetRate));
            product->add_reference_to_child_object(static_cast<std::uint16_t>(ImplementDDOPObjectIDs::ActualRate));
        }
        return retVal;
    }

    static bool request_value_command_callback(std::uint16_t, std::uint16_t DDI, std::int32_t &value,
                                               void *parentPointer) {
        if (nullptr != parentPointer) {
            auto sim = reinterpret_cast<SectionControlImplementSimulator *>(parentPointer);
            switch (DDI) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::MaximumVolumeContent):
                value = 4542494;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualVolumeContent):
                value = 3785000;
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
                value = sim->get_section_control_state();
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::PrescriptionControlState):
                value = sim->get_prescription_control_state();
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState17_32):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState33_48):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState49_64):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState65_80):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState81_96):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState97_112):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState113_128):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState129_144):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState145_160):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState161_176):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState177_192):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState193_208):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState209_224):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState225_240):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256): {
                std::uint16_t sectionIndexOffset =
                    NUMBER_SECTIONS_PER_CONDENSED_MESSAGE *
                    (DDI - static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16));
                value = 0;
                for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++) {
                    if ((i + sectionIndexOffset) < sim->get_number_of_sections()) {
                        value |=
                            (sim->get_section_actual_state(i + sectionIndexOffset) ? static_cast<std::uint32_t>(0x01)
                                                                                   : static_cast<std::uint32_t>(0x00))
                            << (2 * i);
                    } else {
                        value |= (static_cast<std::uint32_t>(0x03) << (2 * i));
                    }
                }
            } break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualVolumePerAreaApplicationRate):
                value = sim->get_actual_rate();
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
                value = sim->get_actual_work_state();
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkingWidth):
                value = 12000; // 12m boom width in mm
                break;
            default:
                value = 0;
                break;
            }
        }
        return true;
    }

    static bool value_command_callback(std::uint16_t, std::uint16_t DDI, std::int32_t processVariableValue,
                                       void *parentPointer) {
        if (nullptr != parentPointer) {
            auto sim = reinterpret_cast<SectionControlImplementSimulator *>(parentPointer);
            switch (DDI) {
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState17_32):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState33_48):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState49_64):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState65_80):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState81_96):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState97_112):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState113_128):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState129_144):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState145_160):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState161_176):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState177_192):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState193_208):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState209_224):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState225_240):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256): {
                std::uint16_t sectionIndexOffset =
                    NUMBER_SECTIONS_PER_CONDENSED_MESSAGE *
                    (DDI - static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16));
                for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++) {
                    if ((sectionIndexOffset + i) < sim->get_number_of_sections()) {
                        sim->set_section_setpoint_state(sectionIndexOffset + i,
                                                        (0x01 == ((processVariableValue >> (2 * i)) & 0x03)));
                    }
                }
            } break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointVolumePerAreaApplicationRate):
                sim->set_target_rate(processVariableValue);
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState):
                sim->set_setpoint_work_state(processVariableValue != 0);
                break;
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
            case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::PrescriptionControlState):
                sim->set_is_mode_auto(processVariableValue != 0);
                break;
            default:
                break;
            }
        }
        return true;
    }

  private:
    std::vector<bool> sectionSetpointStates;
    std::vector<bool> sectionSwitchStates;
    std::uint32_t targetRate = 0;
    bool setpointWorkState = true;
    bool isAutoMode = true;
};

/// @brief Exports a DDOP to an ISOXML TASKDATA.xml file
/// @param ddop The DDOP to export
/// @param filename The output filename for the XML file
/// @returns true if export was successful, false otherwise
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

/// @brief Prints a summary of the DDOP structure to console
/// @param ddop The DDOP to analyze
void print_ddop_summary(std::shared_ptr<isobus::DeviceDescriptorObjectPool> ddop) {
    if (!ddop) {
        std::cout << "DDOP is null\n";
        return;
    }

    std::cout << "\n=== DDOP Structure Summary ===\n";
    std::cout << "Total objects: " << ddop->size() << "\n";

    // Count object types
    int deviceCount = 0, elementCount = 0, processDataCount = 0, propertyCount = 0, presentationCount = 0;

    for (std::uint16_t i = 0; i < ddop->size(); i++) {
        auto obj = ddop->get_object_by_index(i);
        if (obj) {
            switch (obj->get_object_type()) {
            case isobus::task_controller_object::ObjectTypes::Device:
                deviceCount++;
                break;
            case isobus::task_controller_object::ObjectTypes::DeviceElement:
                elementCount++;
                break;
            case isobus::task_controller_object::ObjectTypes::DeviceProcessData:
                processDataCount++;
                break;
            case isobus::task_controller_object::ObjectTypes::DeviceProperty:
                propertyCount++;
                break;
            case isobus::task_controller_object::ObjectTypes::DeviceValuePresentation:
                presentationCount++;
                break;
            default:
                break;
            }
        }
    }

    std::cout << "Devices: " << deviceCount << "\n";
    std::cout << "Device Elements: " << elementCount << "\n";
    std::cout << "Process Data: " << processDataCount << "\n";
    std::cout << "Properties: " << propertyCount << "\n";
    std::cout << "Value Presentations: " << presentationCount << "\n";
    std::cout << "===============================\n\n";
}

int main(int argc, char **argv) {
    const char *serial_device = "/tmp/ttyV0";
    int serial_baud = 115200;
    const char *xml_export_filename = nullptr;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--xml-export" && i + 1 < argc) {
            xml_export_filename = argv[++i];
        } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            std::cout << "Usage: " << argv[0] << " [serial_device] [serial_baud] [--xml-export filename.xml]\n";
            std::cout << "  serial_device: Serial device path (default: /tmp/ttyV0)\n";
            std::cout << "  serial_baud: Serial baud rate (default: 115200)\n";
            std::cout << "  --xml-export filename.xml: Export DDOP to XML file\n";
            return 0;
        } else if (i == 1) {
            serial_device = argv[i];
        } else if (i == 2) {
            serial_baud = std::atoi(argv[i]);
        }
    }

    std::signal(SIGINT, signal_handler);

    std::cout << "Sprayer TC Client + Serial PHTG Reader\n";
    std::cout << "Serial: " << serial_device << " @ " << serial_baud << " baud\n\n";

    auto nmea_serial = std::make_shared<tractor::comms::Serial>(serial_device, serial_baud);
    nmea_serial->on_line([](const std::string &line) { process_nmea_line(line); });
    nmea_serial->on_connection([](bool connected) {
        if (connected) {
            std::cout << "Serial port connected\n";
        } else {
            std::cout << "Serial port disconnected\n";
        }
    });
    nmea_serial->on_error([](const std::string &error) { std::cerr << "Serial error: " << error << "\n"; });

    if (!nmea_serial->start()) {
        std::cerr << "Failed to start serial communication\n";
        return -1;
    }

    auto canDriver = std::make_shared<isobus::SocketCANInterface>("can0");
    if (nullptr == canDriver) {
        std::cout << "Unable to find a CAN driver." << std::endl;
        return -1;
    }

    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);

    if ((!isobus::CANHardwareInterface::start()) || (!canDriver->get_is_valid())) {
        std::cout << "Failed to start hardware interface." << std::endl;
        return -2;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    isobus::NAME TestDeviceNAME(0);
    TestDeviceNAME.set_arbitrary_address_capable(true);
    TestDeviceNAME.set_industry_group(2);
    TestDeviceNAME.set_device_class(6);
    TestDeviceNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
    TestDeviceNAME.set_identity_number(2);
    TestDeviceNAME.set_ecu_instance(0);
    TestDeviceNAME.set_function_instance(0);
    TestDeviceNAME.set_device_class_instance(0);
    TestDeviceNAME.set_manufacturer_code(1407);

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

    auto TestInternalECU = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(TestDeviceNAME, 0);
    auto TestPartnerTC = isobus::CANNetworkManager::CANNetwork.create_partnered_control_function(0, tcNameFilters);

    auto TestTCClient = std::make_shared<isobus::TaskControllerClient>(TestPartnerTC, TestInternalECU, nullptr);

    auto myDDOP = std::make_shared<isobus::DeviceDescriptorObjectPool>();
    bool tcClientStarted = false;

    constexpr std::uint16_t NUMBER_OF_SECTIONS_TO_CREATE = 4;
    SectionControlImplementSimulator rateController;
    rateController.set_number_of_sections(NUMBER_OF_SECTIONS_TO_CREATE);

    std::cout << "Sprayer TC Client Example\n";
    std::cout << "Waiting for TC server...\n\n";

    while (running) {
        if (!tcClientStarted) {
            if (rateController.create_ddop(myDDOP, TestInternalECU->get_NAME())) {
                // Export DDOP to XML if requested
                if (xml_export_filename != nullptr) {
                    if (export_ddop_to_xml(myDDOP, xml_export_filename)) {
                        std::cout << "DDOP XML export completed. Exiting...\n";
                        break;
                    } else {
                        std::cout << "Failed to export DDOP to XML, continuing with TC client...\n";
                    }
                }

                TestTCClient->configure(myDDOP, 1, NUMBER_OF_SECTIONS_TO_CREATE, 1, true, false, true, false, true);
                TestTCClient->add_request_value_callback(
                    SectionControlImplementSimulator::request_value_command_callback, &rateController);
                TestTCClient->add_value_command_callback(SectionControlImplementSimulator::value_command_callback,
                                                         &rateController);
                TestTCClient->initialize(true);
                tcClientStarted = true;
                std::cout << "TC Client initialized successfully\n";
            } else {
                std::cout << "Failed to create DDOP\n";
                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    std::cout << "\nShutting down...\n";
    nmea_serial->stop();
    TestTCClient->terminate();
    isobus::CANHardwareInterface::stop();
    return (!tcClientStarted);
}
