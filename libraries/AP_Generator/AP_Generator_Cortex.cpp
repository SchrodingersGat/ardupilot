/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Oliver Walters / Currawong Engineering Pty Ltd
 */

#pragma GCC optimize("Os")

#include "AP_Generator_config.h"

#if AP_GENERATOR_CORTEX_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Generator_Cortex.h"

#include <AP_Param/AP_Param.h>


AP_Generator_Cortex* AP_Generator_Cortex::_singleton;


void AP_Generator_Cortex::init()
{
    _singleton = this;

    // Inform frontend which measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = false;
    _frontend._has_fuel_remaining = false;
}


// Decode received CAN frame    
bool AP_Generator_Cortex::handle_message(AP_HAL::CANFrame &frame)
{
    bool result = true;

    if (decodeCortex_TelemetryStatusPacketStructure(&frame, &telemetry.status)) {
    } else if (decodeCortex_TelemetryGeneratorPacketStructure(&frame, &telemetry.generator)) {
    } else if (decodeCortex_TelemetryBatteryPacketStructure(&frame, &telemetry.battery)) {
    } else if (decodeCortex_TelemetryOutputRailPacketStructure(&frame, &telemetry.rails)) {
    } else {
        result = false;
    }

    if (result) {
        last_reading_ms = AP_HAL::millis();
    }

    return result;
}


void AP_Generator_Cortex::update()
{
    static bool prev_connection_state = false;

    const bool connected = is_connected();

    if (connected != prev_connection_state) {
        if (connected) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Cortex generator connected");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Cortex generator disconnected");
        }
    }

    prev_connection_state = connected;

    // Update internal readings
    if (connected) {
        _voltage = telemetry.generator.voltage;
        _current = telemetry.generator.current;
        _rpm = (uint16_t) abs(telemetry.generator.rpm);
    }

    update_frontend();

    // TODO: Data logging?
}


bool AP_Generator_Cortex::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    if (!is_connected()) {
        snprintf(failmsg, failmsg_len, "Generator is not connected");
        return false;
    }

    if (is_inhibited()) {
        snprintf(failmsg, failmsg_len, "Generator is inhibited");
        return false;
    }

    return true;
}


void AP_Generator_Cortex::send_generator_status(const GCS_MAVLINK &channel)
{
    uint64_t status_flags = 0;

    if (!is_connected()) {
        return;
    }

    // Copy status flags from MAV_GENERATOR_STATUS_FLAG enum
    const Cortex_StatusBits_t &status = telemetry.status.status;

    // Check inihibit state
    if (status.inhibited) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_START_INHIBITED;
    }
    
    switch (status.mode) {
        case CORTEX_MODE_STANDBY:
            status_flags |= MAV_GENERATOR_STATUS_FLAG_IDLE;
            if (status.readyToRun) {
                status_flags |= MAV_GENERATOR_STATUS_FLAG_READY;
            }
            break;
        case CORTEX_MODE_RUNNING:
            status_flags |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
            break;
        default:
            break;
    }

    if (status.readyToRun) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_READY;
    }

    if (telemetry.generator.current <= 0.05f) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_GENERATING;
    }

    if (telemetry.battery.current <= 0.05f) {
        status_flags |= MAV_GENERATOR_STATUS_FLAG_CHARGING;
    }

    // TODO: Add in other status flags for various operational conditions

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status_flags,
        (uint16_t) abs(telemetry.generator.rpm),
        batteryCurrent(),
        loadCurrent(),
        generatorPower(),
        generatorVoltage(),
        0, // TODO: rectifier temperature
        0, // TODO: battery current setpoint
        telemetry.generator.temperature,
        0, // TODO: runtime
        0  // TODO: time until maintenance
    );
}


bool AP_Generator_Cortex::is_connected(void) const
{
    const uint32_t now = AP_HAL::millis();

    return (now - last_reading_ms) < 2000;
}


bool AP_Generator_Cortex::is_inhibited(void) const
{
    return telemetry.status.status.inhibited;
}


bool AP_Generator_Cortex::healthy() const
{
    if (!is_connected()) {
        return false;
    }

    // TODO: Check if generator is healthy

    return true;
}

bool AP_Generator_Cortex::stop(void)
{
    // Generator cannot be remotely killed - have to stop the engine
    return false;
}


bool AP_Generator_Cortex::idle(void)
{
    // No "idle" state in this configuration
    return false;
}


bool AP_Generator_Cortex::run(void)
{
    // TODO: Send the "Start" command to the Cortex generator
    // TODO: Need to work out how to hook into PiccoloCAN from here
    return false;
}

#endif // AP_GENERATOR_CORTEX_ENABLED
