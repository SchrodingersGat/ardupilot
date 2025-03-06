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
    // Update internal readings

    _voltage = telemetry.generator.voltage;
    _current = telemetry.generator.current;
    _rpm = (uint16_t) abs(telemetry.generator.rpm);

    // TODO: Estimate consumed mAh
    // TODO: Estimate fuel remaining?

    update_frontend();

}


bool AP_Generator_Cortex::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    // TODO: Pre-arm checks
    return true;
}


void AP_Generator_Cortex::send_generator_status(const GCS_MAVLINK &channel)
{
    uint64_t status = 0;

    if (!is_connected()) {
        return;
    }

    // TODO: Copy status flags from MAV_GENERATOR_STATUS_FLAG enum

    mavlink_msg_generator_status_send(
        channel.get_chan(),
        status,
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
    // TODO: Stop the system
    return false;
}


bool AP_Generator_Cortex::idle(void)
{
    // TODO: Start the system
    return false;
}


bool AP_Generator_Cortex::run(void)
{
    // TODO: Crank the system
    return false;
}

#endif // AP_GENERATOR_CORTEX_ENABLED
