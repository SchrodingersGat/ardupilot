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

#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Generator_Cortex.h"


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
    // TODO: Decode CAN frame

    return false;
}


void AP_Generator_Cortex::update()
{
    // TODO: Perform periodic tasks

    // TODO: Update internal readings

    update_frontend();

}


bool AP_Generator_Cortex::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    // TODO: Pre-arm checks
    return true;
}


void AP_Generator_Cortex::send_generator_status(const GCS_MAVLINK &channel)
{
    // TODO
}


bool AP_Generator_Cortex::healthy() const
{
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
