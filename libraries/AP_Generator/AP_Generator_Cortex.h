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

#pragma once

#include "AP_Generator_config.h"

#if AP_GENERATOR_CORTEX_ENABLED

#include "AP_Generator_Backend.h"

#include <AP_Logger/AP_Logger_config.h>
#include <AP_Common/AP_Common.h>
#include <stdint.h>
#include <stdio.h>


/**
 * Class defining the Cortex generator from Currawong Engineering
 */
class AP_Generator_Cortex : public AP_Generator_Backend
{
public:
    using AP_Generator_Backend::AP_Generator_Backend;

    // init should be called at vehicle startup to get the generator library ready
    __INITFUNC__ void init(void) override;
    // update should be called regularly to update the generator state
    void update(void) override;

    // methods to control the generator state:
    bool stop(void) override;
    bool idle(void) override;
    bool run(void) override;

    // method to send a GENERATOR_STATUS mavlink message
    void send_generator_status(const GCS_MAVLINK &channel) override;

    // prearm checks to ensure generator is good for arming.  Note
    // that if the generator has never sent us a message then these
    // automatically pass!
    bool pre_arm_check(char *failmsg, uint8_t failmsg_len) const override;

    // Update front end with voltage, current, and rpm values
    void update_frontend_readings(void);

    // healthy returns true if the generator is not present, or it is
    // present, providing telemetry and not indicating an errors.
    bool healthy() const override;

    static AP_Generator_Cortex* get_instance(void)
    {
        return _singleton;
    }

private:
    bool handle_message(AP_HAL::CANFrame &frame);

    static AP_Generator_Cortex* _singleton;

    friend class AP_PiccoloCAN;
};

#endif // AP_GENERATOR_CORTEX_ENABLED
