#pragma once

#include <GCS_MAVLink/GCS.h>

#include "defines.h"

class GCS_MAVLINK_Blimp : public GCS_MAVLINK
{

public:

    using GCS_MAVLINK::GCS_MAVLINK;

protected:

    MAV_RESULT handle_flight_termination(const mavlink_command_int_t &packet) override;

    bool params_ready() const override;
    void send_banner() override;

    MAV_RESULT _handle_command_preflight_calibration(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;

    void send_position_target_global_int() override;

    MAV_RESULT handle_command_do_set_roi(const Location &roi_loc) override;
    MAV_RESULT handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg) override;
    MAV_RESULT handle_command_int_do_reposition(const mavlink_command_int_t &packet);

#if AP_MAVLINK_COMMAND_LONG_ENABLED
    bool mav_frame_for_command_long(MAV_FRAME &frame, MAV_CMD packet_command) const override;
#endif

    void send_nav_controller_output() const override; //TODO Apparently can't remove this or the build fails.
    uint64_t capabilities() const override;

    virtual MAV_VTOL_STATE vtol_state() const override
    {
        return MAV_VTOL_STATE_MC;
    };
    virtual MAV_LANDED_STATE landed_state() const override;

#if HAL_LOGGING_ENABLED
    uint32_t log_radio_bit() const override { return MASK_LOG_PM; }
#endif

    // Send the mode with the given index (not mode number!) return the total number of modes
    // Index starts at 1
    uint8_t send_available_mode(uint8_t index) const override;

private:

    void handle_message(const mavlink_message_t &msg) override;
    bool try_send_message(enum ap_message id) override;

    void packetReceived(const mavlink_status_t &status,
                        const mavlink_message_t &msg) override;

    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;

    float vfr_hud_airspeed() const override;
    int16_t vfr_hud_throttle() const override;
    float vfr_hud_alt() const override;

    void send_pid_tuning() override;

    void send_wind() const;

    //This is 1-indexed, unlike most enums for consistency with the mavlink PID_TUNING enums.
    enum PID_SEND : uint8_t {
        VELX =        1,
        VELY =        2,
        VELZ =        3,
        VELYAW =      4,
        POSX =        5,
        POSY =        6,
        POSZ =        7,
        POSYAW =      8,
    };

#if HAL_HIGH_LATENCY2_ENABLED
    uint8_t high_latency_wind_speed() const override;
    uint8_t high_latency_wind_direction() const override;
#endif // HAL_HIGH_LATENCY2_ENABLED
};
