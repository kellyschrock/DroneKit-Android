package com.o3dr.services.android.lib.drone.action

import com.o3dr.services.android.lib.drone.action.ControlActions
import com.o3dr.services.android.lib.util.Utils

/**
 * Created by Fredia Huya-Kouadio on 9/7/15.
 */
object ControlActions {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.drone.action.control"
    const val ACTION_DO_GUIDED_TAKEOFF = Utils.PACKAGE_NAME + ".action.DO_GUIDED_TAKEOFF"
    const val EXTRA_ALTITUDE = "extra_altitude"
    const val ACTION_SEND_GUIDED_POINT = Utils.PACKAGE_NAME + ".action.SEND_GUIDED_POINT"
    const val ACTION_SEND_GUIDED_POINT_DIRECT = Utils.PACKAGE_NAME + ".action.SEND_GUIDED_POINT_DIRECT"
    const val EXTRA_GUIDED_POINT = "extra_guided_point"
    const val EXTRA_FORCE_GUIDED_POINT = "extra_force_guided_point"
    const val ACTION_SET_GUIDED_ALTITUDE = Utils.PACKAGE_NAME + ".action.SET_GUIDED_ALTITUDE"
    const val ACTION_SET_CONDITION_YAW = PACKAGE_NAME + ".SET_CONDITION_YAW"
    const val EXTRA_YAW_TARGET_ANGLE = "extra_yaw_target_angle"
    const val EXTRA_YAW_CHANGE_RATE = "extra_yaw_change_rate"
    const val EXTRA_YAW_IS_RELATIVE = "extra_yaw_is_relative"
    const val ACTION_SET_VELOCITY = PACKAGE_NAME + ".SET_VELOCITY"
    const val ACTION_SEND_BRAKE_VEHICLE = PACKAGE_NAME + ".action.SEND_BRAKE_VEHICLE"
    const val ACTION_VTOL_TRANSITION = PACKAGE_NAME + ".action.VTOL_TRANSITION"
    const val EXTRA_VTOL_TARGET_STATE = "extra_vtol_state"

    /**
     * X velocity in meters per second.
     */
    const val EXTRA_VELOCITY_X = "extra_velocity_x"

    /**
     * Y velocity in meters per second.
     */
    const val EXTRA_VELOCITY_Y = "extra_velocity_y"

    /**
     * Z velocity in meters per second.
     */
    const val EXTRA_VELOCITY_Z = "extra_velocity_z"
    const val ACTION_ENABLE_MANUAL_CONTROL = PACKAGE_NAME + ".ENABLE_MANUAL_CONTROL"
    const val EXTRA_DO_ENABLE = "extra_do_enable"
    const val ACTION_LOOK_AT_TARGET = PACKAGE_NAME + ".action.LOOK_AT_TARGET"

    /**
     * Geo coordinate to orient the vehicle to
     */
    const val EXTRA_LOOK_AT_TARGET = "extra_look_at_target"

    /**
     * Reset ROI: Stop staring at something
     */
    const val ACTION_RESET_ROI = PACKAGE_NAME + ".RESET_ROI"
}
