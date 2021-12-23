package com.o3dr.services.android.lib.drone.action

import com.o3dr.services.android.lib.util.Utils

/**
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
object ExperimentalActions {
    const val ACTION_TRIGGER_CAMERA = Utils.PACKAGE_NAME + ".action.TRIGGER_CAMERA"
    const val ACTION_SET_ROI = Utils.PACKAGE_NAME + ".action.SET_ROI"
    const val EXTRA_SET_ROI_LAT_LONG_ALT = "extra_set_roi_lat_long_alt"
    const val ACTION_EPM_COMMAND = Utils.PACKAGE_NAME + ".action.EPM_COMMAND"
    const val EXTRA_EPM_RELEASE = Utils.PACKAGE_NAME + "extra_epm_release"
    const val ACTION_SEND_MAVLINK_MESSAGE = Utils.PACKAGE_NAME + ".action.SEND_MAVLINK_MESSAGE"
    const val EXTRA_MAVLINK_MESSAGE = "extra_mavlink_message"
    const val EXTRA_TARGET_SYS = "extra_target_sys"
    const val EXTRA_TARGET_COMPONENT = "extra_target_component"
    const val ACTION_SET_RELAY = Utils.PACKAGE_NAME + ".action.SET_RELAY"
    const val EXTRA_RELAY_NUMBER = "extra_relay_number"
    const val EXTRA_IS_RELAY_ON = "extra_is_relay_on"
    const val ACTION_SET_SERVO = Utils.PACKAGE_NAME + ".action.SET_SERVO"
    const val EXTRA_SERVO_CHANNEL = "extra_servo_channel"
    const val EXTRA_SERVO_PWM = "extra_servo_PWM"
    const val ACTION_START_VIDEO_STREAM_FOR_OBSERVER = Utils.PACKAGE_NAME + ".action.camera.START_VIDEO_STREAM_FOR_OBSERVER"
    const val ACTION_STOP_VIDEO_STREAM_FOR_OBSERVER = Utils.PACKAGE_NAME + ".action.camera.STOP_VIDEO_STREAM_FOR_OBSERVER"
}
