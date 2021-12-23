package com.o3dr.services.android.lib.drone.action

import com.o3dr.services.android.lib.drone.action.StateActions
import com.o3dr.services.android.lib.util.Utils

/**
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
object StateActions {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.drone.action.state"
    const val ACTION_ARM = Utils.PACKAGE_NAME + ".action.ARM"
    const val EXTRA_ARM = "extra_arm"
    const val EXTRA_EMERGENCY_DISARM = "extra_emergency_disarm"
    const val ACTION_SET_VEHICLE_MODE = Utils.PACKAGE_NAME + ".action.SET_VEHICLE_MODE"
    const val EXTRA_VEHICLE_MODE = "extra_vehicle_mode"
    const val ACTION_SET_VEHICLE_HOME = Utils.PACKAGE_NAME + ".action.SET_VEHICLE_HOME"
    const val EXTRA_VEHICLE_HOME_LOCATION = "extra_vehicle_home_location"
    const val ACTION_ENABLE_RETURN_TO_ME = Utils.PACKAGE_NAME + ".action.ENABLE_RETURN_TO_ME"
    const val EXTRA_IS_RETURN_TO_ME_ENABLED = "extra_is_return_to_me_enabled"
    const val ACTION_UPDATE_VEHICLE_DATA_STREAM_RATE = PACKAGE_NAME + ".action.UPDATE_VEHICLE_DATA_STREAM_RATE"
    const val EXTRA_VEHICLE_DATA_STREAM_RATE = "extra_vehicle_data_stream_rate"
}
