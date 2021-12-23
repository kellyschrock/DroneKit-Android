package com.o3dr.services.android.lib.drone.action

import com.o3dr.services.android.lib.util.Utils

object GimbalActions {
    const val GIMBAL_PITCH = "gimbal_pitch"
    const val GIMBAL_YAW = "gimbal_yaw"
    const val GIMBAL_ROLL = "gimbal_roll"
    const val ACTION_SET_GIMBAL_ORIENTATION = Utils.PACKAGE_NAME + ".action.gimbal" +
            ".SET_GIMBAL_ORIENTATION"
    const val ACTION_SET_GIMBAL_MOUNT_MODE = Utils.PACKAGE_NAME + ".action.gimbal.SET_GIMBAL_MOUNT_MODE"

    /**
     * Gimbal mount mode.
     * @see {@link com.MAVLink.enums.MAV_MOUNT_MODE}
     */
    const val GIMBAL_MOUNT_MODE = "gimbal_mount_mode"
    const val ACTION_RESET_GIMBAL_MOUNT_MODE = Utils.PACKAGE_NAME + ".action.gimbal.RESET_GIMBAL_MOUNT_MODE"
}
