package com.o3dr.services.android.lib.gcs.action

import com.o3dr.services.android.lib.util.Utils

/**
 * Created by Fredia Huya-Kouadio on 1/19/15.
 */
object CalibrationActions {
    const val ACTION_START_IMU_CALIBRATION = Utils.PACKAGE_NAME + ".action.START_IMU_CALIBRATION"
    const val ACTION_SEND_IMU_CALIBRATION_ACK = Utils.PACKAGE_NAME + ".action" +
            ".SEND_IMU_CALIBRATION_ACK"
    const val EXTRA_IMU_STEP = "extra_step"
    const val ACTION_START_MAGNETOMETER_CALIBRATION = Utils.PACKAGE_NAME + ".action" +
            ".START_MAGNETOMETER_CALIBRATION"
    const val ACTION_ACCEPT_MAGNETOMETER_CALIBRATION = Utils.PACKAGE_NAME + ".action" +
            ".ACCEPT_MAGNETOMETER_CALIBRATION"
    const val ACTION_CANCEL_MAGNETOMETER_CALIBRATION = Utils.PACKAGE_NAME + ".action" +
            ".CANCEL_MAGNETOMETER_CALIBRATION"
    const val EXTRA_RETRY_ON_FAILURE = "extra_retry_on_failure"
    const val EXTRA_SAVE_AUTOMATICALLY = "extra_save_automatically"
    const val EXTRA_START_DELAY = "extra_start_delay"
}
