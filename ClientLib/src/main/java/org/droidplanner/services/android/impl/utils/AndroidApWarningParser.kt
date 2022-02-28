package org.droidplanner.services.android.impl.utils

import android.text.TextUtils
import com.o3dr.services.android.lib.drone.attribute.error.ErrorType
import org.droidplanner.services.android.impl.core.model.AutopilotWarningParser
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import java.util.*

/**
 * Autopilot error parser.
 * Created by fhuya on 12/16/14.
 */
class AndroidApWarningParser : AutopilotWarningParser {
    override val defaultWarning: String?
        get() = ErrorType.NO_ERROR.name

    /**
     * Maps the ArduPilot warnings set to the 3DR Services warnings set.
     *
     * @param warning warning originating from the ArduPilot autopilot
     * @return equivalent 3DR Services warning type
     */
    override fun parseWarning(drone: MavLinkDrone?, warning: String?): String? {
        warning ?: return null
        val errorType = getErrorType(warning) ?: return null
        return errorType.name
    }

    private fun getErrorType(warning: String): ErrorType? {
        // TODO: This should be updated to handle prearm messages from > 2011..
        return when (warning.toLowerCase(Locale.US)) {
            "arm: thr below fs", "arm: throttle below failsafe" -> ErrorType.ARM_THROTTLE_BELOW_FAILSAFE
            "arm: gyro calibration failed" -> ErrorType.ARM_GYRO_CALIBRATION_FAILED
            "arm: mode not armable" -> ErrorType.ARM_MODE_NOT_ARMABLE
            "arm: rotor not spinning" -> ErrorType.ARM_ROTOR_NOT_SPINNING
            "arm: altitude disparity", "prearm: altitude disparity" -> ErrorType.ALTITUDE_DISPARITY
            "arm: leaning" -> ErrorType.ARM_LEANING
            "arm: throttle too high" -> ErrorType.ARM_THROTTLE_TOO_HIGH
            "arm: safety switch" -> ErrorType.ARM_SAFETY_SWITCH
            "arm: compass calibration running" -> ErrorType.ARM_COMPASS_CALIBRATION_RUNNING
            "prearm: rc not calibrated" -> ErrorType.PRE_ARM_RC_NOT_CALIBRATED
            "prearm: barometer not healthy" -> ErrorType.PRE_ARM_BAROMETER_NOT_HEALTHY
            "prearm: compass not healthy" -> ErrorType.PRE_ARM_COMPASS_NOT_HEALTHY
            "prearm: compass not calibrated" -> ErrorType.PRE_ARM_COMPASS_NOT_CALIBRATED
            "prearm: compass offsets too high" -> ErrorType.PRE_ARM_COMPASS_OFFSETS_TOO_HIGH
            "prearm: check mag field" -> ErrorType.PRE_ARM_CHECK_MAGNETIC_FIELD
            "prearm: inconsistent compasses" -> ErrorType.PRE_ARM_INCONSISTENT_COMPASSES
            "prearm: check fence" -> ErrorType.PRE_ARM_CHECK_FENCE
            "prearm: ins not calibrated" -> ErrorType.PRE_ARM_INS_NOT_CALIBRATED
            "prearm: accelerometers not healthy" -> ErrorType.PRE_ARM_ACCELEROMETERS_NOT_HEALTHY
            "prearm: inconsistent accelerometers" -> ErrorType.PRE_ARM_INCONSISTENT_ACCELEROMETERS
            "prearm: gyros not healthy" -> ErrorType.PRE_ARM_GYROS_NOT_HEALTHY
            "prearm: inconsistent gyros" -> ErrorType.PRE_ARM_INCONSISTENT_GYROS
            "prearm: check board voltage" -> ErrorType.PRE_ARM_CHECK_BOARD_VOLTAGE
            "prearm: duplicate aux switch options" -> ErrorType.PRE_ARM_DUPLICATE_AUX_SWITCH_OPTIONS
            "prearm: check fs_thr_value" -> ErrorType.PRE_ARM_CHECK_FAILSAFE_THRESHOLD_VALUE
            "prearm: check angle_max" -> ErrorType.PRE_ARM_CHECK_ANGLE_MAX
            "prearm: acro_bal_roll/pitch" -> ErrorType.PRE_ARM_ACRO_BAL_ROLL_PITCH
            "prearm: need 3d fix" -> ErrorType.PRE_ARM_NEED_GPS_LOCK
            "prearm: ekf-home variance" -> ErrorType.PRE_ARM_EKF_HOME_VARIANCE
            "prearm: high gps hdop" -> ErrorType.PRE_ARM_HIGH_GPS_HDOP
            "prearm: gps glitch", "prearm: bad velocity" -> ErrorType.PRE_ARM_GPS_GLITCH
            "prearm: waiting for navigation alignment", "arm: waiting for navigation alignment" -> ErrorType.WAITING_FOR_NAVIGATION_ALIGNMENT
            "no dataflash inserted" -> ErrorType.NO_DATAFLASH_INSERTED
            "low battery!" -> ErrorType.LOW_BATTERY
            "autotune: failed" -> ErrorType.AUTO_TUNE_FAILED
            "crash: disarming" -> ErrorType.CRASH_DISARMING
            "parachute: too low" -> ErrorType.PARACHUTE_TOO_LOW
            "ekf variance" -> ErrorType.EKF_VARIANCE
            "rc failsafe" -> ErrorType.RC_FAILSAFE
            else -> null
        }
    }
}
