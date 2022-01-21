package org.droidplanner.services.android.impl.core.drone.variables.calibration

import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.ardupilotmega.msg_mag_cal_progress
import com.MAVLink.ardupilotmega.msg_mag_cal_report
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCalibration
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import java.util.*
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Created by Fredia Huya-Kouadio on 5/3/15.
 */
class MagnetometerCalibrationImpl(myDrone: MavLinkDrone) : DroneVariable<MavLinkDrone?>(myDrone), OnDroneListener<MavLinkDrone?> {
    interface OnMagnetometerCalibrationListener {
        fun onCalibrationCancelled()
        fun onCalibrationProgress(progress: msg_mag_cal_progress?)
        fun onCalibrationCompleted(result: msg_mag_cal_report?)
    }

    val magCalibrationTracker = HashMap<Short, Info>()
    private var listener: OnMagnetometerCalibrationListener? = null
    private val cancelled = AtomicBoolean(false)

    fun setListener(listener: OnMagnetometerCalibrationListener?) {
        this.listener = listener
    }

    fun startCalibration(retryOnFailure: Boolean, saveAutomatically: Boolean, startDelay: Int) {
        magCalibrationTracker.clear()
        cancelled.set(false)
        MavLinkCalibration.startMagnetometerCalibration(myDrone, retryOnFailure, saveAutomatically, startDelay, null)
    }

    fun cancelCalibration() {
        MavLinkCalibration.cancelMagnetometerCalibration(myDrone, null)
        cancelled.set(true)
        if (listener != null) listener!!.onCalibrationCancelled()
    }

    fun acceptCalibration() {
        MavLinkCalibration.acceptMagnetometerCalibration(myDrone, null)
    }

    fun processCalibrationMessage(message: MAVLinkMessage) {
        when (message.msgid) {
            msg_mag_cal_progress.MAVLINK_MSG_ID_MAG_CAL_PROGRESS -> {
                val progress = message as msg_mag_cal_progress
                var info = magCalibrationTracker[progress.compass_id]
                if (info == null) {
                    info = Info()
                    magCalibrationTracker[progress.compass_id] = info
                }
                info.calProgress = progress
                if (listener != null) listener!!.onCalibrationProgress(progress)
            }
            msg_mag_cal_report.MAVLINK_MSG_ID_MAG_CAL_REPORT -> {
                val report = message as msg_mag_cal_report
                var info = magCalibrationTracker[report.compass_id]
                if (info == null) {
                    info = Info()
                    magCalibrationTracker[report.compass_id] = info
                }
                info.calReport = report
                if (listener != null) listener!!.onCalibrationCompleted(message)
            }
        }
    }

    fun isCancelled(): Boolean {
        return cancelled.get()
    }

    class Info {
        var calProgress: msg_mag_cal_progress? = null
        var calReport: msg_mag_cal_report? = null
    }

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.HEARTBEAT_TIMEOUT, DroneEventsType.DISCONNECTED -> cancelCalibration()
        }
    }

    init {
        myDrone.addDroneListener(this)
    }
}
