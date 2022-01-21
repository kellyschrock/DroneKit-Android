package org.droidplanner.services.android.impl.core.drone.variables.calibration

import android.os.Handler
import android.os.RemoteException
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.common.msg_statustext
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.SimpleCommandListener
import org.droidplanner.services.android.impl.core.MAVLink.MavLinkCalibration
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import timber.log.Timber
import java.util.concurrent.atomic.AtomicReference

class AccelCalibration(drone: MavLinkDrone, private val handler: Handler) : DroneVariable<MavLinkDrone?>(drone), OnDroneListener<MavLinkDrone?> {
    private val onCalibrationStart = Runnable {
        val listener = listenerRef.getAndSet(null)
        if (listener != null) {
            try {
                listener.onSuccess()
            } catch (e: RemoteException) {
                Timber.e(e, e.message)
            }
        }
    }
    var message: String? = null
        private set
    var isCalibrating = false
        private set
    private val listenerRef = AtomicReference<ICommandListener?>(null)
    fun startCalibration(listener: ICommandListener?) {
        if (isCalibrating) {
            if (listener != null) {
                try {
                    listener.onSuccess()
                } catch (e: RemoteException) {
                    Timber.e(e, e.message)
                }
            }
            return
        }

        if (true == myDrone?.state?.isFlying) {
            isCalibrating = false
        } else {
            isCalibrating = true
            message = ""
            listenerRef.set(listener)
            MavLinkCalibration.startAccelerometerCalibration(myDrone, object : SimpleCommandListener() {
                override fun onSuccess() {
                    listenerRef.getAndSet(null)?.let { listener ->
                        try {
                            listener.onSuccess()
                        } catch (e: RemoteException) {
                            Timber.e(e, e.message)
                        }
                    }
                }

                override fun onError(executionError: Int) {
                    listenerRef.getAndSet(null)?.let { listener ->
                        try {
                            listener.onError(executionError)
                        } catch (e: RemoteException) {
                            Timber.e(e, e.message)
                        }
                    }
                }

                override fun onTimeout() {
                    listenerRef.getAndSet(null)?.let { listener ->
                        try {
                            listener.onTimeout()
                        } catch (e: RemoteException) {
                            Timber.e(e, e.message)
                        }
                    }
                }
            })
        }
    }

    fun sendAck(step: Int) {
        if (isCalibrating) MavLinkCalibration.sendCalibrationAckMessage(myDrone, step)
    }

    fun processMessage(msg: MAVLinkMessage) {
        if (isCalibrating && msg.msgid == msg_statustext.MAVLINK_MSG_ID_STATUSTEXT) {
            val statusMsg = msg as msg_statustext
            val message = statusMsg.getText()
            if (message != null && (message.startsWith("Place vehicle") || message.startsWith("Calibration"))) {
                handler.post(onCalibrationStart)
                this.message = message
                if (message.startsWith("Calibration")) isCalibrating = false
                myDrone?.notifyDroneEvent(DroneEventsType.CALIBRATION_IMU)
            }
        }
    }

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.HEARTBEAT_TIMEOUT, DroneEventsType.DISCONNECTED -> if (isCalibrating) cancelCalibration()
        }
    }

    fun cancelCalibration() {
        message = ""
        isCalibrating = false
    }

    init {
        drone.addDroneListener(this)
    }
}
