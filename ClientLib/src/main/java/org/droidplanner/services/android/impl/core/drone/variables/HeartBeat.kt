package org.droidplanner.services.android.impl.core.drone.variables

import android.os.Handler
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.common.msg_heartbeat
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import timber.log.Timber

open class HeartBeat(myDrone: MavLinkDrone, val watchdog: Handler) : DroneVariable<MavLinkDrone?>(myDrone), OnDroneListener<MavLinkDrone?> {
    protected var heartbeatState = FIRST_HEARTBEAT
    var sysid: Short = 1
        private set
    var compid: Short = 1
        private set

    var mavlinkVersion = INVALID_MAVLINK_VERSION.toShort()
        private set

    private val watchdogCallback = Runnable { onHeartbeatTimeout() }

    fun onHeartbeat(msg: MAVLinkMessage) {
        val heartBeatMsg = if (msg is msg_heartbeat) msg else null
        if (heartBeatMsg != null) {
            sysid = validateToUnsignedByteRange(msg.sysid)
            compid = validateToUnsignedByteRange(msg.compid)
            mavlinkVersion = heartBeatMsg.mavlink_version
        }
        when (heartbeatState) {
            FIRST_HEARTBEAT -> if (heartBeatMsg != null) {
                Timber.i("Received first heartbeat.")
                heartbeatState = NORMAL_HEARTBEAT
                restartWatchdog(HEARTBEAT_NORMAL_TIMEOUT)
                myDrone?.notifyDroneEvent(DroneEventsType.HEARTBEAT_FIRST)
            }
            LOST_HEARTBEAT -> {
                myDrone?.notifyDroneEvent(DroneEventsType.HEARTBEAT_RESTORED)
                heartbeatState = NORMAL_HEARTBEAT
                restartWatchdog(HEARTBEAT_NORMAL_TIMEOUT)
            }
            else -> {
                heartbeatState = NORMAL_HEARTBEAT
                restartWatchdog(HEARTBEAT_NORMAL_TIMEOUT)
            }
        }
    }

    fun hasHeartbeat(): Boolean {
        return heartbeatState != FIRST_HEARTBEAT
    }

    val isConnectionAlive: Boolean
        get() = heartbeatState != LOST_HEARTBEAT

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.DISCONNECTED -> notifyDisconnected()
            else -> {}
        }
    }

    private fun notifyDisconnected() {
        watchdog.removeCallbacks(watchdogCallback)
        heartbeatState = FIRST_HEARTBEAT
        mavlinkVersion = INVALID_MAVLINK_VERSION.toShort()
    }

    protected open fun onHeartbeatTimeout() {
        when (heartbeatState) {
            FIRST_HEARTBEAT -> {
                Timber.i("First heartbeat timeout.")
                myDrone?.notifyDroneEvent(DroneEventsType.HEARTBEAT_TIMEOUT)
            }
            else -> {
                heartbeatState = LOST_HEARTBEAT
                restartWatchdog(HEARTBEAT_LOST_TIMEOUT)
                myDrone?.notifyDroneEvent(DroneEventsType.HEARTBEAT_TIMEOUT)
            }
        }
    }

    protected fun restartWatchdog(timeout: Long) {
        // re-start watchdog
        watchdog.removeCallbacks(watchdogCallback)
        watchdog.postDelayed(watchdogCallback, timeout)
    }

    companion object {
        const val HEARTBEAT_NORMAL_TIMEOUT = 5000L //ms
        private const val HEARTBEAT_LOST_TIMEOUT = 15000L //ms
        const val INVALID_MAVLINK_VERSION = -1
        protected const val FIRST_HEARTBEAT = 0
        protected const val LOST_HEARTBEAT = 1
        protected const val NORMAL_HEARTBEAT = 2
    }

    init {
        myDrone.addDroneListener(this)
    }
}
