package org.droidplanner.services.android.impl.core.MAVLink

import android.os.Handler
import android.util.Log
import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.common.*
import org.droidplanner.services.android.impl.core.MAVLink.WaypointManager
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnWaypointManagerListener
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import timber.log.Timber
import java.util.*

/**
 * Class to manage the communication of waypoints to the MAV.
 *
 *
 * Should be initialized with a MAVLink Object, so the manager can send messages
 * via the MAV link. The function processMessage must be called with every new
 * MAV Message.
 */
class WaypointManager(drone: MavLinkDrone?, private val watchdog: Handler)
: DroneVariable<MavLinkDrone?>(drone) {

    enum class WaypointStates {
        IDLE, WAITING_CLEAR_ACK, READ_REQUEST, READING_WP, WRITING_WP_COUNT, WRITING_WP, WAITING_WRITE_ACK
    }

    enum class WaypointEvent_Type {
        WP_UPLOAD, WP_DOWNLOAD, WP_RETRY, WP_CONTINUE, WP_TIMED_OUT
    }

    private var retryTracker = 0
    private var readIndex = 0
    private var writeIndex = 0
    private var retryIndex = 0
    private var wpEventListener: OnWaypointManagerListener? = null
    var state = WaypointStates.IDLE
    private val watchdogCallback: Runnable = object : Runnable {
        override fun run() {
            if (processTimeOut(++retryTracker)) watchdog.postDelayed(this, TIMEOUT)
        }
    }

    fun setWaypointManagerListener(wpEventListener: OnWaypointManagerListener?) {
        this.wpEventListener = wpEventListener
    }

    private fun startWatchdog(delay: Long) {
        stopWatchdog()
        retryTracker = 0
        watchdog.postDelayed(watchdogCallback, delay)
    }

    private fun stopWatchdog() {
        watchdog.removeCallbacks(watchdogCallback)
    }// ensure that WPManager is not doing anything else

    /**
     * Try to receive all waypoints from the MAV.
     *
     *
     * If all runs well the callback will return the list of waypoints.
     */
    val waypoints: Unit
        get() {
            // ensure that WPManager is not doing anything else
            if (state != WaypointStates.IDLE) return
            doBeginWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD)
            readIndex = -1
            state = WaypointStates.READ_REQUEST
            MavLinkWaypoint.requestWaypointsList(myDrone)
            startWatchdog(TIMEOUT)
        }

    /**
     * Write a list of waypoints to the MAV.
     *
     *
     * The callback will return the status of this operation
     *
     * @param data waypoints to be written
     */
    fun writeWaypoints(data: List<msg_mission_item>?) {
        Log.v(TAG, "writeWaypoints(): state=$state")

        // ensure that WPManager is not doing anything else
        if (state != WaypointStates.IDLE) {
            Log.w(TAG, "Resetting state to IDLE from $state")
            state = WaypointStates.IDLE
        }

//        if (state != WaypointStates.IDLE)
//            return;
        if (mission != null) {
            doBeginWaypointEvent(WaypointEvent_Type.WP_UPLOAD)
            mission.clear()
            mission.addAll(data!!)
            sendClearAllMessage()
            startWatchdog(CLEAR_TIMEOUT)
        }
    }

    /**
     * Sets the current waypoint in the MAV
     *
     *
     * The callback will return the status of this operation
     */
    fun setCurrentWaypoint(i: Int) {
        if (mission != null) {
            MavLinkWaypoint.sendSetCurrentWaypoint(myDrone, i.toShort())
        }
    }

    private fun onGotClearAck() {
        Log.v(TAG, "onGotClearAck()")
        writeIndex = 0
        state = WaypointStates.WRITING_WP_COUNT
        Log.v(TAG, "sendWaypointCount()")
        MavLinkWaypoint.sendWaypointCount(myDrone, mission!!.size)
        startWatchdog(TIMEOUT)
    }

    /**
     * Callback for when a waypoint has been reached
     *
     * @param wpNumber number of the completed waypoint
     */
    fun onWaypointReached(wpNumber: Int) {}

    /**
     * Callback for a change in the current waypoint the MAV is heading for
     *
     * @param seq number of the updated waypoint
     */
    private fun onCurrentWaypointUpdate(seq: Int) {}

    /**
     * number of waypoints to be received, used when reading waypoints
     */
    private var waypointCount = 0

    /**
     * list of waypoints used when writing or receiving
     */
    private val mission: MutableList<msg_mission_item>? = ArrayList()

    /**
     * Try to process a Mavlink message if it is a mission related message
     *
     * @param msg Mavlink message to process
     * @return Returns true if the message has been processed
     */
    fun processMessage(msg: MAVLinkMessage): Boolean {
        when (state) {
            WaypointStates.IDLE -> {}
            WaypointStates.READ_REQUEST -> if (msg.msgid == msg_mission_count.MAVLINK_MSG_ID_MISSION_COUNT) {
                waypointCount = (msg as msg_mission_count).count
                mission!!.clear()
                startWatchdog(TIMEOUT)
                MavLinkWaypoint.requestWayPoint(myDrone, mission.size)
                state = WaypointStates.READING_WP
                return true
            }
            WaypointStates.READING_WP -> if (msg.msgid == msg_mission_item.MAVLINK_MSG_ID_MISSION_ITEM) {
                startWatchdog(TIMEOUT)
                processReceivedWaypoint(msg as msg_mission_item)
                doWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD, readIndex + 1, waypointCount)
                if (mission!!.size < waypointCount) {
                    MavLinkWaypoint.requestWayPoint(myDrone, mission.size)
                } else {
                    stopWatchdog()
                    state = WaypointStates.IDLE
                    MavLinkWaypoint.sendAck(myDrone)
                    myDrone?.mission?.onMissionReceived(mission)
                    doEndWaypointEvent(WaypointEvent_Type.WP_DOWNLOAD)
                }
                return true
            }
            WaypointStates.WRITING_WP_COUNT -> {
                state = WaypointStates.WRITING_WP
                if (msg.msgid == msg_mission_request.MAVLINK_MSG_ID_MISSION_REQUEST) {
                    Log.v(TAG, "got MISSION_REQUEST")
                    startWatchdog(TIMEOUT)
                    processWaypointToSend(msg as msg_mission_request)
                    doWaypointEvent(WaypointEvent_Type.WP_UPLOAD, writeIndex + 1, mission!!.size)
                    return true
                }
            }
            WaypointStates.WRITING_WP -> if (msg.msgid == msg_mission_request.MAVLINK_MSG_ID_MISSION_REQUEST) {
                Log.v(TAG, "got MISSION_REQUEST")
                startWatchdog(TIMEOUT)
                processWaypointToSend(msg as msg_mission_request)
                doWaypointEvent(WaypointEvent_Type.WP_UPLOAD, writeIndex + 1, mission!!.size)
                return true
            }
            WaypointStates.WAITING_CLEAR_ACK -> {
                if (msg.msgid == msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK) {
                    Log.v(TAG, "got MISSION_ACK for CLEAR")
                    onGotClearAck()
                    return true
                }
            }
            WaypointStates.WAITING_WRITE_ACK -> {
                if (msg.msgid == msg_mission_ack.MAVLINK_MSG_ID_MISSION_ACK) {
                    Log.v(TAG, "got MISSION_ACK for WRITE")
                    stopWatchdog()
                    myDrone?.mission?.onWriteWaypoints(msg as msg_mission_ack)
                    state = WaypointStates.IDLE
                    doEndWaypointEvent(WaypointEvent_Type.WP_UPLOAD)
                    return true
                }
            }
            else -> {}
        }
        if (msg.msgid == msg_mission_item_reached.MAVLINK_MSG_ID_MISSION_ITEM_REACHED) {
            onWaypointReached((msg as msg_mission_item_reached).seq)
            return true
        }
        if (msg.msgid == msg_mission_current.MAVLINK_MSG_ID_MISSION_CURRENT) {
            onCurrentWaypointUpdate((msg as msg_mission_current).seq)
            return true
        }
        return false
    }

    fun processTimeOut(timeoutCount: Int): Boolean {
        Log.v(TAG, String.format("processTimeout(), state=%s", state))

        // If max retry is reached, set state to IDLE. No more retry.
        if (timeoutCount >= RETRY_LIMIT) {
            Log.v(TAG, "Done with retries for timeouts")
            state = WaypointStates.IDLE
            doWaypointEvent(WaypointEvent_Type.WP_TIMED_OUT, retryIndex, RETRY_LIMIT)
            return false
        }
        retryIndex++
        doWaypointEvent(WaypointEvent_Type.WP_RETRY, retryIndex, RETRY_LIMIT)
        when (state) {
            WaypointStates.IDLE -> {}
            WaypointStates.READ_REQUEST -> MavLinkWaypoint.requestWaypointsList(myDrone)
            WaypointStates.READING_WP -> if (mission!!.size < waypointCount) { // request last lost WP
                MavLinkWaypoint.requestWayPoint(myDrone, mission.size)
            }
            WaypointStates.WRITING_WP_COUNT -> MavLinkWaypoint.sendWaypointCount(myDrone, mission!!.size)
            WaypointStates.WRITING_WP ->                 // Log.d("TIMEOUT", "re Write Msg: " + String.valueOf(writeIndex));
                if (writeIndex < mission!!.size) {
                    myDrone?.mavClient?.sendMessage(mission[writeIndex], null)
                }
            WaypointStates.WAITING_CLEAR_ACK -> {
                Log.v(TAG, "Timed out waiting for clear ack, just send the mission items")
                onGotClearAck()
            }
            WaypointStates.WAITING_WRITE_ACK -> myDrone?.mavClient?.sendMessage(mission!![mission.size - 1], null)
            else -> {}
        }
        return true
    }

    private fun sendClearAllMessage() {
        Log.v(TAG, "sendClearAllMessage()")
        state = WaypointStates.WAITING_CLEAR_ACK
        val msg = msg_mission_clear_all()
        msg.isMavlink2 = false
        msg.mission_type = 0
        myDrone?.mavClient?.sendMessage(msg, null)
    }

    private fun processWaypointToSend(msg: msg_mission_request) {
        /*
         * Log.d("TIMEOUT", "Write Msg: " + String.valueOf(msg.seq));
		 */
        writeIndex = msg.seq
        val item = mission!![writeIndex]
        item.isMavlink2 = false
        item.mission_type = 0
        myDrone?.let {
            item.target_system = it.sysid
            item.target_component = it.compid
            Timber.d("send item %s", item)
            it.mavClient?.sendMessage(item, null)
            if (writeIndex + 1 >= mission.size) {
                state = WaypointStates.WAITING_WRITE_ACK
            }
        }
    }

    private fun processReceivedWaypoint(msg: msg_mission_item) {
        /*
		 * Log.d("TIMEOUT", "Read Last/Curr: " + String.valueOf(readIndex) + "/"
		 * + String.valueOf(msg.seq));
		 */
        // in case of we receive the same WP again after retry
        if (msg.seq <= readIndex) return
        readIndex = msg.seq
        mission!!.add(msg)
    }

    private fun doBeginWaypointEvent(wpEvent: WaypointEvent_Type) {
        retryIndex = 0
        if (wpEventListener == null) return
        wpEventListener!!.onBeginWaypointEvent(wpEvent)
    }

    private fun doEndWaypointEvent(wpEvent: WaypointEvent_Type) {
        if (retryIndex > 0) // if retry successful, notify that we now continue
            doWaypointEvent(WaypointEvent_Type.WP_CONTINUE, retryIndex, RETRY_LIMIT)
        retryIndex = 0
        if (wpEventListener == null) return
        wpEventListener!!.onEndWaypointEvent(wpEvent)
    }

    private fun doWaypointEvent(wpEvent: WaypointEvent_Type, index: Int, count: Int) {
        retryIndex = 0
        if (wpEventListener == null) return
        wpEventListener!!.onWaypointEvent(wpEvent, index, count)
    }

    companion object {
        val TAG = WaypointManager::class.java.simpleName
        private const val TIMEOUT: Long = 7000 //ms
        private const val CLEAR_TIMEOUT: Long = 3000
        private const val RETRY_LIMIT = 3
    }
}
