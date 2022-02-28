package org.droidplanner.services.android.impl.core.gcs

import com.MAVLink.Messages.MAVLinkMessage
import com.MAVLink.common.msg_heartbeat
import com.MAVLink.enums.MAV_AUTOPILOT
import com.MAVLink.enums.MAV_TYPE
import org.droidplanner.services.android.impl.communication.model.DataLink.DataLinkProvider
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit

/**
 * This class is used to send periodic heartbeat messages to the drone.
 */
class GCSHeartbeat(private val dataLink: DataLinkProvider<MAVLinkMessage>,
                   /**
                    * This is the heartbeat period in seconds.
                    */
                   private val period: Int) {
    companion object {
        /**
         * This is the msg heartbeat used to check the drone is present, and
         * responding.
         */
        private val heartbeatMsg = msg_heartbeat()

        init {
            heartbeatMsg.type = MAV_TYPE.MAV_TYPE_GCS.toShort()
            heartbeatMsg.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC.toShort()
        }
    }

    /**
     * ScheduledExecutorService used to periodically schedule the heartbeat.
     */
    private var heartbeatExecutor: ScheduledExecutorService? = null

    /**
     * Runnable used to send the heartbeat.
     */
    private val heartbeatRunnable = Runnable { dataLink.sendMessage(heartbeatMsg, null) }

    /**
     * Set the state of the heartbeat.
     *
     * @param active true to activate the heartbeat, false to deactivate it
     */
    @Synchronized
    fun setActive(active: Boolean) {
        if (active) {
            if (heartbeatExecutor == null || heartbeatExecutor!!.isShutdown) {
                heartbeatExecutor = Executors.newSingleThreadScheduledExecutor().apply {
                    scheduleWithFixedDelay(heartbeatRunnable, 0, period.toLong(), TimeUnit.SECONDS)
                }
            }
        } else if (heartbeatExecutor != null && !heartbeatExecutor!!.isShutdown) {
            heartbeatExecutor!!.shutdownNow()
            heartbeatExecutor = null
        }
    }
}
