package org.droidplanner.services.android.impl.core.drone

import android.os.Handler
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean

class DroneEvents(myDrone: MavLinkDrone, private val handler: Handler) : DroneVariable<MavLinkDrone>(myDrone) {
    private val isDispatcherRunning = AtomicBoolean(false)

    private val eventDispatcher: Runnable = object : Runnable {
        override fun run() {
            handler.removeCallbacks(this)
            val event = eventQueue.poll()
            if (event == null) {
                isDispatcherRunning.set(false)
                return
            }

            for (listener in droneListeners) {
                listener.onDroneEvent(event, myDrone)
            }
            handler.removeCallbacks(this)
            handler.postDelayed(this, EVENT_DISPATCHING_DELAY)
            isDispatcherRunning.set(true)
        }
    }

    private val droneListeners = ConcurrentLinkedQueue<OnDroneListener<MavLinkDrone>>()
    private val eventQueue = ConcurrentLinkedQueue<DroneEventsType>()

    fun addDroneListener(listener: OnDroneListener<MavLinkDrone>) {
        if (listener != null && !droneListeners.contains(listener)) {
            droneListeners.add(listener)
        }
    }

    fun removeDroneListener(listener: OnDroneListener<*>?) {
        if (listener != null && droneListeners.contains(listener)) droneListeners.remove(listener)
    }

    fun removeAllDroneListeners() {
        droneListeners.clear()
    }

    fun notifyDroneEvent(event: DroneEventsType?) {
        if (event == null || droneListeners.isEmpty() || eventQueue.contains(event)) return
        eventQueue.add(event)
        if (isDispatcherRunning.compareAndSet(false, true)) {
            handler.postDelayed(eventDispatcher, EVENT_DISPATCHING_DELAY)
        }
    }

    companion object {
        private const val EVENT_DISPATCHING_DELAY = 33L //milliseconds
    }
}
