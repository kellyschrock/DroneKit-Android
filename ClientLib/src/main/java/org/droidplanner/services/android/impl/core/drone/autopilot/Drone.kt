package org.droidplanner.services.android.impl.core.drone.autopilot

import com.o3dr.services.android.lib.drone.property.DroneAttribute
import com.o3dr.services.android.lib.model.ICommandListener
import com.o3dr.services.android.lib.model.action.Action
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.*

interface Drone {
    val id: String?
    val isConnected: Boolean
    fun getAttribute(attributeType: String?): DroneAttribute?
    fun executeAsyncAction(action: Action?, listener: ICommandListener?): Boolean
    fun setAttributeListener(listener: AttributeEventListener?)
    fun destroy()
    fun addDroneListener(listener: OnDroneListener<*>?)
    fun removeDroneListener(listener: OnDroneListener<*>?)
    fun notifyDroneEvent(event: DroneEventsType?)
}
