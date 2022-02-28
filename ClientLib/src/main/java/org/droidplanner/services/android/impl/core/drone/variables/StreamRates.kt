package org.droidplanner.services.android.impl.core.drone.variables

import org.droidplanner.services.android.impl.core.MAVLink.MavLinkStreamRates
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.OnDroneListener
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

class StreamRates(myDrone: MavLinkDrone) : DroneVariable<MavLinkDrone?>(myDrone), OnDroneListener<MavLinkDrone?> {
    private var rates: Rates? = null
    fun setRates(rates: Rates?) {
        this.rates = rates
    }

    override fun onDroneEvent(event: DroneEventsType, drone: MavLinkDrone?) {
        when (event) {
            DroneEventsType.CONNECTED, DroneEventsType.HEARTBEAT_FIRST, DroneEventsType.HEARTBEAT_RESTORED -> setupStreamRatesFromPref()
            else -> {}
        }
    }

    fun setupStreamRatesFromPref() {
        rates ?: return
        myDrone ?: return
        
        MavLinkStreamRates.setupStreamRates(myDrone!!.mavClient, myDrone!!.sysid,
                myDrone!!.compid, rates!!.extendedStatus, rates!!.extra1, rates!!.extra2,
                rates!!.extra3, rates!!.position, rates!!.rcChannels, rates!!.rawSensors,
                rates!!.rawController)
    }

    class Rates {
        var extendedStatus = 0
        var extra1 = 0
        var extra2 = 0
        var extra3 = 0
        var position = 0
        var rcChannels = 0
        var rawSensors = 0
        var rawController = 0

        constructor() {}
        constructor(rate: Int) {
            extendedStatus = rate
            extra1 = rate
            extra2 = rate
            extra3 = rate
            position = rate
            rcChannels = rate
            rawSensors = rate
            rawController = rate
        }
    }

    init {
        myDrone.addDroneListener(this)
    }
}
