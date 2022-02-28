package org.droidplanner.services.android.impl.core.drone

import org.droidplanner.services.android.impl.core.drone.profiles.VehicleProfile
import org.droidplanner.services.android.impl.core.drone.variables.StreamRates.Rates
import org.droidplanner.services.android.impl.core.firmware.FirmwareType

interface Preferences {
    fun loadVehicleProfile(firmwareType: FirmwareType?): VehicleProfile?
    val rates: Rates?
}
