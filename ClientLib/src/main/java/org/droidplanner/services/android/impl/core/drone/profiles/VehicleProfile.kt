package org.droidplanner.services.android.impl.core.drone.profiles

class VehicleProfile {
    var parameterMetadataType: String? = null
    var default = Default()

    class Default {
        var wpNavSpeed = 0
        var maxAltitude = 0
    }
}
