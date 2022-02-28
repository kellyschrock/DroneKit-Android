package org.droidplanner.services.android.impl.core.drone.profiles

import java.io.Serializable

class ParameterMetadata : Serializable {
    var name: String? = null
    var displayName: String? = null
    var description: String? = null
    var units: String? = null
    var range: String? = null
    var values: String? = null

    override fun toString(): String {
        return "ParameterMetadata{" +
                "name='" + name + '\'' +
                ", displayName='" + displayName + '\'' +
                ", description='" + description + '\'' +
                ", units='" + units + '\'' +
                ", range='" + range + '\'' +
                ", values='" + values + '\'' +
                '}'
    }
}
