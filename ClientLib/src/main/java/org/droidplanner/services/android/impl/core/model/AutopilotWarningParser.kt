package org.droidplanner.services.android.impl.core.model

import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone

/**
 * Parse received autopilot warning messages.
 */
interface AutopilotWarningParser {
    val defaultWarning: String?
    fun parseWarning(drone: MavLinkDrone?, warning: String?): String?
}
