package org.droidplanner.services.android.impl.core.drone.autopilot.apm

/**
 * Set of APM autopilots related constants.
 * Created by Fredia Huya-Kouadio on 7/28/15.
 */
object APMConstants {
    /**
     * Index of the home waypoint within a mission items list.
     */
    const val HOME_WAYPOINT_INDEX = 0

    /**
     * Severity levels used in STATUSTEXT messages
     */
    object Severity {
        const val SEVERITY_LOW = 1
        const val SEVERITY_MEDIUM = 2
        const val SEVERITY_HIGH = 3
        const val SEVERITY_CRITICAL = 4
        const val SEVERITY_USER_RESPONSE = 5
    }
}
