package org.droidplanner.services.android.impl.utils.prefs

import android.content.Context
import android.content.SharedPreferences
import org.droidplanner.services.android.impl.core.firmware.FirmwareType
import org.droidplanner.services.android.impl.core.drone.profiles.VehicleProfile
import org.droidplanner.services.android.impl.utils.file.IO.VehicleProfileReader
import org.droidplanner.services.android.impl.core.drone.variables.StreamRates.Rates
import org.droidplanner.services.android.impl.utils.prefs.DroidPlannerPrefs
import android.preference.PreferenceManager
import org.droidplanner.services.android.impl.core.drone.Preferences

/**
 * Provides structured access to 3DR Services preferences
 *
 *
 * Over time it might be good to move the various places that are doing
 * prefs.getFoo(blah, default) here - to collect prefs in one place and avoid
 * duplicating string constants (which tend to become stale as code evolves).
 * This is called the DRY (don't repeat yourself) principle of software
 * development.
 */
class DroidPlannerPrefs(private val context: Context) : Preferences {
    private val prefs: SharedPreferences = PreferenceManager.getDefaultSharedPreferences(context)

    override fun loadVehicleProfile(firmwareType: FirmwareType?): VehicleProfile? {
        return VehicleProfileReader.load(context, firmwareType!!)
    }

    override val rates: Rates
        get() = Rates(DEFAULT_STREAM_RATE)

    /**
     * @return true if google analytics reporting is enabled.
     */
    val isUsageStatisticsEnabled: Boolean
        get() = prefs.getBoolean("pref_usage_statistics", true)

    companion object {
        const val DEFAULT_STREAM_RATE = 2 //Hz
    }
}
