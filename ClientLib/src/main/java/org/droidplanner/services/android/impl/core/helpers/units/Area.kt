package org.droidplanner.services.android.impl.core.helpers.units

import java.util.*

class Area(private var areaInSqMeters: Double) {
    fun valueInSqMeters(): Double {
        return areaInSqMeters
    }

    fun set(areaInSqMeters: Double) {
        this.areaInSqMeters = areaInSqMeters
    }

    override fun toString(): String {
        return if (areaInSqMeters >= 100000) {
            String.format(Locale.US, "%2.1f km" + SQUARE_SYMBOL, areaInSqMeters / 1000000)
        } else if (areaInSqMeters >= 1) {
            String.format(Locale.US, "%2.1f m" + SQUARE_SYMBOL, areaInSqMeters)
        } else if (areaInSqMeters >= 0.00001) {
            String.format(Locale.US, "%2.2f cm" + SQUARE_SYMBOL, areaInSqMeters * 10000)
        } else {
            areaInSqMeters.toString() + " m" + SQUARE_SYMBOL
        }
    }

    companion object {
        const val SQUARE_SYMBOL = "\u00B2"
    }
}
