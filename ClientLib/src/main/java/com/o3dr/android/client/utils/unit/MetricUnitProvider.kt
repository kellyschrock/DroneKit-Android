package com.o3dr.android.client.utils.unit

import com.o3dr.android.client.utils.unit.UnitProvider
import java.util.*

/**
 * Unit provider implementation for the metric system.
 */
class MetricUnitProvider : UnitProvider {
    override fun areaToString(areaInSqMeters: Double): String {
        val absArea = Math.abs(areaInSqMeters)
        return if (absArea >= 100000) {
            String.format(Locale.US, "%2.1f km" + UnitProvider.SQUARE_SYMBOL, areaInSqMeters / 1000000)
        } else if (absArea >= 1) {
            String.format(Locale.US, "%2.1f m" + UnitProvider.SQUARE_SYMBOL, areaInSqMeters)
        } else if (absArea >= 0.00001) {
            String.format(Locale.US, "%2.2f cm" + UnitProvider.SQUARE_SYMBOL, areaInSqMeters * 10000)
        } else {
            areaInSqMeters.toString() + " m" + UnitProvider.SQUARE_SYMBOL
        }
    }

    override fun distanceToString(distanceInMeters: Double): String {
        val absDistance = Math.abs(distanceInMeters)
        return if (absDistance >= 1000) {
            String.format(Locale.US, "%2.1f km", distanceInMeters / 1000)
        } else if (absDistance >= 0.1) {
            String.format(Locale.US, "%2.1f m", distanceInMeters)
        } else if (absDistance >= 0.001) {
            String.format(Locale.US, "%2.1f mm", distanceInMeters * 1000)
        } else {
            "$distanceInMeters m"
        }
    }

    override fun speedToString(speedInMetersPerSeconds: Double): String {
        return String.format(Locale.US, "%2.1f m/s", speedInMetersPerSeconds)
    }

    override fun electricChargeToString(chargeInmAh: Double): String {
        val absCharge = Math.abs(chargeInmAh)
        return if (absCharge >= 1000) {
            String.format(Locale.US, "%2.0f Ah", chargeInmAh / 1000)
        } else {
            String.format(Locale.ENGLISH, "%2.0f mAh", chargeInmAh)
        }
    }
}
