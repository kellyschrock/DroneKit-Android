package com.o3dr.services.android.lib.coordinate

import android.os.Parcel
import android.os.Parcelable

/** Stores latitude, longitude, and altitude information for a coordinate. */
class LatLongAlt : LatLong {
    var altitude: Double

    constructor(latitude: Double, longitude: Double, altitude: Double) : super(latitude, longitude) {
        this.altitude = altitude
    }

    constructor(location: LatLong?, altitude: Double) : super(location!!) {
        this.altitude = altitude
    }

    constructor(copy: LatLongAlt) : this(copy.latitude, copy.longitude, copy.altitude) {}

    fun set(source: LatLongAlt) {
        super.set(source)
        altitude = source.altitude
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is LatLongAlt) return false
        if (!super.equals(o)) return false
        return if (java.lang.Double.compare(o.altitude, altitude) != 0) false else true
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        val temp: Long = java.lang.Double.doubleToLongBits(altitude)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun toString(): String {
        val superToString = super.toString()
        return "LatLongAlt{" +
                superToString +
                ", mAltitude=" + altitude +
                '}'
    }

    companion object {
        private const val serialVersionUID = -4771550293045623743L

        @JvmField
        val CREATOR: Parcelable.Creator<LatLongAlt> = object : Parcelable.Creator<LatLongAlt> {
            override fun createFromParcel(source: Parcel): LatLongAlt? {
                return source.readSerializable() as LatLongAlt
            }

            override fun newArray(size: Int): Array<LatLongAlt?> {
                return arrayOfNulls(size)
            }
        }
    }
}
