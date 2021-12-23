package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

/**
 * Created by fhuya on 10/28/14.
 */
class Altitude : DroneAttribute {
    var altitude = 0.0
    var targetAltitude = 0.0

    constructor() {}
    constructor(altitude: Double, targetAltitude: Double) {
        this.altitude = altitude
        this.targetAltitude = targetAltitude
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is Altitude) return false
        val altitude1 = o
        if (java.lang.Double.compare(altitude1.altitude, altitude) != 0) return false
        return if (java.lang.Double.compare(altitude1.targetAltitude, targetAltitude) != 0) false else true
    }

    override fun hashCode(): Int {
        var result: Int
        var temp: Long = java.lang.Double.doubleToLongBits(altitude)
        result = (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(targetAltitude)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun toString(): String {
        return "Altitude{" +
                "altitude=" + altitude +
                ", targetAltitude=" + targetAltitude +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(altitude)
        dest.writeDouble(targetAltitude)
    }

    private constructor(input: Parcel) {
        altitude = input.readDouble()
        targetAltitude = input.readDouble()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Altitude> = object : Parcelable.Creator<Altitude> {
            override fun createFromParcel(source: Parcel): Altitude {
                return Altitude(source)
            }

            override fun newArray(size: Int): Array<Altitude?> {
                return arrayOfNulls(size)
            }
        }
    }
}
