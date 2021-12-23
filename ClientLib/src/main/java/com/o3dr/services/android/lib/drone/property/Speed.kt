package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

/** Created by fhuya on 10/28/14. */
class Speed : DroneAttribute {
    var verticalSpeed // m/s
            = 0.0
    var groundSpeed // m/s
            = 0.0
    var airSpeed // m/s
            = 0.0

    constructor() {}

    constructor(verticalSpeed: Double, groundSpeed: Double, airSpeed: Double) {
        this.verticalSpeed = verticalSpeed
        this.groundSpeed = groundSpeed
        this.airSpeed = airSpeed
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is Speed) return false
        val speed = o
        if (java.lang.Double.compare(speed.airSpeed, airSpeed) != 0) return false
        if (java.lang.Double.compare(speed.groundSpeed, groundSpeed) != 0) return false
        return if (java.lang.Double.compare(speed.verticalSpeed, verticalSpeed) != 0) false else true
    }

    override fun hashCode(): Int {
        var result: Int
        var temp: Long = java.lang.Double.doubleToLongBits(verticalSpeed)
        result = (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(groundSpeed)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(airSpeed)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun toString(): String {
        return "Speed{" +
                "verticalSpeed=" + verticalSpeed +
                ", groundSpeed=" + groundSpeed +
                ", airSpeed=" + airSpeed +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(verticalSpeed)
        dest.writeDouble(groundSpeed)
        dest.writeDouble(airSpeed)
    }

    private constructor(input: Parcel) {
        verticalSpeed = input.readDouble()
        groundSpeed = input.readDouble()
        airSpeed = input.readDouble()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Speed> = object : Parcelable.Creator<Speed> {
            override fun createFromParcel(source: Parcel): Speed? {
                return Speed(source)
            }

            override fun newArray(size: Int): Array<Speed?> {
                return arrayOfNulls(size)
            }
        }
    }
}
