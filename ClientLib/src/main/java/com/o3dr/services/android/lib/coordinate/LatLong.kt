package com.o3dr.services.android.lib.coordinate

import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLong
import android.os.Parcel
import java.io.Serializable

/** Stores latitude and longitude in degrees. */
open class LatLong(
    var latitude: Double,
    var longitude: Double) : Parcelable, Serializable {

    constructor(copy: LatLong) : this(copy.latitude, copy.longitude) {}

    fun set(update: LatLong) {
        latitude = update.latitude
        longitude = update.longitude
    }

    fun dot(scalar: Double): LatLong {
        return LatLong(latitude * scalar, longitude * scalar)
    }

    fun negate(): LatLong {
        return LatLong(latitude * -1, longitude * -1)
    }

    fun subtract(coord: LatLong): LatLong {
        return LatLong(latitude - coord.latitude, longitude - coord.longitude)
    }

    fun sum(coord: LatLong): LatLong {
        return LatLong(latitude + coord.latitude, longitude + coord.longitude)
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is LatLong) return false
        val latLong = o
        if (java.lang.Double.compare(latLong.latitude, latitude) != 0) return false
        return if (java.lang.Double.compare(latLong.longitude, longitude) != 0) false else true
    }

    override fun hashCode(): Int {
        var result: Int
        var temp: Long
        temp = java.lang.Double.doubleToLongBits(latitude)
        result = (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(longitude)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun toString(): String {
        return "LatLong{" +
                "latitude=" + latitude +
                ", longitude=" + longitude +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeSerializable(this)
    }

    companion object {
        private const val serialVersionUID = -5809863197722412339L

        @JvmStatic
        fun sum(vararg toBeAdded: LatLong): LatLong {
            var latitude = 0.0
            var longitude = 0.0
            for (coord in toBeAdded) {
                latitude += coord.latitude
                longitude += coord.longitude
            }
            return LatLong(latitude, longitude)
        }

        @JvmField
        val CREATOR: Parcelable.Creator<LatLong> = object : Parcelable.Creator<LatLong> {
            override fun createFromParcel(source: Parcel): LatLong? {
                return source.readSerializable() as LatLong
            }

            override fun newArray(size: Int): Array<LatLong?> {
                return arrayOfNulls(size)
            }
        }
    }
}
