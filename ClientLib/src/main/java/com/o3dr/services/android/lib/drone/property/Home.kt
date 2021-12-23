package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt

/**
 * Location from which the drone took off.
 */
class Home : DroneAttribute {
    /**
     * @return the launch pad 3D coordinate.
     */
    /**
     * Lauch pad 3D coordinate.
     */
    var coordinate: LatLongAlt? = null

    constructor() {}
    constructor(latitude: Double, longitude: Double, altitude: Double) {
        coordinate = LatLongAlt(latitude, longitude, altitude)
    }

    constructor(coordinate: LatLongAlt?) {
        this.coordinate = coordinate
    }

    val isValid: Boolean
        get() = coordinate != null

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Home) return false
        return !if (coordinate != null) coordinate != other.coordinate else other.coordinate != null
    }

    override fun hashCode(): Int {
        return if (coordinate != null) coordinate.hashCode() else 0
    }

    override fun toString(): String {
        return "LaunchPad{" +
                "mCoordinate=" + coordinate +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeParcelable(coordinate, 0)
    }

    private constructor(input: Parcel) {
        coordinate = input.readParcelable(LatLongAlt::class.java.classLoader)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Home> = object : Parcelable.Creator<Home> {
            override fun createFromParcel(source: Parcel): Home? {
                return Home(source)
            }

            override fun newArray(size: Int): Array<Home?> {
                return arrayOfNulls(size)
            }
        }
    }
}
