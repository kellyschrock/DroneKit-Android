package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.command.Takeoff
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * The vehicle will climb straight up from it’s current location to the altitude specified (in meters).
 * This should be the first command of nearly all missions.
 * If the mission is begun while the copter is already flying, the vehicle will climb straight up to the specified altitude.
 * If the vehicle is already above the specified altitude the takeoff command will be ignored and the mission will move onto the next command immediately.
 *
 * Created by fhuya on 11/6/14.
 */
class Takeoff : MissionItem, MissionItem.Command, Parcelable {
    /**
     * @return take off altitude in meters
     */
    /**
     * Sets the take off altitude
     * @param takeoffAltitude Altitude value in meters
     */
    var takeoffAltitude = 0.0
    var takeoffPitch = 0.0

    constructor() : super(MissionItemType.TAKEOFF) {}
    constructor(copy: Takeoff) : this() {
        takeoffAltitude = copy.takeoffAltitude
        takeoffPitch = copy.takeoffPitch
    }

    override fun toString(): String {
        return "Takeoff{" +
                "takeoffAltitude=" + takeoffAltitude +
                ", takeoffPitch=" + takeoffPitch +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is Takeoff) return false
        if (!super.equals(o)) return false
        val takeoff = o
        return if (java.lang.Double.compare(takeoff.takeoffAltitude, takeoffAltitude) != 0) false else java.lang.Double.compare(takeoff.takeoffPitch, takeoffPitch) == 0
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        var temp: Long
        temp = java.lang.Double.doubleToLongBits(takeoffAltitude)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(takeoffPitch)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(takeoffAltitude)
        dest.writeDouble(takeoffPitch)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        takeoffAltitude = `in`.readDouble()
        takeoffPitch = `in`.readDouble()
    }

    override fun clone(): MissionItem {
        return Takeoff(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Takeoff> = object : Parcelable.Creator<Takeoff> {
            override fun createFromParcel(source: Parcel): Takeoff? {
                return Takeoff(source)
            }

            override fun newArray(size: Int): Array<Takeoff?> {
                return arrayOfNulls(size)
            }
        }
    }
}
