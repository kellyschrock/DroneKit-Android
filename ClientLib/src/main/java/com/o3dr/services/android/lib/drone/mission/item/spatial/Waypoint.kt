package com.o3dr.services.android.lib.drone.mission.item.spatial

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/6/14.
 */
class Waypoint : BaseSpatialItem, Parcelable {
    var delay = 0.0
    var acceptanceRadius = 0.0
    var yawAngle = 0.0
    var orbitalRadius = 0.0
    var isOrbitCCW = false

    constructor() : super(MissionItemType.WAYPOINT) {}

    constructor(copy: Waypoint) : super(copy) {
        delay = copy.delay
        acceptanceRadius = copy.acceptanceRadius
        yawAngle = copy.yawAngle
        orbitalRadius = copy.orbitalRadius
        isOrbitCCW = copy.isOrbitCCW
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(delay)
        dest.writeDouble(acceptanceRadius)
        dest.writeDouble(yawAngle)
        dest.writeDouble(orbitalRadius)
        dest.writeByte(if (isOrbitCCW) 1.toByte() else 0.toByte())
    }

    private constructor(`in`: Parcel) : super(`in`) {
        delay = `in`.readDouble()
        acceptanceRadius = `in`.readDouble()
        yawAngle = `in`.readDouble()
        orbitalRadius = `in`.readDouble()
        isOrbitCCW = `in`.readByte().toInt() != 0
    }

    override fun toString(): String {
        return "Waypoint{" +
                "acceptanceRadius=" + acceptanceRadius +
                ", delay=" + delay +
                ", yawAngle=" + yawAngle +
                ", orbitalRadius=" + orbitalRadius +
                ", orbitCCW=" + isOrbitCCW +
                ", " + super.toString() +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is Waypoint) return false
        if (!super.equals(o)) return false
        if (java.lang.Double.compare(o.delay, delay) != 0) return false
        if (java.lang.Double.compare(o.acceptanceRadius, acceptanceRadius) != 0) return false
        if (java.lang.Double.compare(o.yawAngle, yawAngle) != 0) return false
        return if (java.lang.Double.compare(o.orbitalRadius, orbitalRadius) != 0) false else isOrbitCCW == o.isOrbitCCW
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        var temp: Long
        temp = java.lang.Double.doubleToLongBits(delay)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(acceptanceRadius)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(yawAngle)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(orbitalRadius)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + if (isOrbitCCW) 1 else 0
        return result
    }

    override fun clone(): MissionItem {
        return Waypoint(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Waypoint> = object : Parcelable.Creator<Waypoint> {
            override fun createFromParcel(source: Parcel): Waypoint? {
                return Waypoint(source)
            }

            override fun newArray(size: Int): Array<Waypoint?> {
                return arrayOfNulls(size)
            }
        }
    }
}
