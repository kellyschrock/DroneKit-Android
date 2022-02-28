package com.o3dr.services.android.lib.drone.mission.item.spatial

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Created by fhuya on 11/6/14.
 */
class SplineWaypoint : BaseSpatialItem, Parcelable {
    /**
     * Hold time in decimal seconds. (ignored by fixed wing, time to stay at
     * MISSION for rotary wing)
     */
    var delay = 0.0

    constructor() : super(MissionItemType.SPLINE_WAYPOINT) {}
    constructor(copy: SplineWaypoint) : super(copy) {
        delay = copy.delay
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(delay)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        delay = `in`.readDouble()
    }

    override fun toString(): String {
        return "SplineWaypoint{" +
                "delay=" + delay +
                ", " + super.toString() +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is SplineWaypoint) return false
        if (!super.equals(o)) return false
        return java.lang.Double.compare(o.delay, delay) == 0
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        val temp: Long = java.lang.Double.doubleToLongBits(delay)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        return result
    }

    override fun clone(): MissionItem {
        return SplineWaypoint(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<SplineWaypoint> = object : Parcelable.Creator<SplineWaypoint> {
            override fun createFromParcel(source: Parcel): SplineWaypoint? {
                return SplineWaypoint(source)
            }

            override fun newArray(size: Int): Array<SplineWaypoint?> {
                return arrayOfNulls(size)
            }
        }
    }
}
