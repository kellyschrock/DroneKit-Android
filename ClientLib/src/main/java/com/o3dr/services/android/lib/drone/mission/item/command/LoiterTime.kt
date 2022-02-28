package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.item.command.LoiterTime
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * MAV_CMD_NAV_LOITER_TIME
 *
 * Begin loiter at the specified coordinate, and don't consider this waypoint complete
 * until the specified delay in seconds has elapsed.
 */
class LoiterTime : MissionItem, MissionItem.Command, Parcelable {
    var coordinate: LatLongAlt? = null
    var delay: Long = 0
    var radius = 0.0

    constructor() : super(MissionItemType.LOITER_TIME) {}
    constructor(src: LoiterTime) : super(MissionItemType.LOITER_TIME) {
        coordinate = src.coordinate
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(coordinate!!.latitude)
        dest.writeDouble(coordinate!!.longitude)
        dest.writeDouble(coordinate!!.altitude)
        dest.writeLong(delay)
        dest.writeDouble(radius)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        val lat = `in`.readDouble()
        val lng = `in`.readDouble()
        val alt = `in`.readDouble()
        coordinate = LatLongAlt(lat, lng, alt)
        delay = `in`.readLong()
        radius = `in`.readDouble()
    }

    override fun clone(): MissionItem {
        return LoiterTime(this)
    }

    override fun toString(): String {
        return "LoiterTime{" +
                "coordinate=" + coordinate +
                ", delay=" + delay +
                ", radius=" + radius +
                '}'
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<LoiterTime> = object : Parcelable.Creator<LoiterTime> {
            override fun createFromParcel(source: Parcel): LoiterTime? {
                return LoiterTime(source)
            }

            override fun newArray(size: Int): Array<LoiterTime?> {
                return arrayOfNulls(size)
            }
        }
    }
}
