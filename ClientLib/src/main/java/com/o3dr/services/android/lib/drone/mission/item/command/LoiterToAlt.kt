package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.item.command.LoiterToAlt
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * MAV_CMD_NAV_LOITER_TO_ALT
 *
 * Begin loiter at the specified coordinate, and don't consider this waypoint complete
 * until the target coordinate's altitude has been reached.
 */
class LoiterToAlt : MissionItem, MissionItem.Command, Parcelable {
    var coordinate: LatLongAlt? = null

    constructor() : super(MissionItemType.LOITER_TO_ALT) {}
    constructor(src: LoiterToAlt) : super(MissionItemType.LOITER_TO_ALT) {
        coordinate = src.coordinate
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(coordinate!!.latitude)
        dest.writeDouble(coordinate!!.longitude)
        dest.writeDouble(coordinate!!.altitude)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        val lat = `in`.readDouble()
        val lng = `in`.readDouble()
        val alt = `in`.readDouble()
        coordinate = LatLongAlt(lat, lng, alt)
    }

    override fun clone(): MissionItem {
        return LoiterToAlt(this)
    }

    override fun toString(): String {
        return "LoiterToAlt{" +
                "coordinate=" + coordinate +
                '}'
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<LoiterToAlt> = object : Parcelable.Creator<LoiterToAlt> {
            override fun createFromParcel(source: Parcel): LoiterToAlt? {
                return LoiterToAlt(source)
            }

            override fun newArray(size: Int): Array<LoiterToAlt?> {
                return arrayOfNulls(size)
            }
        }
    }
}
