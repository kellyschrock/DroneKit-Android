package com.o3dr.services.android.lib.drone.mission.item.command

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * MAV_CMD_NAV_VTOL_LANDING
 *
 * Land in VTOL mode
 */
class VTOLLand : MissionItem, MissionItem.Command, Parcelable {
    var approachAltitude = 0.0
    var yawAngle = 0.0
    var coordinate: LatLongAlt? = null

    constructor() : super(MissionItemType.VTOL_LAND) {}
    constructor(src: VTOLLand) : super(MissionItemType.VTOL_TAKEOFF) {
        approachAltitude = src.approachAltitude
        yawAngle = src.yawAngle
        coordinate = src.coordinate
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeDouble(approachAltitude)
        dest.writeDouble(yawAngle)
        dest.writeDouble(coordinate!!.latitude)
        dest.writeDouble(coordinate!!.longitude)
        dest.writeDouble(coordinate!!.altitude)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        approachAltitude = `in`.readDouble()
        yawAngle = `in`.readDouble()
        val lat = `in`.readDouble()
        val lng = `in`.readDouble()
        val alt = `in`.readDouble()
        coordinate = LatLongAlt(lat, lng, alt)
    }

    override fun clone(): MissionItem {
        return VTOLLand(this)
    }

    override fun toString(): String {
        return javaClass.simpleName + "{" +
                "approachAltitude=" + approachAltitude +
                ", yawAngle=" + yawAngle +
                ", coordinate=" + coordinate +
                '}'
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<VTOLLand> = object : Parcelable.Creator<VTOLLand> {
            override fun createFromParcel(source: Parcel): VTOLLand? {
                return VTOLLand(source)
            }

            override fun newArray(size: Int): Array<VTOLLand?> {
                return arrayOfNulls(size)
            }
        }
    }
}
