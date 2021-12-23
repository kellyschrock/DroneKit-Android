package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTakeoff.TransitionHeading
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTakeoff
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * MAV_CMD_NAV_VTOL_TAKEOFF
 *
 * Takeoff in VTOL mode
 */
class VTOLTakeoff : MissionItem, MissionItem.Command, Parcelable {
    enum class TransitionHeading(val value: Int) {
        Default(0), NextWaypoint(1), TakeoffHeading(2), UseSpecified(3), AnyHeading(4);

        companion object {
            @JvmStatic
            fun fromValue(value: Int): TransitionHeading {
                for (t in values()) {
                    if (t.value == value) {
                        return t
                    }
                }
                return Default
            }
        }
    }

    var transitionHeading = TransitionHeading.Default
    var yawAngle = 0.0
    var coordinate: LatLongAlt? = null

    constructor() : super(MissionItemType.VTOL_TAKEOFF) {}
    constructor(src: VTOLTakeoff) : super(MissionItemType.VTOL_TAKEOFF) {
        transitionHeading = src.transitionHeading
        yawAngle = src.yawAngle
        coordinate = src.coordinate
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeInt(transitionHeading.value)
        dest.writeDouble(yawAngle)
        dest.writeDouble(coordinate!!.latitude)
        dest.writeDouble(coordinate!!.longitude)
        dest.writeDouble(coordinate!!.altitude)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        transitionHeading = TransitionHeading.fromValue(`in`.readInt())
        yawAngle = `in`.readDouble()
        val lat = `in`.readDouble()
        val lng = `in`.readDouble()
        val alt = `in`.readDouble()
        coordinate = LatLongAlt(lat, lng, alt)
    }

    override fun clone(): MissionItem {
        return VTOLTakeoff(this)
    }

    override fun toString(): String {
        return javaClass.simpleName + "{" +
                "transitionHeading=" + transitionHeading +
                ", yawAngle=" + yawAngle +
                ", coordinate=" + coordinate +
                '}'
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<VTOLTakeoff> = object : Parcelable.Creator<VTOLTakeoff> {
            override fun createFromParcel(source: Parcel): VTOLTakeoff? {
                return VTOLTakeoff(source)
            }

            override fun newArray(size: Int): Array<VTOLTakeoff?> {
                return arrayOfNulls(size)
            }
        }
    }
}
