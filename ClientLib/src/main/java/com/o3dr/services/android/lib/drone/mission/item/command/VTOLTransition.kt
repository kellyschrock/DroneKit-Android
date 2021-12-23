package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTransition.TargetState
import com.o3dr.services.android.lib.drone.mission.item.command.VTOLTransition
import android.os.Parcel
import com.MAVLink.enums.MAV_VTOL_STATE
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * MAV_CMD_DO_VTOL_TRANSITION
 *
 * Start a VTOL transition either to fixed-wing or multi-copter state
 */
class VTOLTransition : MissionItem, MissionItem.Command, Parcelable {
    enum class TargetState(val state: Int) {
        Undefined(MAV_VTOL_STATE.MAV_VTOL_STATE_UNDEFINED),
        MultiCopter(MAV_VTOL_STATE.MAV_VTOL_STATE_MC),
        FixedWing(MAV_VTOL_STATE.MAV_VTOL_STATE_FW)
        ;

        companion object {
            @JvmStatic
            fun fromOrdinal(o: Int): TargetState {
                for (s in values()) {
                    if (s.ordinal == o) {
                        return s
                    }
                }
                return Undefined
            }

            @JvmStatic
            fun fromValue(v: Int): TargetState {
                for (s in values()) {
                    if (s.state == v) {
                        return s
                    }
                }
                return Undefined
            }
        }
    }

    var targetState = TargetState.Undefined

    constructor() : super(MissionItemType.VTOL_TRANSITION) {}
    constructor(src: VTOLTransition) : super(MissionItemType.VTOL_TRANSITION) {
        targetState = src.targetState
    }

    constructor(state: TargetState) : this() {
        targetState = state
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeInt(targetState.ordinal)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        targetState = TargetState.fromOrdinal(`in`.readInt())
    }

    override fun clone(): MissionItem {
        return VTOLTransition(this)
    }

    override fun toString(): String {
        return javaClass.simpleName + "{" +
                "targetState=" + targetState +
                '}'
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<VTOLTransition> = object : Parcelable.Creator<VTOLTransition> {
            override fun createFromParcel(source: Parcel): VTOLTransition? {
                return VTOLTransition(source)
            }

            override fun newArray(size: Int): Array<VTOLTransition?> {
                return arrayOfNulls(size)
            }
        }
    }
}
