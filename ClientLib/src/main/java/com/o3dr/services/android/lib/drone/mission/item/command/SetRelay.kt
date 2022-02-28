package com.o3dr.services.android.lib.drone.mission.item.command

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem

/**
 * Set a Relay pin’s voltage high or low.
 */
class SetRelay : MissionItem, MissionItem.Command, Parcelable {
    /**
     * @return relay number
     */
    /**
     * Set the relay number
     *
     * @param relayNumber
     */
    var relayNumber = 0
    /**
     * @return true if relay is on, false if relay if off.
     */
    /**
     * @param enabled true for relay to be on, false for relay to be off.
     */
    var isEnabled = false

    constructor() : super(MissionItemType.SET_RELAY) {}
    constructor(copy: SetRelay) : this() {
        relayNumber = copy.relayNumber
        isEnabled = copy.isEnabled
    }

    override fun clone(): MissionItem {
        return SetRelay(this)
    }

    override fun toString(): String {
        return "SetRelay{" +
                "enabled=" + isEnabled +
                ", relayNumber=" + relayNumber +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is SetRelay) return false
        if (!super.equals(o)) return false
        val setRelay = o
        return if (relayNumber != setRelay.relayNumber) false else isEnabled == setRelay.isEnabled
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        result = 31 * result + relayNumber
        result = 31 * result + if (isEnabled) 1 else 0
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeInt(relayNumber)
        dest.writeByte(if (isEnabled) 1.toByte() else 0.toByte())
    }

    private constructor(`in`: Parcel) : super(`in`) {
        relayNumber = `in`.readInt()
        isEnabled = `in`.readByte().toInt() != 0
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<SetRelay> = object : Parcelable.Creator<SetRelay> {
            override fun createFromParcel(source: Parcel): SetRelay? {
                return SetRelay(source)
            }

            override fun newArray(size: Int): Array<SetRelay?> {
                return arrayOfNulls(size)
            }
        }
    }
}
