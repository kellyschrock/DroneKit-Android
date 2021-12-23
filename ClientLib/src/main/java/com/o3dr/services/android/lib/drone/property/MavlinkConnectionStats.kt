package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

class MavlinkConnectionStats : DroneAttribute {
    var receivedCount = 0
    var crcErrorCount = 0
    var lostPacketCount = 0

    constructor() : super() {}

    operator fun set(received: Int, crcErrors: Int, droppedCount: Int) {
        receivedCount = received
        crcErrorCount = crcErrors
        lostPacketCount = droppedCount
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(receivedCount)
        dest.writeInt(crcErrorCount)
        dest.writeInt(lostPacketCount)
    }

    protected constructor(input: Parcel) {
        receivedCount = input.readInt()
        crcErrorCount = input.readInt()
        lostPacketCount = input.readInt()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<MavlinkConnectionStats> = object : Parcelable.Creator<MavlinkConnectionStats> {
            override fun createFromParcel(source: Parcel): MavlinkConnectionStats? {
                return MavlinkConnectionStats(source)
            }

            override fun newArray(size: Int): Array<MavlinkConnectionStats?> {
                return arrayOfNulls(size)
            }
        }
    }
}
