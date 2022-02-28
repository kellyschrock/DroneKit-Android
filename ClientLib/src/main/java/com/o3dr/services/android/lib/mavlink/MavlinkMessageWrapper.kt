package com.o3dr.services.android.lib.mavlink

import android.os.Parcelable
import android.os.Parcel
import com.MAVLink.Messages.MAVLinkMessage

/**
 * Wrapper class for a mavlink message, allowing it to be transmitted over android IPC mechanism.
 */
class MavlinkMessageWrapper : Parcelable {
    var mavLinkMessage: MAVLinkMessage

    constructor(mavlinkMsg: MAVLinkMessage) {
        mavLinkMessage = mavlinkMsg
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeSerializable(mavLinkMessage)
    }

    private constructor(input: Parcel) {
        mavLinkMessage = input.readSerializable() as MAVLinkMessage
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<MavlinkMessageWrapper> = object : Parcelable.Creator<MavlinkMessageWrapper> {
            override fun createFromParcel(source: Parcel): MavlinkMessageWrapper? {
                return MavlinkMessageWrapper(source)
            }

            override fun newArray(size: Int): Array<MavlinkMessageWrapper?> {
                return arrayOfNulls(size)
            }
        }
    }
}
