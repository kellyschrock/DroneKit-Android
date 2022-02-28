package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

/** Stores information about the drone's type. */
class Type : DroneAttribute {
    enum class Firmware(val label: String) {
        ARDU_PLANE("ArduPlane"),
        ARDU_COPTER("ArduCopter"),
        APM_ROVER("APMRover")
        ;

    }

    var droneType = TYPE_UNKNOWN
    var firmwareVersion: String? = null
    var firmware: Firmware? = null

    constructor() {}

    constructor(droneType: Int, firmwareVersion: String?) {
        this.droneType = droneType
        this.firmwareVersion = firmwareVersion
        firmware = when (droneType) {
            TYPE_COPTER -> Firmware.ARDU_COPTER
            TYPE_PLANE, TYPE_VTOL -> Firmware.ARDU_PLANE
            TYPE_ROVER -> Firmware.APM_ROVER
            TYPE_UNKNOWN -> null
            else -> null
        }
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(droneType)
        dest.writeString(firmwareVersion)
        dest.writeInt(if (firmware == null) -1 else firmware!!.ordinal)
    }

    private constructor(input: Parcel) {
        droneType = input.readInt()
        firmwareVersion = input.readString()
        val tmpFirmware = input.readInt()
        firmware = if (tmpFirmware == -1) null else Firmware.values()[tmpFirmware]
    }

    companion object {
        const val TYPE_UNKNOWN = -1
        const val TYPE_PLANE = 1
        const val TYPE_COPTER = 2
        const val TYPE_ROVER = 10
        const val TYPE_VTOL = 12

        @JvmField
        val CREATOR: Parcelable.Creator<Type> = object : Parcelable.Creator<Type> {
            override fun createFromParcel(source: Parcel): Type? {
                return Type(source)
            }

            override fun newArray(size: Int): Array<Type?> {
                return arrayOfNulls(size)
            }
        }
    }
}
