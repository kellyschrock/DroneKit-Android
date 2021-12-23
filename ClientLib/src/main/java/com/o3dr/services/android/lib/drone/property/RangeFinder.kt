package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import java.util.*

class RangeFinder : DroneAttribute {
    var distance = 0f
    var voltage = 0f
    val sensors: Map<Int, DistanceSensor> = HashMap()

    constructor() : super() {}

    override fun toString(): String {
        return "RangeFinder{" +
                "distance=" + distance +
                ", voltage=" + voltage +
                ", sensors=" + sensors +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeFloat(distance)
        dest.writeFloat(voltage)
    }

    private constructor(input: Parcel) {
        distance = input.readFloat()
        voltage = input.readFloat()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<RangeFinder> = object : Parcelable.Creator<RangeFinder> {
            override fun createFromParcel(source: Parcel): RangeFinder? {
                return RangeFinder(source)
            }

            override fun newArray(size: Int): Array<RangeFinder?> {
                return arrayOfNulls(size)
            }
        }
    }
}
