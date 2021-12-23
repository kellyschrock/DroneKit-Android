package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

/**
 * Created by fhuya on 10/28/14.
 */
class Attitude : DroneAttribute {
    /** Roll angle (deg, -180..+180) */
    var roll = 0.0

    /** Roll angular speed (deg/s) */
    var rollSpeed = 0f
    /** Pitch angle (deg, -180 to 180) */
    var pitch = 0.0

    /** Pitch angular speed (deg / s) */
    var pitchSpeed = 0f

    /** Yaw angle (deg, -180 to 180) */
    var yaw = 0.0

    /** Yaw angular speed (deg/ s) */
    var yawSpeed = 0f

    constructor() {}
    constructor(roll: Double, pitch: Double, yaw: Double, rollSpeed: Float, pitchSpeed: Float, yawSpeed: Float) {
        this.roll = roll
        this.pitch = pitch
        this.yaw = yaw
        this.rollSpeed = rollSpeed
        this.pitchSpeed = pitchSpeed
        this.yawSpeed = yawSpeed
    }

    override fun toString(): String {
        return "Attitude{" +
                "pitch=" + pitch +
                ", roll=" + roll +
                ", rollSpeed=" + rollSpeed +
                ", pitchSpeed=" + pitchSpeed +
                ", yaw=" + yaw +
                ", yawSpeed=" + yawSpeed +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is Attitude) return false
        if (java.lang.Double.compare(o.roll, roll) != 0) return false
        if (java.lang.Float.compare(o.rollSpeed, rollSpeed) != 0) return false
        if (java.lang.Double.compare(o.pitch, pitch) != 0) return false
        if (java.lang.Float.compare(o.pitchSpeed, pitchSpeed) != 0) return false
        return if (java.lang.Double.compare(o.yaw, yaw) != 0) false else java.lang.Float.compare(o.yawSpeed, yawSpeed) == 0
    }

    override fun hashCode(): Int {
        var result: Int
        var temp: Long = java.lang.Double.doubleToLongBits(roll)
        result = (temp xor (temp ushr 32)).toInt()
        result = 31 * result + if (rollSpeed != +0.0f) java.lang.Float.floatToIntBits(rollSpeed) else 0
        temp = java.lang.Double.doubleToLongBits(pitch)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + if (pitchSpeed != +0.0f) java.lang.Float.floatToIntBits(pitchSpeed) else 0
        temp = java.lang.Double.doubleToLongBits(yaw)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + if (yawSpeed != +0.0f) java.lang.Float.floatToIntBits(yawSpeed) else 0
        return result
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(roll)
        dest.writeDouble(pitch)
        dest.writeDouble(yaw)
        dest.writeFloat(rollSpeed)
        dest.writeFloat(pitchSpeed)
        dest.writeFloat(yawSpeed)
    }

    private constructor(input: Parcel) {
        roll = input.readDouble()
        pitch = input.readDouble()
        yaw = input.readDouble()
        rollSpeed = input.readFloat()
        pitchSpeed = input.readFloat()
        yawSpeed = input.readFloat()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Attitude> = object : Parcelable.Creator<Attitude> {
            override fun createFromParcel(source: Parcel): Attitude {
                return Attitude(source)
            }

            override fun newArray(size: Int): Array<Attitude?> {
                return arrayOfNulls(size)
            }
        }
    }
}
