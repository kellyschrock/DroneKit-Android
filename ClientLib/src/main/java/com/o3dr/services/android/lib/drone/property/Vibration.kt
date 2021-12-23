package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

/**
 * Reports the vehicle vibration levels and accelerometer clipping.
 * Created by Fredia Huya-Kouadio on 9/14/15.
 */
class Vibration : DroneAttribute {
    /*
    Vibration levels thresholds:
    - Good <= 30
    - 30 < Warning <= 60
    - 60 < Danger
     */
    /**
     * Vibration levels on the x-axis.
     */
    var vibrationX = 0f

    /**
     * Vibration levels on the y-axis.
     */
    var vibrationY = 0f

    /**
     * Vibration levels on the z-axis.
     */
    var vibrationZ = 0f

    /**
     * First accelerometer clipping count.
     */
    var firstAccelClipping: Long = 0

    /**
     * Second accelerometer clipping count.
     */
    var secondAccelClipping: Long = 0

    /**
     * Third accelerometer clipping count.
     */
    var thirdAccelClipping: Long = 0

    constructor() {}

    constructor(firstAccelClipping: Long, secondAccelClipping: Long, thirdAccelClipping: Long, vibrationX: Float, vibrationY: Float, vibrationZ: Float) {
        this.firstAccelClipping = firstAccelClipping
        this.secondAccelClipping = secondAccelClipping
        this.thirdAccelClipping = thirdAccelClipping
        this.vibrationX = vibrationX
        this.vibrationY = vibrationY
        this.vibrationZ = vibrationZ
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Vibration) return false
        if (java.lang.Float.compare(other.vibrationX, vibrationX) != 0) return false
        if (java.lang.Float.compare(other.vibrationY, vibrationY) != 0) return false
        if (java.lang.Float.compare(other.vibrationZ, vibrationZ) != 0) return false
        if (firstAccelClipping != other.firstAccelClipping) return false
        return if (secondAccelClipping != other.secondAccelClipping) false else thirdAccelClipping == other.thirdAccelClipping
    }

    override fun hashCode(): Int {
        var result = if (vibrationX != +0.0f) java.lang.Float.floatToIntBits(vibrationX) else 0
        result = 31 * result + if (vibrationY != +0.0f) java.lang.Float.floatToIntBits(vibrationY) else 0
        result = 31 * result + if (vibrationZ != +0.0f) java.lang.Float.floatToIntBits(vibrationZ) else 0
        result = 31 * result + (firstAccelClipping xor (firstAccelClipping ushr 32)).toInt()
        result = 31 * result + (secondAccelClipping xor (secondAccelClipping ushr 32)).toInt()
        result = 31 * result + (thirdAccelClipping xor (thirdAccelClipping ushr 32)).toInt()
        return result
    }

    override fun toString(): String {
        return "Vibration{" +
                "firstAccelClipping=" + firstAccelClipping +
                ", vibrationX=" + vibrationX +
                ", vibrationY=" + vibrationY +
                ", vibrationZ=" + vibrationZ +
                ", secondAccelClipping=" + secondAccelClipping +
                ", thirdAccelClipping=" + thirdAccelClipping +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeFloat(vibrationX)
        dest.writeFloat(vibrationY)
        dest.writeFloat(vibrationZ)
        dest.writeLong(firstAccelClipping)
        dest.writeLong(secondAccelClipping)
        dest.writeLong(thirdAccelClipping)
    }

    protected constructor(input: Parcel) {
        vibrationX = input.readFloat()
        vibrationY = input.readFloat()
        vibrationZ = input.readFloat()
        firstAccelClipping = input.readLong()
        secondAccelClipping = input.readLong()
        thirdAccelClipping = input.readLong()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Vibration> = object : Parcelable.Creator<Vibration> {
            override fun createFromParcel(source: Parcel): Vibration? {
                return Vibration(source)
            }

            override fun newArray(size: Int): Array<Vibration?> {
                return arrayOfNulls(size)
            }
        }
    }
}
