package com.o3dr.services.android.lib.drone.mission.item.complex

import android.os.Parcelable
import android.os.Parcel

/**
 * Created by fhuya on 11/6/14.
 */
class CameraDetail : Parcelable {
    val name: String?
    val sensorWidth: Double
    val sensorHeight: Double
    val sensorResolution: Double
    val focalLength: Double
    val overlap: Double
    val sidelap: Double
    val isInLandscapeOrientation: Boolean

    constructor() {
        name = "Canon SX260"
        sensorWidth = 6.12
        sensorHeight = 4.22
        sensorResolution = 12.1
        focalLength = 5.0
        overlap = 50.0
        sidelap = 60.0
        isInLandscapeOrientation = true
    }

    constructor(name: String?, sensorWidth: Double, sensorHeight: Double, sensorResolution: Double,
                focalLength: Double, overlap: Double, sidelap: Double,
                isInLandscapeOrientation: Boolean) {
        this.name = name
        this.sensorWidth = sensorWidth
        this.sensorHeight = sensorHeight
        this.sensorResolution = sensorResolution
        this.focalLength = focalLength
        this.overlap = overlap
        this.sidelap = sidelap
        this.isInLandscapeOrientation = isInLandscapeOrientation
    }

    constructor(copy: CameraDetail) : this(copy.name, copy.sensorWidth, copy.sensorHeight, copy.sensorResolution, copy.focalLength, copy.overlap,
            copy.sidelap, copy.isInLandscapeOrientation) {
    }

    val sensorLateralSize: Double
        get() = if (isInLandscapeOrientation) {
            sensorWidth
        } else {
            sensorHeight
        }

    val sensorLongitudinalSize: Double
        get() = if (isInLandscapeOrientation) {
            sensorHeight
        } else {
            sensorWidth
        }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is CameraDetail) return false
        if (java.lang.Double.compare(o.focalLength, focalLength) != 0) return false
        if (isInLandscapeOrientation != o.isInLandscapeOrientation) return false
        if (java.lang.Double.compare(o.overlap, overlap) != 0) return false
        if (java.lang.Double.compare(o.sensorHeight, sensorHeight) != 0) return false
        if (java.lang.Double.compare(o.sensorResolution, sensorResolution) != 0) return false
        if (java.lang.Double.compare(o.sensorWidth, sensorWidth) != 0) return false
        if (java.lang.Double.compare(o.sidelap, sidelap) != 0) return false
        return if (if (name != null) name != o.name else o.name != null) false else true
    }

    override fun hashCode(): Int {
        var result: Int = name?.hashCode() ?: 0
        var temp: Long = java.lang.Double.doubleToLongBits(sensorWidth)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(sensorHeight)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(sensorResolution)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(focalLength)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(overlap)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        temp = java.lang.Double.doubleToLongBits(sidelap)
        result = 31 * result + (temp xor (temp ushr 32)).toInt()
        result = 31 * result + if (isInLandscapeOrientation) 1 else 0
        return result
    }

    override fun toString(): String {
        return "CameraDetail{" +
                "name='" + name + '\'' +
                ", sensorWidth=" + sensorWidth +
                ", sensorHeight=" + sensorHeight +
                ", sensorResolution=" + sensorResolution +
                ", focalLength=" + focalLength +
                ", overlap=" + overlap +
                ", sidelap=" + sidelap +
                ", isInLandscapeOrientation=" + isInLandscapeOrientation +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeString(name)
        dest.writeDouble(sensorWidth)
        dest.writeDouble(sensorHeight)
        dest.writeDouble(sensorResolution)
        dest.writeDouble(focalLength)
        dest.writeDouble(overlap)
        dest.writeDouble(sidelap)
        dest.writeByte(if (isInLandscapeOrientation) 1.toByte() else 0.toByte())
    }

    private constructor(input: Parcel) {
        name = input.readString()
        sensorWidth = input.readDouble()
        sensorHeight = input.readDouble()
        sensorResolution = input.readDouble()
        focalLength = input.readDouble()
        overlap = input.readDouble()
        sidelap = input.readDouble()
        isInLandscapeOrientation = input.readByte().toInt() != 0
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<CameraDetail> = object : Parcelable.Creator<CameraDetail> {
            override fun createFromParcel(source: Parcel): CameraDetail? {
                return CameraDetail(source)
            }

            override fun newArray(size: Int): Array<CameraDetail?> {
                return arrayOfNulls(size)
            }
        }
    }
}
