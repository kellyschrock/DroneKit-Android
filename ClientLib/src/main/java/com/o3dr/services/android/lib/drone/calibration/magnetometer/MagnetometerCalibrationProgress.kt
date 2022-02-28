package com.o3dr.services.android.lib.drone.calibration.magnetometer

import android.os.Parcel
import android.os.Parcelable

/**
 * Created by Fredia Huya-Kouadio on 5/3/15.
 */
class MagnetometerCalibrationProgress : Parcelable {
    var compassId = 0
        private set
    var completionPercentage = 0
        private set

    /**
     * Body frame direction vector for display
     */
    var directionX = 0f
    var directionY = 0f
    var directionZ = 0f

    constructor() {}
    constructor(compassId: Int, percentage: Int, directionX: Float, directionY: Float, directionZ: Float) {
        this.compassId = compassId
        completionPercentage = percentage
        this.directionX = directionX
        this.directionY = directionY
        this.directionZ = directionZ
    }

    fun setCompassId(compassId: Byte) {
        this.compassId = compassId.toInt()
    }

    fun setCompletionPercentage(completionPercentage: Byte) {
        this.completionPercentage = completionPercentage.toInt()
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(compassId)
        dest.writeInt(completionPercentage)
        dest.writeFloat(directionX)
        dest.writeFloat(directionY)
        dest.writeFloat(directionZ)
    }

    private constructor(input: Parcel) {
        compassId = input.readInt()
        completionPercentage = input.readInt()
        directionX = input.readFloat()
        directionY = input.readFloat()
        directionZ = input.readFloat()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<MagnetometerCalibrationProgress> = object : Parcelable.Creator<MagnetometerCalibrationProgress> {
            override fun createFromParcel(source: Parcel): MagnetometerCalibrationProgress? {
                return MagnetometerCalibrationProgress(source)
            }

            override fun newArray(size: Int): Array<MagnetometerCalibrationProgress?> {
                return arrayOfNulls(size)
            }
        }
    }
}
