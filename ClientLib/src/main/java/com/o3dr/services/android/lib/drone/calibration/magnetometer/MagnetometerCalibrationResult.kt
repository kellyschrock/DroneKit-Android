package com.o3dr.services.android.lib.drone.calibration.magnetometer

import android.os.Parcel
import android.os.Parcelable

/** Created by Fredia Huya-Kouadio on 5/3/15. */
class MagnetometerCalibrationResult : Parcelable {
    var compassId = 0
        private set

    /**
     * RMS milligauss residuals
     */
    var fitness = 0f

    /**
     * X offset
     */
    private var xOffset = 0f
    private var yOffset = 0f
    private var zOffset = 0f
    private var xDiag = 0f
    private var yDiag = 0f
    private var zDiag = 0f
    private var xOffDiag = 0f
    private var yOffDiag = 0f
    private var zOffDiag = 0f
    var isAutoSaved = false
    var isCalibrationSuccessful = false

    constructor() {}
    constructor(compassId: Int,
                calibrationSuccessful: Boolean, autoSaved: Boolean, fitness: Float,
                xOffset: Float, yOffset: Float, zOffset: Float,
                xDiag: Float, yDiag: Float, zDiag: Float,
                xOffDiag: Float, yOffDiag: Float, zOffDiag: Float) {
        this.compassId = compassId
        isCalibrationSuccessful = calibrationSuccessful
        isAutoSaved = autoSaved
        this.fitness = fitness
        this.xDiag = xDiag
        this.xOffDiag = xOffDiag
        this.xOffset = xOffset
        this.yDiag = yDiag
        this.yOffDiag = yOffDiag
        this.yOffset = yOffset
        this.zDiag = zDiag
        this.zOffDiag = zOffDiag
        this.zOffset = zOffset
    }

    fun setCompassId(compassId: Byte) {
        this.compassId = compassId.toInt()
    }

    fun getxDiag(): Float {
        return xDiag
    }

    fun setxDiag(xDiag: Float) {
        this.xDiag = xDiag
    }

    fun getxOffDiag(): Float {
        return xOffDiag
    }

    fun setxOffDiag(xOffDiag: Float) {
        this.xOffDiag = xOffDiag
    }

    fun getxOffset(): Float {
        return xOffset
    }

    fun setxOffset(xOffset: Float) {
        this.xOffset = xOffset
    }

    fun getyDiag(): Float {
        return yDiag
    }

    fun setyDiag(yDiag: Float) {
        this.yDiag = yDiag
    }

    fun getyOffDiag(): Float {
        return yOffDiag
    }

    fun setyOffDiag(yOffDiag: Float) {
        this.yOffDiag = yOffDiag
    }

    fun getyOffset(): Float {
        return yOffset
    }

    fun setyOffset(yOffset: Float) {
        this.yOffset = yOffset
    }

    fun getzDiag(): Float {
        return zDiag
    }

    fun setzDiag(zDiag: Float) {
        this.zDiag = zDiag
    }

    fun getzOffDiag(): Float {
        return zOffDiag
    }

    fun setzOffDiag(zOffDiag: Float) {
        this.zOffDiag = zOffDiag
    }

    fun getzOffset(): Float {
        return zOffset
    }

    fun setzOffset(zOffset: Float) {
        this.zOffset = zOffset
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(compassId)
        dest.writeFloat(fitness)
        dest.writeFloat(xOffset)
        dest.writeFloat(yOffset)
        dest.writeFloat(zOffset)
        dest.writeFloat(xDiag)
        dest.writeFloat(yDiag)
        dest.writeFloat(zDiag)
        dest.writeFloat(xOffDiag)
        dest.writeFloat(yOffDiag)
        dest.writeFloat(zOffDiag)
        dest.writeByte(if (isAutoSaved) 1.toByte() else 0.toByte())
        dest.writeByte(if (isCalibrationSuccessful) 1.toByte() else 0.toByte())
    }

    private constructor(input: Parcel) {
        compassId = input.readInt()
        fitness = input.readFloat()
        xOffset = input.readFloat()
        yOffset = input.readFloat()
        zOffset = input.readFloat()
        xDiag = input.readFloat()
        yDiag = input.readFloat()
        zDiag = input.readFloat()
        xOffDiag = input.readFloat()
        yOffDiag = input.readFloat()
        zOffDiag = input.readFloat()
        isAutoSaved = input.readByte().toInt() != 0
        isCalibrationSuccessful = input.readByte().toInt() != 0
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<MagnetometerCalibrationResult> = object : Parcelable.Creator<MagnetometerCalibrationResult> {
            override fun createFromParcel(source: Parcel): MagnetometerCalibrationResult? {
                return MagnetometerCalibrationResult(source)
            }

            override fun newArray(size: Int): Array<MagnetometerCalibrationResult?> {
                return arrayOfNulls(size)
            }
        }
    }
}
