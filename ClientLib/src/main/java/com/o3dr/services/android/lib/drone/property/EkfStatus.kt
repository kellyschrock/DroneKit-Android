package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import com.MAVLink.enums.EKF_STATUS_FLAGS
import java.util.*

/**
 * Abstraction for vehicle EFK status. See http://copter.ardupilot.com/wiki/common-apm-navigation-extended-kalman-filter-overview/
 */
class EkfStatus : DroneAttribute {
    enum class EkfFlags(val value: Int) {
        EKF_ATTITUDE(EKF_STATUS_FLAGS.EKF_ATTITUDE),
        EKF_VELOCITY_HORIZ(EKF_STATUS_FLAGS.EKF_VELOCITY_HORIZ),
        EKF_VELOCITY_VERT(EKF_STATUS_FLAGS.EKF_VELOCITY_VERT),
        EKF_POS_HORIZ_REL(EKF_STATUS_FLAGS.EKF_POS_HORIZ_REL),
        EKF_POS_HORIZ_ABS(EKF_STATUS_FLAGS.EKF_POS_HORIZ_ABS),
        EKF_POS_VERT_ABS(EKF_STATUS_FLAGS.EKF_POS_VERT_ABS),
        EKF_POS_VERT_AGL(EKF_STATUS_FLAGS.EKF_POS_VERT_AGL),
        EKF_CONST_POS_MODE(EKF_STATUS_FLAGS.EKF_CONST_POS_MODE),
        EKF_PRED_POS_HORIZ_REL(EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_REL),
        EKF_PRED_POS_HORIZ_ABS(EKF_STATUS_FLAGS.EKF_PRED_POS_HORIZ_ABS);
    }

    var velocityVariance = 0f
    var horizontalPositionVariance = 0f
    var verticalPositionVariance = 0f
    var compassVariance = 0f
    var terrainAltitudeVariance = 0f
    private val flags: BitSet

    constructor() {
        flags = BitSet(FLAGS_BIT_COUNT)
    }

    constructor(flags: Int, compassVariance: Float, horizontalPositionVariance: Float, terrainAltitudeVariance: Float, velocityVariance: Float, verticalPositionVariance: Float) : this() {
        this.compassVariance = compassVariance
        this.horizontalPositionVariance = horizontalPositionVariance
        this.terrainAltitudeVariance = terrainAltitudeVariance
        this.velocityVariance = velocityVariance
        this.verticalPositionVariance = verticalPositionVariance
        fromShortToBitSet(flags)
    }

    private fun fromShortToBitSet(flags: Int) {
        val ekfFlags = EkfFlags.values()
        val ekfFlagsCount = ekfFlags.size
        for (i in 0 until ekfFlagsCount) {
            this.flags[i] = flags and ekfFlags[i].value != 0
        }
    }

    fun isEkfFlagSet(flag: EkfFlags): Boolean {
        return flags[flag.ordinal]
    }

    /**
     * Returns true if the horizontal absolute position is ok, and home position is set.
     *
     * @param armed
     * @return
     */
    fun isPositionOk(armed: Boolean): Boolean {
        return if (armed) {
            (flags[EkfFlags.EKF_POS_HORIZ_ABS.ordinal]
                    && !flags[EkfFlags.EKF_CONST_POS_MODE.ordinal])
        } else {
            (flags[EkfFlags.EKF_POS_HORIZ_ABS.ordinal]
                    || flags[EkfFlags.EKF_PRED_POS_HORIZ_ABS.ordinal])
        }
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeFloat(velocityVariance)
        dest.writeFloat(horizontalPositionVariance)
        dest.writeFloat(verticalPositionVariance)
        dest.writeFloat(compassVariance)
        dest.writeFloat(terrainAltitudeVariance)
        dest.writeSerializable(this.flags)
    }

    private constructor(input: Parcel) {
        velocityVariance = input.readFloat()
        horizontalPositionVariance = input.readFloat()
        verticalPositionVariance = input.readFloat()
        compassVariance = input.readFloat()
        terrainAltitudeVariance = input.readFloat()
        flags = input.readSerializable() as BitSet
    }

    companion object {
        private const val FLAGS_BIT_COUNT = 16
        @JvmField
        val CREATOR: Parcelable.Creator<EkfStatus> = object : Parcelable.Creator<EkfStatus> {
            override fun createFromParcel(source: Parcel): EkfStatus? {
                return EkfStatus(source)
            }

            override fun newArray(size: Int): Array<EkfStatus?> {
                return arrayOfNulls(size)
            }
        }
    }
}
