package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.coordinate.LatLong

/** Stores GPS information. */
class Gps : DroneAttribute {
    var gpsEph = 0.0
    var satellitesCount = 0
        private set
    var fixType = 0

    private var _position: LatLong? = null
    private var vehicleArmed = false
    private var ekfStatus: EkfStatus? = null

    var position: LatLong?
        get() = if(isValid) _position else null
        set(value) { _position = value }

    constructor() {}

    constructor(position: LatLong?, gpsEph: Double, satCount: Int, fixType: Int) {
        this._position = position
        this.gpsEph = gpsEph
        satellitesCount = satCount
        this.fixType = fixType
    }

    constructor(latitude: Double, longitude: Double, gpsEph: Double, satCount: Int, fixType: Int) : this(LatLong(latitude, longitude), gpsEph, satCount, fixType) {}

    val isValid: Boolean
        get() = if (ekfStatus == null) {
            _position != null
        } else {
            ekfStatus!!.isPositionOk(vehicleArmed) && _position != null
        }

    val fixStatus: String
        get() = when (fixType) {
            LOCK_2D_TYPE -> LOCK_2D
            LOCK_3D_TYPE -> LOCK_3D
            LOCK_3D_DGPS_TYPE -> LOCK_3D_DGPS
            LOCK_3D_RTK_FLOAT -> LOCK_3D_RTK_FLOAT_NAME
            LOCK_3D_RTK_FIXED -> LOCK_3D_RTK_FIXED_NAME
            LOCK_STATIC -> LOCK_STATIC_NAME
            else -> NO_FIX
        }

    fun setSatCount(satCount: Int) {
        satellitesCount = satCount
    }

    fun setEkfStatus(ekfStatus: EkfStatus?) {
        this.ekfStatus = ekfStatus
    }

    fun setVehicleArmed(vehicleArmed: Boolean) {
        this.vehicleArmed = vehicleArmed
    }

    /**
     * @return True if there's a 3D GPS lock, false otherwise.
     * @since 2.6.8
     */
    fun has3DLock(): Boolean {
        return fixType == LOCK_3D_TYPE ||
                fixType == LOCK_3D_DGPS_TYPE ||
                fixType == LOCK_3D_RTK_FLOAT ||
                fixType == LOCK_3D_RTK_FIXED
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Gps) return false
        if (fixType != other.fixType) return false
        if (java.lang.Double.compare(other.gpsEph, gpsEph) != 0) return false
        if (satellitesCount != other.satellitesCount) return false
        if (if (_position != null) _position != other._position else other._position != null) return false
        if (vehicleArmed != other.vehicleArmed) return false
        return !if (ekfStatus != null) ekfStatus != other.ekfStatus else other.ekfStatus != null
    }

    override fun hashCode(): Int {
        var result: Int
        val temp: Long = java.lang.Double.doubleToLongBits(gpsEph)
        result = (temp xor (temp ushr 32)).toInt()
        result = 31 * result + satellitesCount
        result = 31 * result + fixType
        result = 31 * result + if (_position != null) _position.hashCode() else 0
        result = 31 * result + if (vehicleArmed) 1 else 0
        result = 31 * result + if (ekfStatus != null) ekfStatus.hashCode() else 0
        return result
    }

    override fun toString(): String {
        return "Gps{" +
                "gpsEph=" + gpsEph +
                ", satCount=" + satellitesCount +
                ", fixType=" + fixType +
                ", position=" + _position +
                ", vehicleArmed=" + vehicleArmed +
                ", ekfStatus=" + ekfStatus +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(gpsEph)
        dest.writeInt(satellitesCount)
        dest.writeInt(fixType)
        dest.writeParcelable(_position, 0)
        dest.writeByte((if (vehicleArmed) 1 else 0).toByte())
        dest.writeParcelable(ekfStatus, 0)
    }

    private constructor(input: Parcel) {
        gpsEph = input.readDouble()
        satellitesCount = input.readInt()
        fixType = input.readInt()
        _position = input.readParcelable(LatLong::class.java.classLoader)
        vehicleArmed = input.readByte().toInt() != 0
        ekfStatus = input.readParcelable(EkfStatus::class.java.classLoader)
    }

    companion object {
        const val LOCK_2D = "2D"
        const val LOCK_3D = "3D"
        const val LOCK_3D_DGPS = "3D+DGPS"
        const val LOCK_3D_RTK_FLOAT_NAME = "RTK Float"
        const val LOCK_3D_RTK_FIXED_NAME = "RTK Fixed"
        const val LOCK_STATIC_NAME = "Static"
        const val NO_FIX = "NoFix"

        private const val LOCK_2D_TYPE = 2
        private const val LOCK_3D_TYPE = 3
        private const val LOCK_3D_DGPS_TYPE = 4
        private const val LOCK_3D_RTK_FLOAT = 5
        private const val LOCK_3D_RTK_FIXED = 6
        private const val LOCK_STATIC = 7

        @JvmField
        val CREATOR: Parcelable.Creator<Gps> = object : Parcelable.Creator<Gps> {
            override fun createFromParcel(source: Parcel): Gps? {
                return Gps(source)
            }

            override fun newArray(size: Int): Array<Gps?> {
                return arrayOfNulls(size)
            }
        }
    }
}
