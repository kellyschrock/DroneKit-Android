package com.o3dr.services.android.lib.drone.property

import com.o3dr.services.android.lib.drone.property.DroneAttribute
import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.property.AutopilotVersion

/**
 * Stores information about the drone's various versions.
 */
class AutopilotVersion : DroneAttribute {
    /**
     * bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
     */
    var capabilities: Long = 0

    /**
     * UID if provided by hardware
     */
    var uid: Long = 0

    /**
     * Firmware version number
     */
    var flightSwVersion: Long = 0

    /**
     * Middleware version number
     */
    var middlewareSwVersion: Long = 0

    /**
     * Operating system version number
     */
    var osSwVersion: Long = 0

    /**
     * HW / board version (last 8 bytes should be silicon ID, if any)
     */
    var boardVersion: Long = 0

    /**
     * ID of the board vendor
     */
    var vendorId = 0

    /**
     * ID of the product
     */
    var productId = 0

    constructor() : super() {}

    constructor(
            capabilities: Long, uid: Long, flightSwVersion: Long, middlewareSwVersion: Long,
            osSwVersion: Long, boardVersion: Long, vendorId: Int, productId: Int) : this() {
        this.capabilities = capabilities
        this.uid = uid
        this.flightSwVersion = flightSwVersion
        this.middlewareSwVersion = middlewareSwVersion
        this.osSwVersion = osSwVersion
        this.boardVersion = boardVersion
        this.vendorId = vendorId
        this.productId = productId
    }

    override fun toString(): String {
        return "AutopilotVersion{" +
                "capabilities=" + capabilities +
                ", uid=" + uid +
                ", flightSwVersion=" + flightSwVersion +
                ", middlewareSwVersion=" + middlewareSwVersion +
                ", osSwVersion=" + osSwVersion +
                ", boardVersion=" + boardVersion +
                ", vendorId=" + vendorId +
                ", productId=" + productId +
                '}'
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeLong(capabilities)
        dest.writeLong(uid)
        dest.writeLong(flightSwVersion)
        dest.writeLong(middlewareSwVersion)
        dest.writeLong(osSwVersion)
        dest.writeLong(boardVersion)
        dest.writeInt(vendorId)
        dest.writeInt(productId)
    }

    private constructor(`in`: Parcel) {
        capabilities = `in`.readLong()
        uid = `in`.readLong()
        flightSwVersion = `in`.readLong()
        middlewareSwVersion = `in`.readLong()
        osSwVersion = `in`.readLong()
        boardVersion = `in`.readLong()
        vendorId = `in`.readInt()
        productId = `in`.readInt()
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<AutopilotVersion> = object : Parcelable.Creator<AutopilotVersion> {
            override fun createFromParcel(source: Parcel): AutopilotVersion {
                return AutopilotVersion(source)
            }

            override fun newArray(size: Int): Array<AutopilotVersion?> {
                return arrayOfNulls(size)
            }
        }
    }
}
