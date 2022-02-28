package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import android.text.TextUtils
import android.util.Log
import org.json.JSONException
import org.json.JSONObject

/** Created by fhuya on 10/28/14. */
class State : DroneAttribute {
    var isConnected = false
    var isArmed = false
    var isFlying = false
    var calibrationStatus: String? = null
    var autopilotErrorId: String? = null
    var mavlinkVersion = INVALID_MAVLINK_VERSION
    var flightStartTime: Long = 0
    var vehicleMode = VehicleMode.UNKNOWN
    var ekfStatus = EkfStatus()
        private set
    var isTelemetryLive = false
    var sysid: Short = 0
        private set
    var compid: Short = 0
        private set
    var vehicleVibration = Vibration()
        private set
    val vehicleUid: JSONObject

    constructor() {
        vehicleUid = JSONObject()
    }

    constructor(isConnected: Boolean, mode: VehicleMode?, armed: Boolean, flying: Boolean,
                autopilotErrorId: String?, mavlinkVersion: Int, calibrationStatus: String?,
                flightStartTime: Long, ekfStatus: EkfStatus?, isTelemetryLive: Boolean,
                vibration: Vibration?, sysid: Short, compid: Short) {
        vehicleUid = JSONObject()
        this.isConnected = isConnected
        isArmed = armed
        isFlying = flying
        this.flightStartTime = flightStartTime
        this.autopilotErrorId = autopilotErrorId
        this.mavlinkVersion = mavlinkVersion
        this.calibrationStatus = calibrationStatus
        this.sysid = sysid
        this.compid = compid
        if (ekfStatus != null) this.ekfStatus = ekfStatus
        if (mode != null) vehicleMode = mode
        this.isTelemetryLive = isTelemetryLive
        if (vibration != null) vehicleVibration = vibration
    }

    val isWarning: Boolean
        get() = TextUtils.isEmpty(autopilotErrorId)

    val isCalibrating: Boolean
        get() = calibrationStatus != null

    fun setCalibration(message: String?) {
        calibrationStatus = message
    }

    fun addToVehicleUid(uidLabel: String?, uid: String?) {
        try {
            vehicleUid.put(uidLabel, uid)
        } catch (e: JSONException) {
            Log.e(TAG, e.message, e)
        }
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeByte(if (isConnected) 1.toByte() else 0.toByte())
        dest.writeByte(if (isArmed) 1.toByte() else 0.toByte())
        dest.writeByte(if (isFlying) 1.toByte() else 0.toByte())
        dest.writeString(calibrationStatus)
        dest.writeParcelable(vehicleMode, 0)
        dest.writeString(autopilotErrorId)
        dest.writeInt(mavlinkVersion)
        dest.writeLong(flightStartTime)
        dest.writeParcelable(ekfStatus, 0)
        dest.writeByte(if (isTelemetryLive) 1.toByte() else 0.toByte())
        dest.writeParcelable(vehicleVibration, 0)
        dest.writeString(vehicleUid.toString())
    }

    private constructor(input: Parcel) {
        isConnected = input.readByte().toInt() != 0
        isArmed = input.readByte().toInt() != 0
        isFlying = input.readByte().toInt() != 0
        calibrationStatus = input.readString()
        vehicleMode = input.readParcelable(VehicleMode::class.java.classLoader)
        autopilotErrorId = input.readString()
        mavlinkVersion = input.readInt()
        flightStartTime = input.readLong()
        ekfStatus = input.readParcelable(EkfStatus::class.java.classLoader)
        isTelemetryLive = input.readByte().toInt() != 0
        vehicleVibration = input.readParcelable(Vibration::class.java.classLoader)
        val temp: JSONObject
        temp = try {
            JSONObject(input.readString())
        } catch (e: JSONException) {
            JSONObject()
        }
        vehicleUid = temp
    }

    companion object {
        private val TAG = State::class.java.simpleName
        const val INVALID_MAVLINK_VERSION = -1

        @JvmField
        val CREATOR: Parcelable.Creator<State> = object : Parcelable.Creator<State> {
            override fun createFromParcel(source: Parcel): State? {
                return State(source)
            }

            override fun newArray(size: Int): Array<State?> {
                return arrayOfNulls(size)
            }
        }
    }
}
