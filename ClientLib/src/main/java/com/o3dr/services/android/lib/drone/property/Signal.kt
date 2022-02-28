package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable

/** Created by fhuya on 11/4/14. */
class Signal : DroneAttribute {
    var isValid = false
    var rxerrors = 0
    var fixed = 0
    var txbuf = 0
    var rssi = 0.0
    var remrssi = 0.0
    var noise = 0.0
    var remnoise = 0.0
    var signalStrength = 0.0

    constructor() {}

    constructor(isValid: Boolean, rxerrors: Int, fixed: Int, txbuf: Int, rssi: Double, remrssi: Double, noise: Double, remnoise: Double, signalStrength: Double) {
        this.isValid = isValid
        this.rxerrors = rxerrors
        this.fixed = fixed
        this.txbuf = txbuf
        this.rssi = rssi
        this.remrssi = remrssi
        this.noise = noise
        this.remnoise = remnoise
        this.signalStrength = signalStrength
    }

    val fadeMargin: Double
        get() = rssi - noise

    val remFadeMargin: Double
        get() = remrssi - remnoise

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeByte(if (isValid) 1.toByte() else 0.toByte())
        dest.writeInt(rxerrors)
        dest.writeInt(fixed)
        dest.writeInt(txbuf)
        dest.writeDouble(rssi)
        dest.writeDouble(remrssi)
        dest.writeDouble(noise)
        dest.writeDouble(remnoise)
        dest.writeDouble(signalStrength)
    }

    private constructor(input: Parcel) {
        isValid = input.readByte().toInt() != 0
        rxerrors = input.readInt()
        fixed = input.readInt()
        txbuf = input.readInt()
        rssi = input.readDouble()
        remrssi = input.readDouble()
        noise = input.readDouble()
        remnoise = input.readDouble()
        signalStrength = input.readDouble()
    }

    companion object {
        const val MAX_FADE_MARGIN = 50
        const val MIN_FADE_MARGIN = 6

        @JvmField
        val CREATOR: Parcelable.Creator<Signal> = object : Parcelable.Creator<Signal> {
            override fun createFromParcel(source: Parcel): Signal? {
                return Signal(source)
            }

            override fun newArray(size: Int): Array<Signal?> {
                return arrayOfNulls(size)
            }
        }
    }
}
