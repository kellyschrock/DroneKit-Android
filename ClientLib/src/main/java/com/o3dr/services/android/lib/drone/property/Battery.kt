package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import java.util.*

/**
 * Created by fhuya on 10/28/14.
 */
class Battery : DroneAttribute {
    var batteryVoltage = 0.0
    var batteryRemain = 0.0
    var batteryCurrent = 0.0
    var batteryDischarge: Double? = null
    var currentConsumed = 0
    private var temperature: Short = 0
    var batteryFunction: Short = 0
    private var voltages: IntArray? = null
    private var mHasCellVoltages = false
    private var mHasTemperature = false

    constructor() {}
    constructor(batteryVoltage: Double, batteryRemain: Double, batteryCurrent: Double,
                batteryDischarge: Double?) {
        this.batteryVoltage = batteryVoltage
        this.batteryRemain = batteryRemain
        this.batteryCurrent = batteryCurrent
        this.batteryDischarge = batteryDischarge
    }

    fun setTemperature(temperature: Short) {
        this.temperature = temperature
        mHasTemperature = temperature > 0
    }

    fun getTemperature(): Short {
        return temperature
    }

    fun hasTemperature(): Boolean {
        return mHasTemperature
    }

    var cellVoltages: IntArray?
        get() = voltages
        set(voltages) {
            this.voltages = voltages
            mHasCellVoltages = voltages != null
        }

    fun hasCellVoltages(): Boolean {
        return mHasCellVoltages
    }

    val validCellVoltages: List<Int>
        get() {
            val list: MutableList<Int> = ArrayList()
            if (voltages != null) {
                var i = 0
                val size = voltages!!.size
                while (i < size) {
                    if (voltages!![i] < INT16_MAX) {
                        list.add(voltages!![i])
                    }
                    ++i
                }
            }
            return list
        }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeDouble(batteryVoltage)
        dest.writeDouble(batteryRemain)
        dest.writeDouble(batteryCurrent)
        dest.writeValue(batteryDischarge)
        dest.writeInt(currentConsumed)
        dest.writeInt(temperature.toInt())
        dest.writeInt(batteryFunction.toInt())
        val len = if (voltages != null) voltages!!.size else 0
        dest.writeInt(len)
        dest.writeIntArray(if (voltages != null) voltages else IntArray(0))
    }

    private constructor(input: Parcel) {
        batteryVoltage = input.readDouble()
        batteryRemain = input.readDouble()
        batteryCurrent = input.readDouble()
        batteryDischarge = input.readValue(Double::class.java.classLoader) as Double
        currentConsumed = input.readInt()
        temperature = input.readInt().toShort()
        batteryFunction = input.readInt().toShort()
        voltages = IntArray(input.readInt())
        input.readIntArray(voltages)
    }

    override fun toString(): String {
        return "Battery{" +
                "batteryVoltage=" + batteryVoltage +
                ", batteryRemain=" + batteryRemain +
                ", batteryCurrent=" + batteryCurrent +
                ", batteryDischarge=" + batteryDischarge +
                ", currentConsumed=" + currentConsumed +
                ", temperature=" + temperature +
                ", batteryFunction=" + batteryFunction +
                ", voltages=" + Arrays.toString(voltages) +
                '}'
    }

    companion object {
        private const val INT16_MAX = 65535
        @JvmField
        val CREATOR: Parcelable.Creator<Battery> = object : Parcelable.Creator<Battery> {
            override fun createFromParcel(source: Parcel): Battery {
                return Battery(source)
            }

            override fun newArray(size: Int): Array<Battery?> {
                return arrayOfNulls(size)
            }
        }
    }
}
