package com.o3dr.services.android.lib.drone.property

import android.os.Parcelable
import android.os.Parcel
import java.text.DecimalFormat
import java.text.ParseException
import java.util.LinkedHashMap
import kotlin.Throws

/**
 * Created by fhuya on 10/28/14.
 */
class Parameter : DroneAttribute, Comparable<Parameter> {
    companion object {
        const val RANGE_LOW = 0
        const val RANGE_HIGH = 1
        private val formatter = DecimalFormat.getInstance() as DecimalFormat

        @JvmField
        val CREATOR: Parcelable.Creator<Parameter> = object : Parcelable.Creator<Parameter> {
            override fun createFromParcel(source: Parcel): Parameter? {
                return Parameter(source)
            }

            override fun newArray(size: Int): Array<Parameter?> {
                return arrayOfNulls(size)
            }
        }

        init {
            formatter.applyPattern("0.###")
        }
    }

    var name: String?
    var value: Double
    var type: Int
    var displayName: String? = null
    var description: String? = null
    var units: String? = null
    var range: String? = null
    var values: String? = null

    constructor(name: String?, value: Double, type: Int) {
        this.name = name
        this.value = value
        this.type = type
    }

    val displayValue: String
        get() = formatter.format(value)

    fun hasInfo(): Boolean {
        return (description != null && !description!!.isEmpty()
                || values != null && !values!!.isEmpty())
    }

    @Throws(ParseException::class)
    fun parseRange(): DoubleArray {
        val format = formatter
        val parts = range!!.split(" ".toRegex()).toTypedArray()
        require(parts.size == 2)
        val outRange = DoubleArray(2)
        outRange[RANGE_LOW] = format.parse(parts[RANGE_LOW]).toDouble()
        outRange[RANGE_HIGH] = format.parse(parts[RANGE_HIGH]).toDouble()
        return outRange
    }

    @Throws(ParseException::class)
    fun parseValues(): Map<Double, String> {
        val format = formatter
        val outValues: MutableMap<Double, String> = LinkedHashMap()
        if (values != null) {
            val tparts = values!!.split(",".toRegex()).toTypedArray()
            for (tpart in tparts) {
                val parts = tpart.split(":".toRegex()).toTypedArray()
                require(parts.size == 2)
                outValues[format.parse(parts[0].trim { it <= ' ' }).toDouble()] = parts[1].trim { it <= ' ' }
            }
        }
        return outValues
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Parameter) return false
        return !if (name != null) name != other.name else other.name != null
    }

    override fun hashCode(): Int {
        return if (name != null) name.hashCode() else 0
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeString(name)
        dest.writeDouble(value)
        dest.writeInt(type)
        dest.writeString(displayName)
        dest.writeString(description)
        dest.writeString(units)
        dest.writeString(range)
        dest.writeString(values)
    }

    private constructor(`in`: Parcel) {
        name = `in`.readString()
        value = `in`.readDouble()
        type = `in`.readInt()
        displayName = `in`.readString()
        description = `in`.readString()
        units = `in`.readString()
        range = `in`.readString()
        values = `in`.readString()
    }

    override fun compareTo(another: Parameter): Int {
        return name!!.compareTo(another.name!!)
    }
}
