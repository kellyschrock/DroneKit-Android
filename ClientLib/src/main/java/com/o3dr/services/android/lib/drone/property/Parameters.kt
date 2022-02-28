package com.o3dr.services.android.lib.drone.property

import android.os.Parcel
import android.os.Parcelable
import android.text.TextUtils
import java.util.*

/** Created by fhuya on 10/28/14. */
class Parameters : DroneAttribute {
    private val parametersList: MutableList<Parameter> = ArrayList()

    constructor() {}

    constructor(parameterList: Collection<Parameter>?) {
        setParametersList(parameterList)
    }

    val parameters: List<Parameter>
        get() = parametersList

    fun getParameter(name: String?): Parameter? {
        if (TextUtils.isEmpty(name)) return null
        for (param in parametersList) {
            if (param.name.equals(name, ignoreCase = true)) return param
        }
        return null
    }

    fun updateParameter(param: Parameter?): Boolean {
        var success = false
        if (param != null) {
            var i = 0
            val size = parametersList.size
            while (i < size) {
                val p = parametersList[i]
                if (p.name == param.name) {
                    parametersList[i] = param
                    success = true
                    break
                }
                ++i
            }
        }
        return success
    }

    fun updateFrom(params: Parameters?): Boolean {
        var success = false
        if (params != null) {
            val list = params.parameters
            for (param in list) {
                success = updateParameter(param)
            }
        }
        return success
    }

    fun setParametersList(parametersList: Collection<Parameter>?) {
        this.parametersList.clear()
        if (parametersList != null && !parametersList.isEmpty()) {
            this.parametersList.addAll(parametersList)
        }
    }

    /**
     * Adds a parameter to the parameters set.
     * @param parameter
     * @since 2.8.0
     */
    fun addParameter(parameter: Parameter) {
        if (parameter == null) throw NullPointerException("Invalid parameter argument.")
        parametersList.add(parameter)
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeTypedList(parametersList)
    }

    private constructor(input: Parcel) {
        input.readTypedList(parametersList, Parameter.CREATOR)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Parameters> = object : Parcelable.Creator<Parameters> {
            override fun createFromParcel(source: Parcel): Parameters? {
                return Parameters(source)
            }

            override fun newArray(size: Int): Array<Parameters?> {
                return arrayOfNulls(size)
            }
        }
    }
}
