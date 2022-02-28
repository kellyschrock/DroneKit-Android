package com.o3dr.services.android.lib.drone.mission

import android.os.Bundle
import android.os.Parcel
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.property.DroneAttribute
import java.util.*

/** Holds a set of mission items. */
class Mission : DroneAttribute {
    var currentMissionItem = 0
    private val missionItemsList: MutableList<MissionItem> = ArrayList()

    constructor() {}

    fun addMissionItem(missionItem: MissionItem) {
        missionItemsList.add(missionItem)
    }

    fun addMissionItem(index: Int, missionItem: MissionItem) {
        missionItemsList.add(index, missionItem)
    }

    fun removeMissionItem(missionItem: MissionItem) {
        missionItemsList.remove(missionItem)
    }

    fun removeMissionItem(index: Int) {
        missionItemsList.removeAt(index)
    }

    fun clear() {
        missionItemsList.clear()
    }

    fun getMissionItem(index: Int): MissionItem {
        return missionItemsList[index]
    }

    val missionItems: MutableList<MissionItem>
        get() = missionItemsList

    override fun equals(other: Any?): Boolean {
        if (this === other) {
            return true
        }
        if (other !is Mission) {
            return false
        }

        return if (currentMissionItem != other.currentMissionItem) {
            false
        } else missionItemsList == other.missionItemsList
    }

    override fun hashCode(): Int {
        var result = currentMissionItem
        result = 31 * result + missionItemsList.hashCode()
        return result
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        dest.writeInt(currentMissionItem)
        val missionItemsBundles: MutableList<Bundle> = ArrayList(missionItemsList.size)
        if (!missionItemsList.isEmpty()) {
            for (missionItem in missionItemsList) {
                missionItemsBundles.add(missionItem.type!!.storeMissionItem(missionItem))
            }
        }
        dest.writeTypedList(missionItemsBundles)
    }

    private constructor(input: Parcel) {
        currentMissionItem = input.readInt()
        val missionItemsBundles: List<Bundle> = ArrayList()
        input.readTypedList(missionItemsBundles, Bundle.CREATOR)
        if (!missionItemsBundles.isEmpty()) {
            for (bundle in missionItemsBundles) {
                missionItemsList.add(MissionItemType.restoreMissionItemFromBundle(bundle))
            }
        }
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<Mission> = object : Parcelable.Creator<Mission> {
            override fun createFromParcel(source: Parcel): Mission? {
                return Mission(source)
            }

            override fun newArray(size: Int): Array<Mission?> {
                return arrayOfNulls(size)
            }
        }
    }
}
