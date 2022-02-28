package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import org.droidplanner.services.android.impl.core.mission.commands.RawMissionCommandImpl
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

class RawMissionCommand : MissionItem, MissionItem.Command, Parcelable {
    var param1 = 0f
    var param2 = 0f
    var param3 = 0f
    var param4 = 0f
    var x = 0f
    var y = 0f
    var z = 0f
    var command = 0
    var target_system: Short = 0
    var target_component: Short = 0

    constructor() : super(MissionItemType.RAW_COMMAND) {}
    constructor(input: RawMissionCommand) : this() {
        command = input.command
        param1 = input.param1
        param2 = input.param2
        param3 = input.param3
        param4 = input.param4
        x = input.x
        y = input.y
        z = input.z
        target_component = input.target_component
        target_system = input.target_system
    }

    fun setTo(s: RawMissionCommandImpl): RawMissionCommand {
        command = s.command
        param1 = s.param1
        param2 = s.param2
        param3 = s.param3
        param4 = s.param4
        x = s.x
        y = s.y
        z = s.z
        target_system = s.target_system
        target_component = s.target_component
        return this
    }

    fun setParam1(param1: Float): RawMissionCommand {
        this.param1 = param1
        return this
    }

    fun setParam2(param2: Float): RawMissionCommand {
        this.param2 = param2
        return this
    }

    fun setParam3(param3: Float): RawMissionCommand {
        this.param3 = param3
        return this
    }

    fun setParam4(param4: Float): RawMissionCommand {
        this.param4 = param4
        return this
    }

    fun setX(x: Float): RawMissionCommand {
        this.x = x
        return this
    }

    fun setY(y: Float): RawMissionCommand {
        this.y = y
        return this
    }

    fun setZ(z: Float): RawMissionCommand {
        this.z = z
        return this
    }

    fun setCommand(command: Int): RawMissionCommand {
        this.command = command
        return this
    }

    fun setTarget_system(target_system: Short): RawMissionCommand {
        this.target_system = target_system
        return this
    }

    fun setTarget_component(target_component: Short): RawMissionCommand {
        this.target_component = target_component
        return this
    }

    override fun clone(): MissionItem {
        return RawMissionCommand(this)
    }

    override fun describeContents(): Int {
        return 0
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeFloat(param1)
        dest.writeFloat(param2)
        dest.writeFloat(param3)
        dest.writeFloat(param4)
        dest.writeFloat(x)
        dest.writeFloat(y)
        dest.writeFloat(z)
        dest.writeInt(command)
        dest.writeInt(target_system.toInt())
        dest.writeInt(target_component.toInt())
    }

    protected constructor(`in`: Parcel) : super(`in`) {
        param1 = `in`.readFloat()
        param2 = `in`.readFloat()
        param3 = `in`.readFloat()
        param4 = `in`.readFloat()
        x = `in`.readFloat()
        y = `in`.readFloat()
        z = `in`.readFloat()
        command = `in`.readInt()
        target_system = `in`.readInt().toShort()
        target_component = `in`.readInt().toShort()
    }

    override fun toString(): String {
        return "RawMissionCommand{" +
                "param1=" + param1 +
                ", param2=" + param2 +
                ", param3=" + param3 +
                ", param4=" + param4 +
                ", x=" + x +
                ", y=" + y +
                ", z=" + z +
                ", command=" + command +
                ", target_system=" + target_system +
                ", target_component=" + target_component +
                '}'
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<RawMissionCommand> = object : Parcelable.Creator<RawMissionCommand> {
            override fun createFromParcel(source: Parcel): RawMissionCommand? {
                return RawMissionCommand(source)
            }

            override fun newArray(size: Int): Array<RawMissionCommand?> {
                return arrayOfNulls(size)
            }
        }
    }
}
