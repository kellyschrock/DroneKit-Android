package com.o3dr.services.android.lib.drone.mission.item.command

import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import android.os.Parcelable
import com.o3dr.services.android.lib.drone.mission.item.command.SetServo
import android.os.Parcel
import com.o3dr.services.android.lib.drone.mission.MissionItemType

/**
 * Mission command used to move a servo to a particular pwm value.
 */
class SetServo : MissionItem, MissionItem.Command, Parcelable {
    /**
     * @return PWM value to output to the servo
     */
    /**
     * Set PWM value to output to the servo
     * @param pwm value to output to the servo
     */
    var pwm = 0
    /**
     * @return the output channel the servo is attached to
     */
    /**
     * @param channel the output channel the servo is attached to
     */
    var channel = 0

    constructor() : super(MissionItemType.SET_SERVO) {}
    constructor(copy: SetServo) : this() {
        pwm = copy.pwm
        channel = copy.channel
    }

    override fun toString(): String {
        return "SetServo{" +
                "channel=" + channel +
                ", pwm=" + pwm +
                '}'
    }

    override fun equals(o: Any?): Boolean {
        if (this === o) return true
        if (o !is SetServo) return false
        if (!super.equals(o)) return false
        val setServo = o
        return if (pwm != setServo.pwm) false else channel == setServo.channel
    }

    override fun hashCode(): Int {
        var result = super.hashCode()
        result = 31 * result + pwm
        result = 31 * result + channel
        return result
    }

    override fun writeToParcel(dest: Parcel, flags: Int) {
        super.writeToParcel(dest, flags)
        dest.writeInt(pwm)
        dest.writeInt(channel)
    }

    private constructor(`in`: Parcel) : super(`in`) {
        pwm = `in`.readInt()
        channel = `in`.readInt()
    }

    override fun clone(): MissionItem {
        return SetServo(this)
    }

    companion object {
        @JvmField
        val CREATOR: Parcelable.Creator<SetServo> = object : Parcelable.Creator<SetServo> {
            override fun createFromParcel(source: Parcel): SetServo? {
                return SetServo(source)
            }

            override fun newArray(size: Int): Array<SetServo?> {
                return arrayOfNulls(size)
            }
        }
    }
}
