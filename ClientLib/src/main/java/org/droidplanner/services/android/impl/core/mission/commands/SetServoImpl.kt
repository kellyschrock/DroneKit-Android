package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.commands.MissionCMD
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class SetServoImpl : MissionCMD {
    var pwm = 0
    var channel = 0

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, channel: Int, pwm: Int) : super(mission) {
        this.channel = channel
        this.pwm = pwm
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        channel = mavMsg.param1.toInt()
        pwm = mavMsg.param2.toInt()
    }

    override fun getType(): MissionItemType = MissionItemType.SET_SERVO

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_DO_SET_SERVO
        mavMsg.param1 = channel.toFloat()
        mavMsg.param2 = pwm.toFloat()
        return list
    }
}
