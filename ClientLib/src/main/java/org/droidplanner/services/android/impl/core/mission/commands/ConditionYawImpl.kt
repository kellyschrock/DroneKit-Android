package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class ConditionYawImpl : MissionCMD {
    var isRelative = false
    var angle = 0.0
    var angularSpeed = 0.0

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, angle: Double, isRelative: Boolean) : super(mission) {
        this.angle = angle
        this.isRelative = isRelative
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_CONDITION_YAW
        mavMsg.param1 = GeoTools.warpToPositiveAngle(angle).toFloat()
        mavMsg.param2 = Math.abs(angularSpeed).toFloat()
        mavMsg.param3 = if (angularSpeed < 0) 1.toFloat() else -1.toFloat()
        mavMsg.param4 = if (isRelative) 1.toFloat() else 0.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        isRelative = mavMsg.param4 != 0f
        angle = mavMsg.param1.toDouble()
        angularSpeed = (mavMsg.param2 * if (mavMsg.param3 > 0) -1 else +1).toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.CONDITION_YAW
}
