package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

/**
 * Created by Toby on 7/31/2015.
 */
class DoJumpImpl : MissionCMD {
    var waypoint = 0
    var repeatCount = 0

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(mission: Mission?) : super(mission) {}
    constructor(mavMsg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(mavMsg)
    }

    constructor(mission: Mission?, waypoint: Int, repeatCount: Int) : super(mission) {
        this.waypoint = waypoint
        this.repeatCount = repeatCount
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        waypoint = mavMsg.param1.toInt()
        repeatCount = mavMsg.param2.toInt()
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_DO_JUMP
        mavMsg.param1 = waypoint.toFloat()
        mavMsg.param2 = repeatCount.toFloat()
        return list
    }

    override fun getType(): MissionItemType = MissionItemType.DO_JUMP
}
