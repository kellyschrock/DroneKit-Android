package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class CameraTriggerImpl : MissionCMD {
    var triggerDistance = 0.toDouble()

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, triggerDistance: Double) : super(mission) {
        this.triggerDistance = triggerDistance
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST
        mavMsg.param1 = triggerDistance.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        triggerDistance = mavMsg.param1.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.CAMERA_TRIGGER
}
