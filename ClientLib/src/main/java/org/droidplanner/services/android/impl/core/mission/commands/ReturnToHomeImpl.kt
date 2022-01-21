package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.commands.MissionCMD
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class ReturnToHomeImpl : MissionCMD {
    var height = 0.0

    constructor(item: MissionItemImpl?) : super(item) {
        height = 0.toDouble()
    }

    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?) : super(mission) {
        height = 0.0
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH
        mavMsg.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT.toShort()
        mavMsg.z = height.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMessageItem: msg_mission_item) {
        height = mavMessageItem.z.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.RTL
}
