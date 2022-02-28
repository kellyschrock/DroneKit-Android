package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.commands.MissionCMD
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class TakePictureImpl : MissionCMD {
    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, triggerDistance: Double) : super(mission) {}

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_DO_DIGICAM_CONTROL
        mavMsg.x = 1f // Yes, this is correct. AP_Mission.cpp:717
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
//        distance = (mavMsg.param1);
    }

    override fun getType(): MissionItemType = MissionItemType.CAMERA_TRIGGER
}
