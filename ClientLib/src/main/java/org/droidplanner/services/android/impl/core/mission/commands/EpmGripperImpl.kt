package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.GRIPPER_ACTIONS
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class EpmGripperImpl : MissionCMD {
    var isRelease = true
        private set

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission) {
        unpackMAVMessage(msg)
    }

    constructor(mission: Mission?, release: Boolean) : super(mission) {
        isRelease = release
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_DO_GRIPPER
        mavMsg.param2 = if (isRelease) GRIPPER_ACTIONS.GRIPPER_ACTION_RELEASE.toFloat() else GRIPPER_ACTIONS.GRIPPER_ACTION_GRAB.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        if (mavMsg.param2 == GRIPPER_ACTIONS.GRIPPER_ACTION_GRAB.toFloat()) {
            isRelease = false
        } else if (mavMsg.param2 == GRIPPER_ACTIONS.GRIPPER_ACTION_RELEASE.toFloat()) {
            isRelease = true
        }
    }

    override fun getType(): MissionItemType = MissionItemType.EPM_GRIPPER

    fun setAsRelease(release: Boolean) {
        isRelease = release
    }
}
