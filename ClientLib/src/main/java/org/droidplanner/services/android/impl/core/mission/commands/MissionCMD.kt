package org.droidplanner.services.android.impl.core.mission.commands

import com.MAVLink.common.msg_mission_item
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl

abstract class MissionCMD : MissionItemImpl {
    constructor(mission: Mission?) : super(mission) {}
    constructor(item: MissionItemImpl?) : super(item) {}

    override fun packMissionItem(): List<msg_mission_item> {
        return super.packMissionItem()
    }
}
