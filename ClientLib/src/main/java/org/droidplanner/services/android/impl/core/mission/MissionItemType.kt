package org.droidplanner.services.android.impl.core.mission

import kotlin.Throws
import org.droidplanner.services.android.impl.core.mission.waypoints.WaypointImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.SplineWaypointImpl
import org.droidplanner.services.android.impl.core.mission.commands.TakeoffImpl
import org.droidplanner.services.android.impl.core.mission.commands.ChangeSpeedImpl
import org.droidplanner.services.android.impl.core.mission.commands.LoiterToAltImpl
import org.droidplanner.services.android.impl.core.mission.commands.LoiterTimeImpl
import org.droidplanner.services.android.impl.core.mission.commands.TakePictureImpl
import org.droidplanner.services.android.impl.core.mission.commands.CameraTriggerImpl
import org.droidplanner.services.android.impl.core.mission.commands.EpmGripperImpl
import org.droidplanner.services.android.impl.core.mission.commands.ReturnToHomeImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.LandImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.CircleImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.RegionOfInterestImpl
import org.droidplanner.services.android.impl.core.mission.survey.SurveyImpl
import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.mission.survey.SplineSurveyImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.StructureScannerImpl
import org.droidplanner.services.android.impl.core.mission.commands.SetServoImpl
import org.droidplanner.services.android.impl.core.mission.commands.ConditionYawImpl
import org.droidplanner.services.android.impl.core.mission.commands.SetRelayImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.DoLandStartImpl
import org.droidplanner.services.android.impl.core.mission.commands.DoJumpImpl
import org.droidplanner.services.android.impl.core.mission.commands.RawMissionCommandImpl
import java.lang.IllegalArgumentException

enum class MissionItemType(val typeName: String) {
    WAYPOINT("Waypoint"),
    SPLINE_WAYPOINT("Spline Waypoint"),
    TAKEOFF("Takeoff"),
    RTL("Return to Launch"),
    LAND("Land"),
    CIRCLE("Circle"),
    ROI("Region of Interest"),
    SURVEY("Survey"),
    SPLINE_SURVEY("Spline Survey"),
    CYLINDRICAL_SURVEY("Structure Scan"),
    CHANGE_SPEED("Change Speed"),
    CAMERA_TRIGGER("Camera Trigger"),
    EPM_GRIPPER("EPM"),
    SET_SERVO("Set Servo"),
    CONDITION_YAW("Set Yaw"),
    SET_RELAY("Set Relay"),
    DO_LAND_START("Do Land Start"),
    DO_JUMP("Do Jump"),
    LOITER_TO_ALT("Loiter to Alt"),
    LOITER_TIME("Loiter Time"),
    TAKE_PICTURE("Take Picture"),
    VTOL_TAKEOFF("VTOL Takeoff"),
    VTOL_TRANSITION("VTOL Transition"),
    VTOL_LAND("VTOL Land"),
    RAW_COMMAND("Raw Command")
    ;

    @Throws(IllegalArgumentException::class)
    fun getNewItem(referenceItem: MissionItemImpl): MissionItemImpl {
        return when (this) {
            WAYPOINT -> WaypointImpl(referenceItem)
            SPLINE_WAYPOINT -> SplineWaypointImpl(referenceItem)
            TAKEOFF -> TakeoffImpl(referenceItem)
            CHANGE_SPEED -> ChangeSpeedImpl(referenceItem)
            LOITER_TO_ALT -> LoiterToAltImpl(referenceItem)
            LOITER_TIME -> LoiterTimeImpl(referenceItem)
            TAKE_PICTURE -> TakePictureImpl(referenceItem)
            CAMERA_TRIGGER -> CameraTriggerImpl(referenceItem)
            EPM_GRIPPER -> EpmGripperImpl(referenceItem)
            RTL -> ReturnToHomeImpl(referenceItem)
            LAND -> LandImpl(referenceItem)
            CIRCLE -> CircleImpl(referenceItem)
            ROI -> RegionOfInterestImpl(referenceItem)
            SURVEY -> SurveyImpl(referenceItem.getMission(), emptyList<LatLong>())
            SPLINE_SURVEY -> SplineSurveyImpl(referenceItem.getMission(), emptyList<LatLong>())
            CYLINDRICAL_SURVEY -> StructureScannerImpl(referenceItem)
            SET_SERVO -> SetServoImpl(referenceItem)
            CONDITION_YAW -> ConditionYawImpl(referenceItem)
            SET_RELAY -> SetRelayImpl(referenceItem)
            DO_LAND_START -> DoLandStartImpl(referenceItem)
            DO_JUMP -> DoJumpImpl(referenceItem)
            RAW_COMMAND -> RawMissionCommandImpl(referenceItem)
            else -> throw IllegalArgumentException("Unrecognized mission item type ($typeName)")
        }
    }
}
