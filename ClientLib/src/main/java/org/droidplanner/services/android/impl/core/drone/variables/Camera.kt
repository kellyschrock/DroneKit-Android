package org.droidplanner.services.android.impl.core.drone.variables

import com.MAVLink.ardupilotmega.msg_camera_feedback
import com.MAVLink.ardupilotmega.msg_mount_status
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.property.Altitude
import com.o3dr.services.android.lib.drone.property.Attitude
import com.o3dr.services.android.lib.drone.property.Gps
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.MavLinkDrone
import org.droidplanner.services.android.impl.core.survey.CameraInfo
import org.droidplanner.services.android.impl.core.survey.Footprint
import java.util.*

class Camera(myDrone: MavLinkDrone?) : DroneVariable<MavLinkDrone?>(myDrone) {
    val camera = CameraInfo()
    private val footprints: MutableList<Footprint> = ArrayList()
    private var gimbal_pitch = 0.0
    
    fun newImageLocation(msg: msg_camera_feedback?) {
        footprints.add(Footprint(camera, msg))
        myDrone?.notifyDroneEvent(DroneEventsType.FOOTPRINT)
    }

    fun getFootprints(): List<Footprint> {
        return footprints
    }

    val lastFootprint: Footprint
        get() = footprints[footprints.size - 1]

    //double pitch = myDrone.getOrientation().getPitch() - gimbal_pitch;
    val currentFieldOfView: Footprint?
        get() {
            myDrone?.let { drone ->
                val droneAltitude = drone.getAttribute(AttributeType.ALTITUDE) as Altitude
                val altitude = droneAltitude.altitude
                val droneGps = drone.getAttribute(AttributeType.GPS) as Gps
                val position = droneGps.position
                //double pitch = drone.getOrientation().getPitch() - gimbal_pitch;
                val attitude = drone.getAttribute(AttributeType.ATTITUDE) as Attitude
                val pitch = attitude.pitch
                val roll = attitude.roll
                val yaw = attitude.yaw
                return Footprint(camera, position, altitude, pitch, roll, yaw)
            } ?: run {
                return null
            }
        }

    fun updateMountOrientation(msg_mount_status: msg_mount_status) {
        gimbal_pitch = (90 - msg_mount_status.pointing_a / 100).toDouble()
    }
}
