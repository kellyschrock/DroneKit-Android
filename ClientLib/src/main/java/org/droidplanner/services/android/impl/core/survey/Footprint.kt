package org.droidplanner.services.android.impl.core.survey

import com.MAVLink.ardupilotmega.msg_camera_feedback
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.util.MathUtils.dcmFromEuler
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.getDistance
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools.Companion.moveCoordinate
import java.util.*

class Footprint(camera: CameraInfo, center: LatLong, alt: Double, pitch: Double, roll: Double, yaw: Double) {
    /**
     * Vertex of the footprint in local frame index 0 is top right, where top is
     * direction of longitudinal travel. Index increases CCW
     */
    private val vertex: MutableList<LatLong> = ArrayList()
    val gSD: Double

    constructor(camera: CameraInfo, altitude: Double)
            : this(camera, LatLong(0.0, 0.0), altitude, 0.0, 0.0, 0.0) {}

    constructor(camera: CameraInfo, msg: msg_camera_feedback)
            : this(camera, LatLong(msg.lat / 1E7, msg.lng / 1E7), msg.alt_rel.toDouble(), msg.pitch.toDouble(), msg.roll.toDouble(), msg.yaw.toDouble()) {}

    val lateralFootPrint: Double
        get() = (getDistance(vertex[0], vertex[1]) + getDistance(vertex[2], vertex[3])) / 2

    // What is this!
    //get() {
//        return footprint.lateralSize
//    }
    val longitudinalFootPrint: Double
        get() = (getDistance(vertex[0], vertex[3]) + getDistance(vertex[1], vertex[2])) / 2
//  What is this??
//get() {
//        return footprint.longitudinalSize
//    }

    val vertexInGlobalFrame: List<LatLong>
        get() = vertex

    companion object {
        /**
         * based on http://www.asprs.org/a/publications/pers/2005journal/july/2005_july_863-871.pdf
         */
        private fun cameraFrameToLocalFrame(img: LatLong, dcm: Array<DoubleArray>, alt: Double,
                                            focalLength: Double, center: LatLong): LatLong {
            val x = (alt
                    * (dcm[0][0] * img.latitude + dcm[1][0] * img.longitude + dcm[2][0] * -focalLength)
                    / (dcm[0][2] * img.latitude + dcm[1][2] * img.longitude + dcm[2][2] * -focalLength))
            val y = (alt
                    * (dcm[0][1] * img.latitude + dcm[1][1] * img.longitude + dcm[2][1] * -focalLength)
                    / (dcm[0][2] * img.latitude + dcm[1][2] * img.longitude + dcm[2][2] * -focalLength))
            return moveCoordinate(center, x, y)
        }
    }

    init {
        val sx = camera.sensorLateralSize / 2
        val sy = camera.sensorLongitudinalSize / 2
        val f = camera.focalLength
        val dcm = dcmFromEuler(Math.toRadians(pitch), Math.toRadians(-roll + 180), Math.toRadians(-yaw))
        vertex.add(cameraFrameToLocalFrame(LatLong(-sx, -sy), dcm, alt, f, center))
        vertex.add(cameraFrameToLocalFrame(LatLong(+sx, -sy), dcm, alt, f, center))
        vertex.add(cameraFrameToLocalFrame(LatLong(+sx, +sy), dcm, alt, f, center))
        vertex.add(cameraFrameToLocalFrame(LatLong(-sx, +sy), dcm, alt, f, center))
        gSD = (0.001 * lateralFootPrint * (sy / sx) / Math.sqrt(camera.sensorResolution))
    }
}
