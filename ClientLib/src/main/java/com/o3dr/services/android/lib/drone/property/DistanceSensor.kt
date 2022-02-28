package com.o3dr.services.android.lib.drone.property

import com.MAVLink.common.msg_distance_sensor
import com.MAVLink.enums.MAV_DISTANCE_SENSOR
import com.o3dr.services.android.lib.drone.property.DistanceSensor

class DistanceSensor {
    enum class Type(val mavType: Int) {
        Laser(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_LASER),
        Ultrasound(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_ULTRASOUND),
        Infrared(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_INFRARED),
        Radar(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_RADAR),
        Unknown(MAV_DISTANCE_SENSOR.MAV_DISTANCE_SENSOR_UNKNOWN)
        ;

        companion object {
            @JvmStatic
            fun fromType(type: Int): Type {
                for (t in values()) {
                    if (t.mavType == type) {
                        return t
                    }
                }
                return Unknown
            }
        }
    }

    enum class Orientation(val direction: Int, val label: String) {
        Forward(0, "Forward"),
        ForwardRight(1, "Forward Right"),
        Right(2, "Right"),
        BackRight(3, "Back Right"),
        Back(4, "Back"),
        BackLeft(5, "Back Left"),
        Left(6, "Left"),
        ForwardLeft(7, "Forward Left"),
        Up(24, "Up"),
        Down(25, "Down")
        ;

        companion object {
            @JvmStatic
            fun from(o: Int): Orientation {
                for (v in values()) {
                    if (v.direction == o) {
                        return v
                    }
                }
                return Forward
            }
        }
    }

    var minDistance = 0
        private set
    var maxDistance = 0
        private set
    var currDistance = 0
        private set
    var type: Type? = null
        private set
    var id = 0
        private set
    var orientation: Orientation? = null
        private set
    var covariance: Short = 0
        private set

    override fun toString(): String {
        return "DistanceSensor{" +
                "minDistance=" + minDistance +
                ", maxDistance=" + maxDistance +
                ", currDistance=" + currDistance +
                ", type=" + type +
                ", id=" + id +
                ", orientation=" + orientation +
                ", covariance=" + covariance +
                '}'
    }

    companion object {
        @JvmStatic
        fun populate(sensor: DistanceSensor, msg: msg_distance_sensor): DistanceSensor {
            sensor.minDistance = msg.min_distance
            sensor.maxDistance = msg.max_distance
            sensor.currDistance = msg.current_distance
            sensor.type = Type.fromType(msg.type.toInt())
            sensor.id = msg.id.toInt()
            sensor.orientation = Orientation.from(msg.orientation.toInt())
            sensor.covariance = msg.covariance
            return sensor
        }
    }
}
