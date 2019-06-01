/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE ALTITUDE PACKING
package com.MAVLink.common;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
 * The current system altitude.
 */
public class msg_altitude extends MAVLinkMessage {

    public static final int MAVLINK_MSG_ID_ALTITUDE = 141;
    public static final int MAVLINK_MSG_LENGTH = 32;
    private static final long serialVersionUID = MAVLINK_MSG_ID_ALTITUDE;


      
    /**
     * Timestamp (micros since boot or Unix epoch)
     */
    public long time_usec;
      
    /**
     * This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
     */
    public float altitude_monotonic;
      
    /**
     * This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
     */
    public float altitude_amsl;
      
    /**
     * This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
     */
    public float altitude_local;
      
    /**
     * This is the altitude above the home position. It resets on each change of the current home position.
     */
    public float altitude_relative;
      
    /**
     * This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
     */
    public float altitude_terrain;
      
    /**
     * This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
     */
    public float bottom_clearance;
    

    /**
     * Generates the payload for a mavlink message for a message of this type
     * @return
     */
    public MAVLinkPacket pack() {
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH,isMavlink2);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_ALTITUDE;
        
        packet.payload.putUnsignedLong(time_usec);
        
        packet.payload.putFloat(altitude_monotonic);
        
        packet.payload.putFloat(altitude_amsl);
        
        packet.payload.putFloat(altitude_local);
        
        packet.payload.putFloat(altitude_relative);
        
        packet.payload.putFloat(altitude_terrain);
        
        packet.payload.putFloat(bottom_clearance);
        
        if(isMavlink2) {
            
        }
        return packet;
    }

    /**
     * Decode a altitude message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
        
        this.time_usec = payload.getUnsignedLong();
        
        this.altitude_monotonic = payload.getFloat();
        
        this.altitude_amsl = payload.getFloat();
        
        this.altitude_local = payload.getFloat();
        
        this.altitude_relative = payload.getFloat();
        
        this.altitude_terrain = payload.getFloat();
        
        this.bottom_clearance = payload.getFloat();
        
        if(isMavlink2) {
            
        }
    }

    /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_altitude() {
        msgid = MAVLINK_MSG_ID_ALTITUDE;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a mavlink packet
     *
     */
    public msg_altitude(MAVLinkPacket mavLinkPacket) {
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_ALTITUDE;
        this.isMavlink2 = mavLinkPacket.isMavlink2;
        unpack(mavLinkPacket.payload);        
    }

                  
    /**
     * Returns a string with the MSG name and data
     */
    public String toString() {
        return "MAVLINK_MSG_ID_ALTITUDE - sysid:"+sysid+" compid:"+compid+" time_usec:"+time_usec+" altitude_monotonic:"+altitude_monotonic+" altitude_amsl:"+altitude_amsl+" altitude_local:"+altitude_local+" altitude_relative:"+altitude_relative+" altitude_terrain:"+altitude_terrain+" bottom_clearance:"+bottom_clearance+"";
    }
}
        