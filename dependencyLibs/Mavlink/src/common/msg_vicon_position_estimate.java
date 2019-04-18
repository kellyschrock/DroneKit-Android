/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE VICON_POSITION_ESTIMATE PACKING
package com.MAVLink.common;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
 * 
 */
public class msg_vicon_position_estimate extends MAVLinkMessage {

    public static final int MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE = 104;
    public static final int MAVLINK_MSG_LENGTH = 116;
    private static final long serialVersionUID = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;


      
    /**
     * Timestamp (microseconds, synced to UNIX time or since system boot)
     */
    public long usec;
      
    /**
     * Global X position
     */
    public float x;
      
    /**
     * Global Y position
     */
    public float y;
      
    /**
     * Global Z position
     */
    public float z;
      
    /**
     * Roll angle in rad
     */
    public float roll;
      
    /**
     * Pitch angle in rad
     */
    public float pitch;
      
    /**
     * Yaw angle in rad
     */
    public float yaw;
      
    /**
     * Pose covariance matrix upper right triangular (first six entries are the first ROW, next five entries are the second ROW, etc.)
     */
    public float covariance[] = new float[21];
    

    /**
     * Generates the payload for a mavlink message for a message of this type
     * @return
     */
    public MAVLinkPacket pack() {
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
        
        packet.payload.putUnsignedLong(usec);
        
        packet.payload.putFloat(x);
        
        packet.payload.putFloat(y);
        
        packet.payload.putFloat(z);
        
        packet.payload.putFloat(roll);
        
        packet.payload.putFloat(pitch);
        
        packet.payload.putFloat(yaw);
        
        if(isMavlink2) {
            
            
        for (int i = 0; i < covariance.length; i++) {
            packet.payload.putFloat(covariance[i]);
        }
                    
            
        }
        return packet;
    }

    /**
     * Decode a vicon_position_estimate message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
        
        this.usec = payload.getUnsignedLong();
        
        this.x = payload.getFloat();
        
        this.y = payload.getFloat();
        
        this.z = payload.getFloat();
        
        this.roll = payload.getFloat();
        
        this.pitch = payload.getFloat();
        
        this.yaw = payload.getFloat();
        
        if(isMavlink2) {
            
             
        for (int i = 0; i < this.covariance.length; i++) {
            try { this.covariance[i] = payload.getFloat(); } catch(IndexOutOfBoundsException ex) { break; }
        }
                
            
        }
    }

    /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_vicon_position_estimate() {
        msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a mavlink packet
     *
     */
    public msg_vicon_position_estimate(MAVLinkPacket mavLinkPacket) {
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
        this.isMavlink2 = mavLinkPacket.isMavlink2;
        unpack(mavLinkPacket.payload);        
    }

                    
    /**
     * Returns a string with the MSG name and data
     */
    public String toString() {
        return "MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE - sysid:"+sysid+" compid:"+compid+" usec:"+usec+" x:"+x+" y:"+y+" z:"+z+" roll:"+roll+" pitch:"+pitch+" yaw:"+yaw+" covariance:"+covariance+"";
    }
}
        