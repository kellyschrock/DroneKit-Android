/* AUTO-GENERATED FILE.  DO NOT MODIFY.
 *
 * This class was automatically generated by the
 * java mavlink generator tool. It should not be modified by hand.
 */

// MESSAGE VISION_POSITION_DELTA PACKING
package com.MAVLink.ardupilotmega;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        
/**
 * camera vision based attitude and position deltas
 */
public class msg_vision_position_delta extends MAVLinkMessage {

    public static final int MAVLINK_MSG_ID_VISION_POSITION_DELTA = 11011;
    public static final int MAVLINK_MSG_LENGTH = 44;
    private static final long serialVersionUID = MAVLINK_MSG_ID_VISION_POSITION_DELTA;


      
    /**
     * Timestamp (microseconds, synced to UNIX time or since system boot)
     */
    public long time_usec;
      
    /**
     * Time in microseconds since the last reported camera frame
     */
    public long time_delta_usec;
      
    /**
     * Defines a rotation vector in body frame that rotates the vehicle from the previous to the current orientation
     */
    public float angle_delta[] = new float[3];
      
    /**
     * Change in position in meters from previous to current frame rotated into body frame (0=forward, 1=right, 2=down)
     */
    public float position_delta[] = new float[3];
      
    /**
     * normalised confidence value from 0 to 100
     */
    public float confidence;
    

    /**
     * Generates the payload for a mavlink message for a message of this type
     * @return
     */
    public MAVLinkPacket pack() {
        MAVLinkPacket packet = new MAVLinkPacket(MAVLINK_MSG_LENGTH);
        packet.sysid = 255;
        packet.compid = 190;
        packet.msgid = MAVLINK_MSG_ID_VISION_POSITION_DELTA;
        
        packet.payload.putUnsignedLong(time_usec);
        
        packet.payload.putUnsignedLong(time_delta_usec);
        
        
        for (int i = 0; i < angle_delta.length; i++) {
            packet.payload.putFloat(angle_delta[i]);
        }
                    
        
        
        for (int i = 0; i < position_delta.length; i++) {
            packet.payload.putFloat(position_delta[i]);
        }
                    
        
        packet.payload.putFloat(confidence);
        
        if(isMavlink2) {
            
        }
        return packet;
    }

    /**
     * Decode a vision_position_delta message into this class fields
     *
     * @param payload The message to decode
     */
    public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
        
        this.time_usec = payload.getUnsignedLong();
        
        this.time_delta_usec = payload.getUnsignedLong();
        
         
        for (int i = 0; i < this.angle_delta.length; i++) {
            try { this.angle_delta[i] = payload.getFloat(); } catch(IndexOutOfBoundsException ex) { break; }
        }
                
        
         
        for (int i = 0; i < this.position_delta.length; i++) {
            try { this.position_delta[i] = payload.getFloat(); } catch(IndexOutOfBoundsException ex) { break; }
        }
                
        
        this.confidence = payload.getFloat();
        
        if(isMavlink2) {
            
        }
    }

    /**
     * Constructor for a new message, just initializes the msgid
     */
    public msg_vision_position_delta() {
        msgid = MAVLINK_MSG_ID_VISION_POSITION_DELTA;
    }

    /**
     * Constructor for a new message, initializes the message with the payload
     * from a mavlink packet
     *
     */
    public msg_vision_position_delta(MAVLinkPacket mavLinkPacket) {
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_VISION_POSITION_DELTA;
        this.isMavlink2 = mavLinkPacket.isMavlink2;
        unpack(mavLinkPacket.payload);        
    }

              
    /**
     * Returns a string with the MSG name and data
     */
    public String toString() {
        return "MAVLINK_MSG_ID_VISION_POSITION_DELTA - sysid:"+sysid+" compid:"+compid+" time_usec:"+time_usec+" time_delta_usec:"+time_delta_usec+" angle_delta:"+angle_delta+" position_delta:"+position_delta+" confidence:"+confidence+"";
    }
}
        