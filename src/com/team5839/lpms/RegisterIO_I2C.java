package com.team5839.lpms;

import edu.wpi.first.wpilibj.I2C;

public class RegisterIO_I2C{

    I2C port;
    boolean trace = false;
    
    public RegisterIO_I2C( I2C i2c_port ) {
        port = i2c_port;
    }
    
    public boolean write(byte address, byte value ) {
    	boolean aborted;
    	synchronized(this){
	        aborted = port.write(address | 0x33, value);
    	}
        if ( aborted && trace ) System.out.println("LPMS I2C Write Error");
        return !aborted;
    }

    final static int MAX_WPILIB_I2C_READ_BYTES = 127;
    
    public boolean read(byte addr, byte[] data) {
    	boolean read_aborted = true;
    		read_aborted = port.read(addr, 1, data);
    	if (read_aborted) {
    		System.out.println("LPMS I2C Read Error");
		}
		return read_aborted;
    	
    }
    
    public byte[] readbyte(byte addr, byte[] data) {
    	boolean read_aborted = true;
    		read_aborted = port.read(addr, 1, data);
    	if (read_aborted) {
    		System.out.println("LPMS I2C Read Error");
		}
		return data;
    	
    }   

}
