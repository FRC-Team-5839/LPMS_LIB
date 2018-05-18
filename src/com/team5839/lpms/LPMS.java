package com.team5839.lpms;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class LPMS extends SensorBase implements PIDSource, Sendable {
	
	
	
    /* Raw Data */
	
    volatile float      raw_gyro_x_0;
    volatile float      raw_gyro_x_1;
    volatile float      raw_gyro_x_2;
    volatile float      raw_gyro_x_3;
    volatile float      raw_gyro_x;
    
    /* Gyro X 4 bit */
    
    volatile float      raw_gyro_y_0;
    volatile float      raw_gyro_y_1;
    volatile float      raw_gyro_y_2;
    volatile float      raw_gyro_y_3;
    volatile float      raw_gyro_y;
    
    /* Gyro Y 4 bit */
    
    volatile float      raw_gyro_z_0;
    volatile float      raw_gyro_z_1;
    volatile float      raw_gyro_z_2;
    volatile float      raw_gyro_z_3;
    volatile float      raw_gyro_z;
    
    /* Gyro Z 4 bit */
    
    volatile float      raw_accel_x_0;
    volatile float      raw_accel_x_1;
    volatile float      raw_accel_x_2;
    volatile float      raw_accel_x_3;
    volatile float      raw_accel_x;
    
    volatile float      raw_accel_y_0;
    volatile float      raw_accel_y_1;
    volatile float      raw_accel_y_2;
    volatile float      raw_accel_y_3;
    volatile float      raw_accel_y;
    
    volatile float      raw_accel_z_0;
    volatile float      raw_accel_z_1;
    volatile float      raw_accel_z_2;
    volatile float      raw_accel_z_3;
    volatile float      raw_accel_z;
    
    volatile float      cal_mag_x_0;
    volatile float      cal_mag_x_1;
    volatile float      cal_mag_x_2;
    volatile float      cal_mag_x_3;
    volatile float      cal_mag_x;
    
    volatile float      cal_mag_y_0;
    volatile float      cal_mag_y_1;
    volatile float      cal_mag_y_2;
    volatile float      cal_mag_y_3;
    volatile float      cal_mag_y;
    
    volatile float      cal_mag_z_0;
    volatile float      cal_mag_z_1;
    volatile float      cal_mag_z_2;
    volatile float      cal_mag_z_3;
    volatile float      cal_mag_z;
	
    /* Processed Data */
    
    volatile float      world_linear_accel_x_0;
    volatile float      world_linear_accel_x_1;
    volatile float      world_linear_accel_x_2;
    volatile float      world_linear_accel_x_3;
    volatile float      world_linear_accel_x;
    
    volatile float      world_linear_accel_y_0;
    volatile float      world_linear_accel_y_1;
    volatile float      world_linear_accel_y_2;
    volatile float      world_linear_accel_y_3;
    volatile float      world_linear_accel_y;
    
    volatile float      world_linear_accel_z_0;
    volatile float      world_linear_accel_z_1;
    volatile float      world_linear_accel_z_2;
    volatile float      world_linear_accel_z_3;
    volatile float      world_linear_accel_z;
    
    volatile float      mpu_temp_c_0;
    volatile float      mpu_temp_c_1;
    volatile float      mpu_temp_c_2;
    volatile float      mpu_temp_c_3;
    volatile float      mpu_temp_c;
    
    
    volatile float      quaternionW_0;
    volatile float      quaternionW_1;
    volatile float      quaternionW_2;
    volatile float      quaternionW_3;
    volatile float      quaternionW;
    
    volatile float      quaternionX_0;
    volatile float      quaternionX_1;
    volatile float      quaternionX_2;
    volatile float      quaternionX_3;
    volatile float      quaternionX;
    
    volatile float      quaternionY_0;
    volatile float      quaternionY_1;
    volatile float      quaternionY_2;
    volatile float      quaternionY_3;
    volatile float      quaternionY;
    
    volatile float      quaternionZ_0;
    volatile float      quaternionZ_1;
    volatile float      quaternionZ_2;
    volatile float      quaternionZ_3;
    volatile float      quaternionZ;
    
    volatile float		eulerX_0;
    volatile float		eulerX_1;
    volatile float		eulerX_2;
    volatile float		eulerX_3;
    volatile float		eulerX;
    
    volatile float		eulerY_0;
    volatile float		eulerY_1;
    volatile float		eulerY_2;
    volatile float		eulerY_3;
    volatile float		eulerY;
    
    volatile float		eulerZ_0;
    volatile float		eulerZ_1;
    volatile float		eulerZ_2;
    volatile float		eulerZ_3;
    volatile float		eulerZ;
    
    
    /* Board ID */
    volatile byte       board_type;
    volatile byte       who_am_i;
    volatile byte       fw_ver_0;
    volatile byte       fw_ver_1;
    
    volatile float		timestamp;
    
    RegisterIO_I2C		registerIO;
    
    /**
     * Constructs the AHRS class using I2C communication, overriding the 
     * default update rate with a custom rate which may be from 4 to 200, 
     * representing the number of updates per second sent by the sensor.  
     *<p>
     * This constructor should be used if communicating via I2C.
     *<p>
     * Note that increasing the update rate may increase the 
     * CPU utilization.
     *<p>
     * @param i2c_port_id I2C Port to use
     * @param update_rate_hz Custom Update Rate (Hz)
     */
    public LPMS() {
    	registerIO = new RegisterIO_I2C(new I2C(Port.kMXP, 0x33));
    	
    }
    
    
    
	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return 0;
	}

}
