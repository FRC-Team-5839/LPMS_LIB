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
	
    volatile byte[]      raw_gyro_x_0 = new byte[1];
    volatile byte[]      raw_gyro_x_1 = new byte[1];
    volatile byte[]      raw_gyro_x_2 = new byte[1];
    volatile byte[]      raw_gyro_x_3 = new byte[1];
    volatile byte[]      raw_gyro_x_all = new byte[4];
    volatile float      raw_gyro_x;
    
    /* Gyro X 4 bit */
    
    volatile byte[]      raw_gyro_y_0 = new byte[1];
    volatile byte[]      raw_gyro_y_1 = new byte[1];
    volatile byte[]      raw_gyro_y_2 = new byte[1];
    volatile byte[]      raw_gyro_y_3 = new byte[1];
    volatile byte[]      raw_gyro_y_all = new byte[4];
    volatile float      raw_gyro_y;
    
    /* Gyro Y 4 bit */
    
    volatile byte[]      raw_gyro_z_0 = new byte[1];
    volatile byte[]      raw_gyro_z_1 = new byte[1];
    volatile byte[]      raw_gyro_z_2 = new byte[1];
    volatile byte[]      raw_gyro_z_3 = new byte[1];
    volatile byte[]      raw_gyro_z_all = new byte[4];
    volatile float      raw_gyro_z;
    
    /* Gyro Z 4 bit */
    
    volatile byte[]      raw_accel_x_0 = new byte[1];
    volatile byte[]      raw_accel_x_1 = new byte[1];
    volatile byte[]      raw_accel_x_2 = new byte[1];
    volatile byte[]      raw_accel_x_3 = new byte[1];
    volatile byte[]      raw_accel_x_all = new byte[4];
    volatile float      raw_accel_x;
    
    volatile byte[]      raw_accel_y_0 = new byte[1];
    volatile byte[]      raw_accel_y_1 = new byte[1];
    volatile byte[]      raw_accel_y_2 = new byte[1];
    volatile byte[]      raw_accel_y_3 = new byte[1];
    volatile byte[]      raw_accel_y_all = new byte[4];
    volatile float      raw_accel_y;
    
    volatile byte[]      raw_accel_z_0 = new byte[1];
    volatile byte[]      raw_accel_z_1 = new byte[1];
    volatile byte[]      raw_accel_z_2 = new byte[1];
    volatile byte[]      raw_accel_z_3 = new byte[1];
    volatile byte[]      raw_accel_z_all = new byte[4];
    volatile float      raw_accel_z;
    
    volatile byte[]      cal_mag_x_0 = new byte[1];
    volatile byte[]      cal_mag_x_1 = new byte[1];
    volatile byte[]      cal_mag_x_2 = new byte[1];
    volatile byte[]      cal_mag_x_3 = new byte[1];
    volatile byte[]      cal_mag_x_all = new byte[4];
    volatile float      cal_mag_x;
    
    volatile byte[]      cal_mag_y_0 = new byte[1];
    volatile byte[]      cal_mag_y_1 = new byte[1];
    volatile byte[]      cal_mag_y_2 = new byte[1];
    volatile byte[]      cal_mag_y_3 = new byte[1];
    volatile byte[]      cal_mag_y_all = new byte[4];
    volatile float      cal_mag_y;
    
    volatile byte[]      cal_mag_z_0 = new byte[1];
    volatile byte[]      cal_mag_z_1 = new byte[1];
    volatile byte[]      cal_mag_z_2 = new byte[1];
    volatile byte[]      cal_mag_z_3 = new byte[1];
    volatile byte[]      cal_mag_z_all = new byte[4];
    volatile float      cal_mag_z;
	
    /* Processed Data */
    
    volatile byte[]      world_linear_accel_x_0 = new byte[1];
    volatile byte[]      world_linear_accel_x_1 = new byte[1];
    volatile byte[]      world_linear_accel_x_2 = new byte[1];
    volatile byte[]      world_linear_accel_x_3 = new byte[1];
    volatile byte[]      world_linear_accel_x_all = new byte[4];
    volatile float      world_linear_accel_x;
    
    volatile byte[]      world_linear_accel_y_0 = new byte[1];
    volatile byte[]      world_linear_accel_y_1 = new byte[1];
    volatile byte[]      world_linear_accel_y_2 = new byte[1];
    volatile byte[]      world_linear_accel_y_3 = new byte[1];
    volatile byte[]      world_linear_accel_y_all = new byte[4];
    volatile float      world_linear_accel_y;
    
    volatile byte[]      world_linear_accel_z_0 = new byte[1];
    volatile byte[]      world_linear_accel_z_1 = new byte[1];
    volatile byte[]      world_linear_accel_z_2 = new byte[1];
    volatile byte[]      world_linear_accel_z_3 = new byte[1];
    volatile byte[]      world_linear_accel_z_all = new byte[4];
    volatile float      world_linear_accel_z;
    
    volatile byte[]      mpu_temp_c_0 = new byte[1];
    volatile byte[]      mpu_temp_c_1 = new byte[1];
    volatile byte[]      mpu_temp_c_2 = new byte[1];
    volatile byte[]      mpu_temp_c_3 = new byte[1];
    volatile byte[]      mpu_temp_c_all = new byte[4];
    volatile float      mpu_temp_c;
    
    
    volatile byte[]      quaternionW_0 = new byte[1];
    volatile byte[]      quaternionW_1 = new byte[1];
    volatile byte[]      quaternionW_2 = new byte[1];
    volatile byte[]      quaternionW_3 = new byte[1];
    volatile byte[]      quaternionW_all = new byte[4];
    volatile float      quaternionW;
    
    volatile byte[]      quaternionX_0 = new byte[1];
    volatile byte[]      quaternionX_1 = new byte[1];
    volatile byte[]      quaternionX_2 = new byte[1];
    volatile byte[]      quaternionX_3 = new byte[1];
    volatile byte[]      quaternionX_all = new byte[4];
    volatile float      quaternionX;
    
    volatile byte[]      quaternionY_0 = new byte[1];
    volatile byte[]      quaternionY_1 = new byte[1];
    volatile byte[]      quaternionY_2 = new byte[1];
    volatile byte[]      quaternionY_3 = new byte[1];
    volatile byte[]      quaternionY_all = new byte[4];
    volatile float      quaternionY;
    
    volatile byte[]      quaternionZ_0 = new byte[1];
    volatile byte[]      quaternionZ_1 = new byte[1];
    volatile byte[]      quaternionZ_2 = new byte[1];
    volatile byte[]      quaternionZ_3 = new byte[1];
    volatile byte[]      quaternionZ_all = new byte[4];
    volatile float      quaternionZ;
    
    volatile byte[]		eulerX_0 = new byte[1];
    volatile byte[]		eulerX_1 = new byte[1];
    volatile byte[]		eulerX_2 = new byte[1];
    volatile byte[]		eulerX_3 = new byte[1];
    volatile byte[]		eulerX_all = new byte[4];
    volatile float		eulerX;
    
    volatile byte[]		eulerY_0 = new byte[1];
    volatile byte[]		eulerY_1 = new byte[1];
    volatile byte[]		eulerY_2 = new byte[1];
    volatile byte[]		eulerY_3 = new byte[1];
    volatile byte[]		eulerY_all = new byte[4];
    volatile float		eulerY;
    
    volatile byte[]		eulerZ_0 = new byte[1];
    volatile byte[]		eulerZ_1 = new byte[1];
    volatile byte[]		eulerZ_2 = new byte[1];
    volatile byte[]		eulerZ_3 = new byte[1];
    volatile byte[]		eulerZ_all = new byte[4];
    volatile float		eulerZ;
    
    
    /* Board ID */
    volatile byte[]       board_type = new byte[1];
    volatile byte[]       who_am_i = new byte[1];
    volatile byte[]       fw_ver_0 = new byte[1];
    volatile byte[]       fw_ver_1 = new byte[1];
    
    volatile byte[]		timestamp_0 = new byte[1];
    volatile byte[]		timestamp_1 = new byte[1];
    volatile byte[]		timestamp_2 = new byte[1];
    volatile byte[]		timestamp_3 = new byte[1];
    volatile byte[]		timestamp_all = new byte[1];
    volatile float		timestamp;
    
    RegisterIO_I2C		registerIO;
    
    public LPMS() {
    	registerIO = new RegisterIO_I2C(new I2C(Port.kMXP, 0x33));
    	
    }
    
	@Override
	public void initSendable(SendableBuilder builder) {
	      builder.setSmartDashboardType("LPMS IMU");
	      builder.addDoubleProperty("EulerX", this::getEulerX, null);
	      builder.addDoubleProperty("EulerY", this::getEulerY, null);
	      builder.addDoubleProperty("EulerZ", this::getEulerZ, null);
	      builder.addDoubleProperty("QuaternionW", this::getQuaternionW, null);
	      builder.addDoubleProperty("QuaternionX", this::getQuaternionX, null);
	      builder.addDoubleProperty("QuaternionY", this::getQuaternionY, null);
	      builder.addDoubleProperty("QuaternionZ", this::getQuaternionZ, null);
	      builder.addDoubleProperty("Linear_Accel_X", this::getWorld_linear_accel_x, null);
	      builder.addDoubleProperty("Linear_Accel_Y", this::getWorld_linear_accel_y, null);
	      builder.addDoubleProperty("Linear_Accel_Z", this::getWorld_linear_accel_z, null);
		
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return getEulerZ();
	}
	/**
	 * @return the raw_gyro_x_0
	 */
	public byte[] getRaw_gyro_x_0() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_X_0, this.raw_gyro_x_0);
	}
	/**
	 * @return the raw_gyro_x_1
	 */
	public byte[] getRaw_gyro_x_1() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_X_1, this.raw_gyro_x_1);
	}
	/**
	 * @return the raw_gyro_x_2
	 */
	public byte[] getRaw_gyro_x_2() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_X_2, this.raw_gyro_x_2);
	}
	/**
	 * @return the raw_gyro_x_3
	 */
	public byte[] getRaw_gyro_x_3() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_X_3, this.raw_gyro_x_3);
	}
	/**
	 * @return the raw_gyro_x_all
	 */
	public byte[] getRaw_gyro_x_all() {
		byte[] a = {getRaw_gyro_x_0()[0], getRaw_gyro_x_1()[0], getRaw_gyro_x_2()[0], getRaw_gyro_x_3()[0]};
		return a;
	}
	/**
	 * @return the raw_gyro_x
	 */
	public float getRaw_gyro_x() {
		return IMUProtocol.decodebyte2float(getRaw_gyro_x_all());
	}
	/**
	 * @return the raw_gyro_y_0
	 */
	public byte[] getRaw_gyro_y_0() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Y_0, this.raw_gyro_y_0);
	}
	/**
	 * @return the raw_gyro_y_1
	 */
	public byte[] getRaw_gyro_y_1() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Y_1, this.raw_gyro_y_1);
	}
	/**
	 * @return the raw_gyro_y_2
	 */
	public byte[] getRaw_gyro_y_2() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Y_2, this.raw_gyro_y_2);
	}
	/**
	 * @return the raw_gyro_y_3
	 */
	public byte[] getRaw_gyro_y_3() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Y_3, this.raw_gyro_y_3);
	}
	/**
	 * @return the raw_gyro_y_all
	 */
	public byte[] getRaw_gyro_y_all() {
		byte[] b = {getRaw_gyro_y_0()[0], getRaw_gyro_y_1()[0], getRaw_gyro_y_2()[0], getRaw_gyro_y_3()[0]};
		return b;
	}
	/**
	 * @return the raw_gyro_y
	 */
	public float getRaw_gyro_y() {
		return IMUProtocol.decodebyte2float(getRaw_gyro_y_all());
	}
	/**
	 * @return the raw_gyro_z_0
	 */
	public byte[] getRaw_gyro_z_0() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Z_0, this.raw_gyro_z_0);
	}
	/**
	 * @return the raw_gyro_z_1
	 */
	public byte[] getRaw_gyro_z_1() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Z_1, this.raw_gyro_z_1);
	}
	/**
	 * @return the raw_gyro_z_2
	 */
	public byte[] getRaw_gyro_z_2() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Z_2, this.raw_gyro_z_2);
	}
	/**
	 * @return the raw_gyro_z_3
	 */
	public byte[] getRaw_gyro_z_3() {
		return registerIO.readbyte(IMURegisters.LPMS_GYR_Z_3, this.raw_gyro_z_3);
	}
	/**
	 * @return the raw_gyro_z_all
	 */
	public byte[] getRaw_gyro_z_all() {
		byte[] c = {getRaw_gyro_z_0()[0], getRaw_gyro_z_1()[0], getRaw_gyro_z_2()[0], getRaw_gyro_z_3()[0]};
		return c;
	}
	/**
	 * @return the raw_gyro_z
	 */
	public float getRaw_gyro_z() {
		return IMUProtocol.decodebyte2float(getRaw_gyro_z_all());
	}
	/**
	 * @return the raw_accel_x_0
	 */
	public byte[] getRaw_accel_x_0() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_X_0, this.raw_accel_x_0);
	}
	/**
	 * @return the raw_accel_x_1
	 */
	public byte[] getRaw_accel_x_1() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_X_1, this.raw_accel_x_1);
	}
	/**
	 * @return the raw_accel_x_2
	 */
	public byte[] getRaw_accel_x_2() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_X_2, this.raw_accel_x_2);
	}
	/**
	 * @return the raw_accel_x_3
	 */
	public byte[] getRaw_accel_x_3() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_X_3, this.raw_accel_x_3);
	}
	/**
	 * @return the raw_accel_x_all
	 */
	public byte[] getRaw_accel_x_all() {
		byte[] b = {getRaw_accel_x_0()[0], getRaw_accel_x_1()[0], getRaw_accel_x_2()[0], getRaw_accel_x_3()[0]};
		return b;
	}
	/**
	 * @return the raw_accel_x
	 */
	public float getRaw_accel_x() {
		return IMUProtocol.decodebyte2float(getRaw_accel_x_all());
	}
	/**
	 * @return the raw_accel_y_0
	 */
	public byte[] getRaw_accel_y_0() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Y_0, this.raw_accel_y_0);
	}
	/**
	 * @return the raw_accel_y_1
	 */
	public byte[] getRaw_accel_y_1() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Y_1, this.raw_accel_y_1);
	}
	/**
	 * @return the raw_accel_y_2
	 */
	public byte[] getRaw_accel_y_2() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Y_2, this.raw_accel_y_2);
	}
	/**
	 * @return the raw_accel_y_3
	 */
	public byte[] getRaw_accel_y_3() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Y_3, this.raw_accel_y_3);
	}
	/**
	 * @return the raw_accel_y_all
	 */
	public byte[] getRaw_accel_y_all() {
		byte[] b = {getRaw_accel_y_0()[0], getRaw_accel_y_1()[0], getRaw_accel_y_2()[0], getRaw_accel_y_3()[0]};
		return b;
	}
	/**
	 * @return the raw_accel_y
	 */
	public float getRaw_accel_y() {
		return IMUProtocol.decodebyte2float(getRaw_accel_y_all());
	}
	/**
	 * @return the raw_accel_z_0
	 */
	public byte[] getRaw_accel_z_0() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Z_0, this.raw_accel_z_0);
	}
	/**
	 * @return the raw_accel_z_1
	 */
	public byte[] getRaw_accel_z_1() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Z_1, this.raw_accel_z_1);
	}
	/**
	 * @return the raw_accel_z_2
	 */
	public byte[] getRaw_accel_z_2() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Z_2, this.raw_accel_z_2);
	}
	/**
	 * @return the raw_accel_z_3
	 */
	public byte[] getRaw_accel_z_3() {
		return registerIO.readbyte(IMURegisters.LPMS_ACC_Z_3, this.raw_accel_z_3);
	}
	/**
	 * @return the raw_accel_z_all
	 */
	public byte[] getRaw_accel_z_all() {
		byte[] b = {getRaw_accel_z_0()[0], getRaw_accel_z_1()[0], getRaw_accel_z_2()[0], getRaw_accel_z_3()[0]};
		return b;
	}
	/**
	 * @return the raw_accel_z
	 */
	public float getRaw_accel_z() {
		return IMUProtocol.decodebyte2float(getRaw_accel_z_all());
	}
	/**
	 * @return the cal_mag_x_0
	 */
	public byte[] getCal_mag_x_0() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_X_0, this.cal_mag_x_0);
	}
	/**
	 * @return the cal_mag_x_1
	 */
	public byte[] getCal_mag_x_1() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_X_1, this.cal_mag_x_1);
	}
	/**
	 * @return the cal_mag_x_2
	 */
	public byte[] getCal_mag_x_2() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_X_2, this.cal_mag_x_2);
	}
	/**
	 * @return the cal_mag_x_3
	 */
	public byte[] getCal_mag_x_3() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_X_3, this.cal_mag_x_3);
	}
	/**
	 * @return the cal_mag_x_all
	 */
	public byte[] getCal_mag_x_all() {
		byte [] b = {getCal_mag_x_0()[0], getCal_mag_x_1()[0], getCal_mag_x_2()[0], getCal_mag_x_3()[0]};
		return b;
	}
	/**
	 * @return the cal_mag_x
	 */
	public float getCal_mag_x() {
		return IMUProtocol.decodebyte2float(getCal_mag_x_all());
	}
	/**
	 * @return the cal_mag_y_0
	 */
	public byte[] getCal_mag_y_0() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Y_0, this.cal_mag_y_0);
	}
	/**
	 * @return the cal_mag_y_1
	 */
	public byte[] getCal_mag_y_1() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Y_1, this.cal_mag_y_1);
	}
	/**
	 * @return the cal_mag_y_2
	 */
	public byte[] getCal_mag_y_2() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Y_2, this.cal_mag_y_2);
	}
	/**
	 * @return the cal_mag_y_3
	 */
	public byte[] getCal_mag_y_3() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Y_3, this.cal_mag_y_3);
	}
	/**
	 * @return the cal_mag_y_all
	 */
	public byte[] getCal_mag_y_all() {
		byte[] b = {getCal_mag_y_0()[0], getCal_mag_y_1()[0], getCal_mag_y_2()[0], getCal_mag_y_3()[0]};
		return b;
	}
	/**
	 * @return the cal_mag_y
	 */
	public float getCal_mag_y() {
		return IMUProtocol.decodebyte2float(getCal_mag_y_all());
	}
	/**
	 * @return the cal_mag_z_0
	 */
	public byte[] getCal_mag_z_0() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Z_0, this.cal_mag_z_0);
	}
	/**
	 * @return the cal_mag_z_1
	 */
	public byte[] getCal_mag_z_1() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Z_1, this.cal_mag_z_1);
	}
	/**
	 * @return the cal_mag_z_2
	 */
	public byte[] getCal_mag_z_2() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Z_2, this.cal_mag_z_2);
	}
	/**
	 * @return the cal_mag_z_3
	 */
	public byte[] getCal_mag_z_3() {
		return registerIO.readbyte(IMURegisters.LPMS_MAG_Z_3, this.cal_mag_z_3);
	}
	/**
	 * @return the cal_mag_z_all
	 */
	public byte[] getCal_mag_z_all() {
		byte [] b = {getCal_mag_z_0()[0], getCal_mag_z_1()[0], getCal_mag_z_2()[0], getCal_mag_z_3()[0]};
		return b;
	}
	/**
	 * @return the cal_mag_z
	 */
	public float getCal_mag_z() {
		return IMUProtocol.decodebyte2float(getCal_mag_z_all());
	}
	/**
	 * @return the world_linear_accel_x_0
	 */
	public byte[] getWorld_linear_accel_x_0() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_X_0, this.world_linear_accel_x_0);
	}
	/**
	 * @return the world_linear_accel_x_1
	 */
	public byte[] getWorld_linear_accel_x_1() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_X_1, this.world_linear_accel_x_1);
	}
	/**
	 * @return the world_linear_accel_x_2
	 */
	public byte[] getWorld_linear_accel_x_2() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_X_2, this.world_linear_accel_x_2);
	}
	/**
	 * @return the world_linear_accel_x_3
	 */
	public byte[] getWorld_linear_accel_x_3() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_X_3, this.world_linear_accel_x_3);
	}
	/**
	 * @return the world_linear_accel_x_all
	 */
	public byte[] getWorld_linear_accel_x_all() {
		byte[] b = {getWorld_linear_accel_x_0()[0], getWorld_linear_accel_x_1()[0], getWorld_linear_accel_x_2()[0], getWorld_linear_accel_x_3()[0]};
		return b;
	}
	/**
	 * @return the world_linear_accel_x
	 */
	public float getWorld_linear_accel_x() {
		return IMUProtocol.decodebyte2float(getWorld_linear_accel_x_all());
	}
	/**
	 * @return the world_linear_accel_y_0
	 */
	public byte[] getWorld_linear_accel_y_0() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Y_0, this.world_linear_accel_y_0);
	}
	/**
	 * @return the world_linear_accel_y_1
	 */
	public byte[] getWorld_linear_accel_y_1() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Y_1, this.world_linear_accel_y_1);
	}
	/**
	 * @return the world_linear_accel_y_2
	 */
	public byte[] getWorld_linear_accel_y_2() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Y_2, this.world_linear_accel_y_2);
	}
	/**
	 * @return the world_linear_accel_y_3
	 */
	public byte[] getWorld_linear_accel_y_3() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Y_3, this.world_linear_accel_y_3);
	}
	/**
	 * @return the world_linear_accel_y_all
	 */
	public byte[] getWorld_linear_accel_y_all() {
		byte[] b = {getWorld_linear_accel_y_0()[0], getWorld_linear_accel_y_1()[0], getWorld_linear_accel_y_2()[0], getWorld_linear_accel_y_3()[0]};
		return b;
	}
	/**
	 * @return the world_linear_accel_y
	 */
	public float getWorld_linear_accel_y() {
		return IMUProtocol.decodebyte2float(getWorld_linear_accel_y_all());
	}
	/**
	 * @return the world_linear_accel_z_0
	 */
	public byte[] getWorld_linear_accel_z_0() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Z_0, this.world_linear_accel_z_0);
	}
	/**
	 * @return the world_linear_accel_z_1
	 */
	public byte[] getWorld_linear_accel_z_1() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Z_1, this.world_linear_accel_z_1);
	}
	/**
	 * @return the world_linear_accel_z_2
	 */
	public byte[] getWorld_linear_accel_z_2() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Z_2, this.world_linear_accel_z_2);
	}
	/**
	 * @return the world_linear_accel_z_3
	 */
	public byte[] getWorld_linear_accel_z_3() {
		return registerIO.readbyte(IMURegisters.LPMS_LIN_ACC_Z_3, this.world_linear_accel_z_3);
	}
	/**
	 * @return the world_linear_accel_z_all
	 */
	public byte[] getWorld_linear_accel_z_all() {
		byte[] b = {getWorld_linear_accel_z_0()[0], getWorld_linear_accel_z_1()[0], getWorld_linear_accel_z_2()[0], getWorld_linear_accel_z_3()[0]};
		return b;
	}
	/**
	 * @return the world_linear_accel_z
	 */
	public float getWorld_linear_accel_z() {
		return IMUProtocol.decodebyte2float(getWorld_linear_accel_z_all());
	}
	/**
	 * @return the mpu_temp_c_0
	 */
	public byte[] getMpu_temp_c_0() {
		return registerIO.readbyte(IMURegisters.LPMS_TEMP_0, this.mpu_temp_c_0);
	}
	/**
	 * @return the mpu_temp_c_1
	 */
	public byte[] getMpu_temp_c_1() {
		return registerIO.readbyte(IMURegisters.LPMS_TEMP_1, this.mpu_temp_c_1);
	}
	/**
	 * @return the mpu_temp_c_2
	 */
	public byte[] getMpu_temp_c_2() {
		return registerIO.readbyte(IMURegisters.LPMS_TEMP_2, this.mpu_temp_c_2);
	}
	/**
	 * @return the mpu_temp_c_3
	 */
	public byte[] getMpu_temp_c_3() {
		return registerIO.readbyte(IMURegisters.LPMS_TEMP_3, this.mpu_temp_c_3);
	}
	/**
	 * @return the mpu_temp_c_all
	 */
	public byte[] getMpu_temp_c_all() {
		byte[] b = {getMpu_temp_c_0()[0], getMpu_temp_c_1()[0], getMpu_temp_c_2()[0], getMpu_temp_c_3()[0]};
		return b;
	}
	/**
	 * @return the mpu_temp_c
	 */
	public float getMpu_temp_c() {
		return IMUProtocol.decodebyte2float(getMpu_temp_c_all());
	}
	/**
	 * @return the quaternionW_0
	 */
	public byte[] getQuaternionW_0() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_W_0, this.quaternionW_0);
	}
	/**
	 * @return the quaternionW_1
	 */
	public byte[] getQuaternionW_1() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_W_1, this.quaternionW_1);
	}
	/**
	 * @return the quaternionW_2
	 */
	public byte[] getQuaternionW_2() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_W_2, this.quaternionW_2);
	}
	/**
	 * @return the quaternionW_3
	 */
	public byte[] getQuaternionW_3() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_W_3, this.quaternionW_3);
	}
	/**
	 * @return the quaternionW_all
	 */
	public byte[] getQuaternionW_all() {
		byte[] b = {getQuaternionW_0()[0], getQuaternionW_1()[0], getQuaternionW_2()[0], getQuaternionW_3()[0]};
		return b;
	}
	/**
	 * @return the quaternionW
	 */
	public float getQuaternionW() {
		return IMUProtocol.decodebyte2float(getQuaternionW_all());
	}
	/**
	 * @return the quaternionX_0
	 */
	public byte[] getQuaternionX_0() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_X_0, this.quaternionX_0);
	}
	/**
	 * @return the quaternionX_1
	 */
	public byte[] getQuaternionX_1() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_X_1, this.quaternionX_1);
	}
	/**
	 * @return the quaternionX_2
	 */
	public byte[] getQuaternionX_2() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_X_2, this.quaternionX_2);
	}
	/**
	 * @return the quaternionX_3
	 */
	public byte[] getQuaternionX_3() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_X_3, this.quaternionX_3);
	}
	/**
	 * @return the quaternionX_all
	 */
	public byte[] getQuaternionX_all() {
		byte[] b = {getQuaternionX_0()[0], getQuaternionX_1()[0], getQuaternionX_2()[0], getQuaternionX_3()[0]};
		return b;
	}
	/**
	 * @return the quaternionX
	 */
	public float getQuaternionX() {
		return IMUProtocol.decodebyte2float(getQuaternionX_all());
	}
	/**
	 * @return the quaternionY_0
	 */
	public byte[] getQuaternionY_0() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Y_0, this.quaternionY_0);
	}
	/**
	 * @return the quaternionY_1
	 */
	public byte[] getQuaternionY_1() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Y_1, this.quaternionY_1);
	}
	/**
	 * @return the quaternionY_2
	 */
	public byte[] getQuaternionY_2() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Y_2, this.quaternionY_2);
	}
	/**
	 * @return the quaternionY_3
	 */
	public byte[] getQuaternionY_3() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Y_3, this.quaternionY_3);
	}
	/**
	 * @return the quaternionY_all
	 */
	public byte[] getQuaternionY_all() {
		byte[] b = {getQuaternionY_0()[0], getQuaternionY_1()[0], getQuaternionY_2()[0], getQuaternionY_3()[0]};
		return b;
	}
	/**
	 * @return the quaternionY
	 */
	public float getQuaternionY() {
		return IMUProtocol.decodebyte2float(getQuaternionY_all());
	}
	/**
	 * @return the quaternionZ_0
	 */
	public byte[] getQuaternionZ_0() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Z_0, this.quaternionZ_0);
	}
	/**
	 * @return the quaternionZ_1
	 */
	public byte[] getQuaternionZ_1() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Z_1, this.quaternionZ_1);
	}
	/**
	 * @return the quaternionZ_2
	 */
	public byte[] getQuaternionZ_2() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Z_2, this.quaternionZ_2);
	}
	/**
	 * @return the quaternionZ_3
	 */
	public byte[] getQuaternionZ_3() {
		return registerIO.readbyte(IMURegisters.LPMS_QUAT_Z_3, this.quaternionZ_3);
	}
	/**
	 * @return the quaternionZ_all
	 */
	public byte[] getQuaternionZ_all() {
		byte[] b = {getQuaternionZ_0()[0], getQuaternionZ_1()[0], getQuaternionZ_2()[0], getQuaternionZ_3()[0]};
		return b;
	}
	/**
	 * @return the quaternionZ
	 */
	public float getQuaternionZ() {
		return IMUProtocol.decodebyte2float(getQuaternionZ_all());
	}
	/**
	 * @return the eulerX_0
	 */
	public byte[] getEulerX_0() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_X_0, this.eulerX_0);
	}
	/**
	 * @return the eulerX_1
	 */
	public byte[] getEulerX_1() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_X_1, this.eulerX_1);
	}
	/**
	 * @return the eulerX_2
	 */
	public byte[] getEulerX_2() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_X_2, this.eulerX_2);
	}
	/**
	 * @return the eulerX_3
	 */
	public byte[] getEulerX_3() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_X_3, this.eulerX_3);
	}
	/**
	 * @return the eulerX_all
	 */
	public byte[] getEulerX_all() {
		byte[] b = {getEulerX_0()[0], getEulerX_1()[0], getEulerX_2()[0], getEulerX_3()[0]};
		return b;
	}
	/**
	 * @return the eulerX
	 */
	public float getEulerX() {
		return IMUProtocol.decodebyte2float(getEulerX_all());
	}
	/**
	 * @return the eulerY_0
	 */
	public byte[] getEulerY_0() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Y_0, this.eulerY_0);
	}
	/**
	 * @return the eulerY_1
	 */
	public byte[] getEulerY_1() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Y_1, this.eulerY_1);
	}
	/**
	 * @return the eulerY_2
	 */
	public byte[] getEulerY_2() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Y_2, this.eulerY_2);
	}
	/**
	 * @return the eulerY_3
	 */
	public byte[] getEulerY_3() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Y_3, this.eulerY_3);
	}
	/**
	 * @return the eulerY_all
	 */
	public byte[] getEulerY_all() {
		byte[] b = {getEulerY_0()[0], getEulerY_1()[0], getEulerY_2()[0], getEulerY_3()[0]};
		return b;
	}
	/**
	 * @return the eulerY
	 */
	public float getEulerY() {
		return IMUProtocol.decodebyte2float(getEulerY_all());
	}
	/**
	 * @return the eulerZ_0
	 */
	public byte[] getEulerZ_0() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Z_0, this.eulerZ_0);
	}
	/**
	 * @return the eulerZ_1
	 */
	public byte[] getEulerZ_1() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Z_1, this.eulerZ_1);
	}
	/**
	 * @return the eulerZ_2
	 */
	public byte[] getEulerZ_2() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Z_2, this.eulerZ_2);
	}
	/**
	 * @return the eulerZ_3
	 */
	public byte[] getEulerZ_3() {
		return registerIO.readbyte(IMURegisters.LPMS_EULER_Z_3, this.eulerZ_3);
	}
	/**
	 * @return the eulerZ_all
	 */
	public byte[] getEulerZ_all() {
		byte[] b = {getEulerZ_0()[0], getEulerZ_1()[0], getEulerZ_2()[0], getEulerZ_3()[0]};
		return b;
	}
	/**
	 * @return the eulerZ
	 */
	public float getEulerZ() {
		return IMUProtocol.decodebyte2float(getEulerZ_all());
	}
	/**
	 * @return the board_type
	 */
	public byte[] getBoard_type() {
		return board_type;
	}
	/**
	 * @return the who_am_i
	 */
	public byte[] getWho_am_i() {
		return registerIO.readbyte(IMURegisters.LPMS_WHO_AM_I, this.who_am_i);
	}
	/**
	 * @return the fw_ver_0
	 */
	public byte[] getFw_ver_0() {
		return registerIO.readbyte(IMURegisters.LPMS_FIRMWARE_VERSION_0, this.fw_ver_0);
	}
	/**
	 * @return the fw_ver_1
	 */
	public byte[] getFw_ver_1() {
		return registerIO.readbyte(IMURegisters.LPMS_FIRMWARE_VERSION_1, this.fw_ver_1);
	}
	/**
	 * @return the timestamp_0
	 */
	public byte[] getTimestamp_0() {
		return registerIO.readbyte(IMURegisters.LPMS_TIMESTAMP_0, this.timestamp_0);
	}
	/**
	 * @return the timestamp_1
	 */
	public byte[] getTimestamp_1() {
		return registerIO.readbyte(IMURegisters.LPMS_TIMESTAMP_1, this.timestamp_1);
	}
	/**
	 * @return the timestamp_2
	 */
	public byte[] getTimestamp_2() {
		return registerIO.readbyte(IMURegisters.LPMS_TIMESTAMP_2, this.timestamp_2);
	}
	/**
	 * @return the timestamp_3
	 */
	public byte[] getTimestamp_3() {
		return registerIO.readbyte(IMURegisters.LPMS_TIMESTAMP_3, this.timestamp_3);
	}
	/**
	 * @return the timestamp_all
	 */
	public byte[] getTimestamp_all() {
		byte[] b = {getTimestamp_0()[0], getTimestamp_1()[0], getTimestamp_2()[0], getTimestamp_3()[0]};
		return b;
	}
	/**
	 * @return the timestamp
	 */
	public float getTimestamp() {
		return IMUProtocol.decodebyte2float(getTimestamp_all());
	}
	

}
