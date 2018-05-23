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
	
    volatile byte      raw_gyro_x_0;
    volatile byte      raw_gyro_x_1;
    volatile byte      raw_gyro_x_2;
    volatile byte      raw_gyro_x_3;
    volatile float      raw_gyro_x;
    
    /* Gyro X 4 bit */
    
    volatile byte      raw_gyro_y_0;
    volatile byte      raw_gyro_y_1;
    volatile byte      raw_gyro_y_2;
    volatile byte      raw_gyro_y_3;
    volatile float      raw_gyro_y;
    
    /* Gyro Y 4 bit */
    
    volatile byte      raw_gyro_z_0;
    volatile byte      raw_gyro_z_1;
    volatile byte      raw_gyro_z_2;
    volatile byte      raw_gyro_z_3;
    volatile float      raw_gyro_z;
    
    /* Gyro Z 4 bit */
    
    volatile byte      raw_accel_x_0;
    volatile byte      raw_accel_x_1;
    volatile byte      raw_accel_x_2;
    volatile byte      raw_accel_x_3;
    volatile float      raw_accel_x;
    
    volatile byte      raw_accel_y_0;
    volatile byte      raw_accel_y_1;
    volatile byte      raw_accel_y_2;
    volatile byte      raw_accel_y_3;
    volatile float      raw_accel_y;
    
    volatile byte      raw_accel_z_0;
    volatile byte      raw_accel_z_1;
    volatile byte      raw_accel_z_2;
    volatile byte      raw_accel_z_3;
    volatile float      raw_accel_z;
    
    volatile byte      cal_mag_x_0;
    volatile byte      cal_mag_x_1;
    volatile byte      cal_mag_x_2;
    volatile byte      cal_mag_x_3;
    volatile float      cal_mag_x;
    
    volatile byte      cal_mag_y_0;
    volatile byte      cal_mag_y_1;
    volatile byte      cal_mag_y_2;
    volatile byte      cal_mag_y_3;
    volatile float      cal_mag_y;
    
    volatile byte      cal_mag_z_0;
    volatile byte      cal_mag_z_1;
    volatile byte      cal_mag_z_2;
    volatile byte      cal_mag_z_3;
    volatile float      cal_mag_z;
	
    /* Processed Data */
    
    volatile byte      world_linear_accel_x_0;
    volatile byte      world_linear_accel_x_1;
    volatile byte      world_linear_accel_x_2;
    volatile byte      world_linear_accel_x_3;
    volatile float      world_linear_accel_x;
    
    volatile byte      world_linear_accel_y_0;
    volatile byte      world_linear_accel_y_1;
    volatile byte      world_linear_accel_y_2;
    volatile byte      world_linear_accel_y_3;
    volatile float      world_linear_accel_y;
    
    volatile byte      world_linear_accel_z_0;
    volatile byte      world_linear_accel_z_1;
    volatile byte      world_linear_accel_z_2;
    volatile byte      world_linear_accel_z_3;
    volatile float      world_linear_accel_z;
    
    volatile byte      mpu_temp_c_0;
    volatile byte      mpu_temp_c_1;
    volatile byte      mpu_temp_c_2;
    volatile byte      mpu_temp_c_3;
    volatile float      mpu_temp_c;
    
    
    volatile byte      quaternionW_0;
    volatile byte      quaternionW_1;
    volatile byte      quaternionW_2;
    volatile byte      quaternionW_3;
    volatile float      quaternionW;
    
    volatile byte      quaternionX_0;
    volatile byte      quaternionX_1;
    volatile byte      quaternionX_2;
    volatile byte      quaternionX_3;
    volatile float      quaternionX;
    
    volatile byte      quaternionY_0;
    volatile byte      quaternionY_1;
    volatile byte      quaternionY_2;
    volatile byte      quaternionY_3;
    volatile float      quaternionY;
    
    volatile byte      quaternionZ_0;
    volatile byte      quaternionZ_1;
    volatile byte      quaternionZ_2;
    volatile byte      quaternionZ_3;
    volatile float      quaternionZ;
    
    volatile byte		eulerX_0;
    volatile byte		eulerX_1;
    volatile byte		eulerX_2;
    volatile byte		eulerX_3;
    volatile float		eulerX;
    
    volatile byte		eulerY_0;
    volatile byte		eulerY_1;
    volatile byte		eulerY_2;
    volatile byte		eulerY_3;
    volatile float		eulerY;
    
    volatile byte		eulerZ_0;
    volatile byte		eulerZ_1;
    volatile byte		eulerZ_2;
    volatile byte		eulerZ_3;
    volatile float		eulerZ;
    
    
    /* Board ID */
    volatile byte       board_type;
    volatile byte       who_am_i;
    volatile byte       fw_ver_0;
    volatile float       fw_ver_1;
    
    volatile byte		timestamp;
    
    RegisterIO_I2C		registerIO;
    IOThread			ioThread;
    
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
    	ioThread = new IOThread();
    	ioThread.start();
    	
    }
    

	class IOThread implements Runnable {
        
        Thread              m_thread;
        boolean             stop;
        
        public void start() {
        	m_thread = new Thread(null, this, "LPMSDataThread");
        	stop = false;
            m_thread.start();
        }
        
        @Override
		public void run() {
        	while (!stop) {
        		acquire();
        		System.out.println("---------------------");
        	}

        }
   
        public void stop() {
        	stop = true;
        }
    }

    public void acquire() {
    	setRaw_gyro_x_0(registerIO.read(IMURegisters.LPMS_GYR_X_0));
    	setRaw_gyro_x_1(registerIO.read(IMURegisters.LPMS_GYR_X_1));
    	setRaw_gyro_x_2(registerIO.read(IMURegisters.LPMS_GYR_X_2));
    	setRaw_gyro_x_3(registerIO.read(IMURegisters.LPMS_GYR_X_3));
    	setRaw_gyro_x(IMUProtocol.decodebyte2float(getRaw_gyro_x_0(),getRaw_gyro_x_1(),getRaw_gyro_x_2(),getRaw_gyro_x_3()));
    	
    	setRaw_gyro_y_0(registerIO.read(IMURegisters.LPMS_GYR_Y_0));
    	setRaw_gyro_y_1(registerIO.read(IMURegisters.LPMS_GYR_Y_1));
    	setRaw_gyro_y_2(registerIO.read(IMURegisters.LPMS_GYR_Y_2));
    	setRaw_gyro_y_3(registerIO.read(IMURegisters.LPMS_GYR_Y_3));
    	
    	setRaw_gyro_z_0(registerIO.read(IMURegisters.LPMS_GYR_Z_0));
    	setRaw_gyro_z_1(registerIO.read(IMURegisters.LPMS_GYR_Z_1));
    	setRaw_gyro_z_2(registerIO.read(IMURegisters.LPMS_GYR_Z_2));
    	setRaw_gyro_z_3(registerIO.read(IMURegisters.LPMS_GYR_Z_3));
    }
    

    /**
	 * @return the raw_gyro_x_0
	 */
	public byte getRaw_gyro_x_0() {
		return raw_gyro_x_0;
	}

	/**
	 * @return the raw_gyro_x_1
	 */
	public byte getRaw_gyro_x_1() {
		return raw_gyro_x_1;
	}

	/**
	 * @return the raw_gyro_x_2
	 */
	public byte getRaw_gyro_x_2() {
		return raw_gyro_x_2;
	}

	/**
	 * @return the raw_gyro_x_3
	 */
	public byte getRaw_gyro_x_3() {
		return raw_gyro_x_3;
	}

	/**
	 * @return the raw_gyro_x
	 */
	public float getRaw_gyro_x() {
		return raw_gyro_x;
	}

	/**
	 * @return the raw_gyro_y_0
	 */
	public byte getRaw_gyro_y_0() {
		return raw_gyro_y_0;
	}

	/**
	 * @return the raw_gyro_y_1
	 */
	public byte getRaw_gyro_y_1() {
		return raw_gyro_y_1;
	}

	/**
	 * @return the raw_gyro_y_2
	 */
	public byte getRaw_gyro_y_2() {
		return raw_gyro_y_2;
	}

	/**
	 * @return the raw_gyro_y_3
	 */
	public byte getRaw_gyro_y_3() {
		return raw_gyro_y_3;
	}

	/**
	 * @return the raw_gyro_y
	 */
	public float getRaw_gyro_y() {
		return raw_gyro_y;
	}

	/**
	 * @return the raw_gyro_z_0
	 */
	public byte getRaw_gyro_z_0() {
		return raw_gyro_z_0;
	}

	/**
	 * @return the raw_gyro_z_1
	 */
	public byte getRaw_gyro_z_1() {
		return raw_gyro_z_1;
	}

	/**
	 * @return the raw_gyro_z_2
	 */
	public byte getRaw_gyro_z_2() {
		return raw_gyro_z_2;
	}

	/**
	 * @return the raw_gyro_z_3
	 */
	public byte getRaw_gyro_z_3() {
		return raw_gyro_z_3;
	}

	/**
	 * @return the raw_gyro_z
	 */
	public float getRaw_gyro_z() {
		return raw_gyro_z;
	}

	/**
	 * @return the raw_accel_x_0
	 */
	public byte getRaw_accel_x_0() {
		return raw_accel_x_0;
	}

	/**
	 * @return the raw_accel_x_1
	 */
	public byte getRaw_accel_x_1() {
		return raw_accel_x_1;
	}

	/**
	 * @return the raw_accel_x_2
	 */
	public byte getRaw_accel_x_2() {
		return raw_accel_x_2;
	}

	/**
	 * @return the raw_accel_x_3
	 */
	public byte getRaw_accel_x_3() {
		return raw_accel_x_3;
	}

	/**
	 * @return the raw_accel_x
	 */
	public float getRaw_accel_x() {
		return raw_accel_x;
	}

	/**
	 * @return the raw_accel_y_0
	 */
	public byte getRaw_accel_y_0() {
		return raw_accel_y_0;
	}

	/**
	 * @return the raw_accel_y_1
	 */
	public byte getRaw_accel_y_1() {
		return raw_accel_y_1;
	}

	/**
	 * @return the raw_accel_y_2
	 */
	public byte getRaw_accel_y_2() {
		return raw_accel_y_2;
	}

	/**
	 * @return the raw_accel_y_3
	 */
	public byte getRaw_accel_y_3() {
		return raw_accel_y_3;
	}

	/**
	 * @return the raw_accel_y
	 */
	public float getRaw_accel_y() {
		return raw_accel_y;
	}

	/**
	 * @return the raw_accel_z_0
	 */
	public byte getRaw_accel_z_0() {
		return raw_accel_z_0;
	}

	/**
	 * @return the raw_accel_z_1
	 */
	public byte getRaw_accel_z_1() {
		return raw_accel_z_1;
	}

	/**
	 * @return the raw_accel_z_2
	 */
	public byte getRaw_accel_z_2() {
		return raw_accel_z_2;
	}

	/**
	 * @return the raw_accel_z_3
	 */
	public byte getRaw_accel_z_3() {
		return raw_accel_z_3;
	}

	/**
	 * @return the raw_accel_z
	 */
	public float getRaw_accel_z() {
		return raw_accel_z;
	}

	/**
	 * @return the cal_mag_x_0
	 */
	public byte getCal_mag_x_0() {
		return cal_mag_x_0;
	}

	/**
	 * @return the cal_mag_x_1
	 */
	public byte getCal_mag_x_1() {
		return cal_mag_x_1;
	}

	/**
	 * @return the cal_mag_x_2
	 */
	public byte getCal_mag_x_2() {
		return cal_mag_x_2;
	}

	/**
	 * @return the cal_mag_x_3
	 */
	public byte getCal_mag_x_3() {
		return cal_mag_x_3;
	}

	/**
	 * @return the cal_mag_x
	 */
	public float getCal_mag_x() {
		return cal_mag_x;
	}

	/**
	 * @return the cal_mag_y_0
	 */
	public byte getCal_mag_y_0() {
		return cal_mag_y_0;
	}

	/**
	 * @return the cal_mag_y_1
	 */
	public byte getCal_mag_y_1() {
		return cal_mag_y_1;
	}

	/**
	 * @return the cal_mag_y_2
	 */
	public byte getCal_mag_y_2() {
		return cal_mag_y_2;
	}

	/**
	 * @return the cal_mag_y_3
	 */
	public byte getCal_mag_y_3() {
		return cal_mag_y_3;
	}

	/**
	 * @return the cal_mag_y
	 */
	public float getCal_mag_y() {
		return cal_mag_y;
	}

	/**
	 * @return the cal_mag_z_0
	 */
	public byte getCal_mag_z_0() {
		return cal_mag_z_0;
	}

	/**
	 * @return the cal_mag_z_1
	 */
	public byte getCal_mag_z_1() {
		return cal_mag_z_1;
	}

	/**
	 * @return the cal_mag_z_2
	 */
	public byte getCal_mag_z_2() {
		return cal_mag_z_2;
	}

	/**
	 * @return the cal_mag_z_3
	 */
	public byte getCal_mag_z_3() {
		return cal_mag_z_3;
	}

	/**
	 * @return the cal_mag_z
	 */
	public float getCal_mag_z() {
		return cal_mag_z;
	}

	/**
	 * @return the world_linear_accel_x_0
	 */
	public byte getWorld_linear_accel_x_0() {
		return world_linear_accel_x_0;
	}

	/**
	 * @return the world_linear_accel_x_1
	 */
	public byte getWorld_linear_accel_x_1() {
		return world_linear_accel_x_1;
	}

	/**
	 * @return the world_linear_accel_x_2
	 */
	public byte getWorld_linear_accel_x_2() {
		return world_linear_accel_x_2;
	}

	/**
	 * @return the world_linear_accel_x_3
	 */
	public byte getWorld_linear_accel_x_3() {
		return world_linear_accel_x_3;
	}

	/**
	 * @return the world_linear_accel_x
	 */
	public float getWorld_linear_accel_x() {
		return world_linear_accel_x;
	}

	/**
	 * @return the world_linear_accel_y_0
	 */
	public byte getWorld_linear_accel_y_0() {
		return world_linear_accel_y_0;
	}

	/**
	 * @return the world_linear_accel_y_1
	 */
	public byte getWorld_linear_accel_y_1() {
		return world_linear_accel_y_1;
	}

	/**
	 * @return the world_linear_accel_y_2
	 */
	public byte getWorld_linear_accel_y_2() {
		return world_linear_accel_y_2;
	}

	/**
	 * @return the world_linear_accel_y_3
	 */
	public byte getWorld_linear_accel_y_3() {
		return world_linear_accel_y_3;
	}

	/**
	 * @return the world_linear_accel_y
	 */
	public float getWorld_linear_accel_y() {
		return world_linear_accel_y;
	}

	/**
	 * @return the world_linear_accel_z_0
	 */
	public byte getWorld_linear_accel_z_0() {
		return world_linear_accel_z_0;
	}

	/**
	 * @return the world_linear_accel_z_1
	 */
	public byte getWorld_linear_accel_z_1() {
		return world_linear_accel_z_1;
	}

	/**
	 * @return the world_linear_accel_z_2
	 */
	public byte getWorld_linear_accel_z_2() {
		return world_linear_accel_z_2;
	}

	/**
	 * @return the world_linear_accel_z_3
	 */
	public byte getWorld_linear_accel_z_3() {
		return world_linear_accel_z_3;
	}

	/**
	 * @return the world_linear_accel_z
	 */
	public float getWorld_linear_accel_z() {
		return world_linear_accel_z;
	}

	/**
	 * @return the mpu_temp_c_0
	 */
	public byte getMpu_temp_c_0() {
		return mpu_temp_c_0;
	}

	/**
	 * @return the mpu_temp_c_1
	 */
	public byte getMpu_temp_c_1() {
		return mpu_temp_c_1;
	}

	/**
	 * @return the mpu_temp_c_2
	 */
	public byte getMpu_temp_c_2() {
		return mpu_temp_c_2;
	}

	/**
	 * @return the mpu_temp_c_3
	 */
	public byte getMpu_temp_c_3() {
		return mpu_temp_c_3;
	}

	/**
	 * @return the mpu_temp_c
	 */
	public float getMpu_temp_c() {
		return mpu_temp_c;
	}

	/**
	 * @return the quaternionW_0
	 */
	public byte getQuaternionW_0() {
		return quaternionW_0;
	}

	/**
	 * @return the quaternionW_1
	 */
	public byte getQuaternionW_1() {
		return quaternionW_1;
	}

	/**
	 * @return the quaternionW_2
	 */
	public byte getQuaternionW_2() {
		return quaternionW_2;
	}

	/**
	 * @return the quaternionW_3
	 */
	public byte getQuaternionW_3() {
		return quaternionW_3;
	}

	/**
	 * @return the quaternionW
	 */
	public float getQuaternionW() {
		return quaternionW;
	}

	/**
	 * @return the quaternionX_0
	 */
	public byte getQuaternionX_0() {
		return quaternionX_0;
	}

	/**
	 * @return the quaternionX_1
	 */
	public byte getQuaternionX_1() {
		return quaternionX_1;
	}

	/**
	 * @return the quaternionX_2
	 */
	public byte getQuaternionX_2() {
		return quaternionX_2;
	}

	/**
	 * @return the quaternionX_3
	 */
	public byte getQuaternionX_3() {
		return quaternionX_3;
	}

	/**
	 * @return the quaternionX
	 */
	public float getQuaternionX() {
		return quaternionX;
	}

	/**
	 * @return the quaternionY_0
	 */
	public byte getQuaternionY_0() {
		return quaternionY_0;
	}

	/**
	 * @return the quaternionY_1
	 */
	public byte getQuaternionY_1() {
		return quaternionY_1;
	}

	/**
	 * @return the quaternionY_2
	 */
	public byte getQuaternionY_2() {
		return quaternionY_2;
	}

	/**
	 * @return the quaternionY_3
	 */
	public byte getQuaternionY_3() {
		return quaternionY_3;
	}

	/**
	 * @return the quaternionY
	 */
	public float getQuaternionY() {
		return quaternionY;
	}

	/**
	 * @return the quaternionZ_0
	 */
	public byte getQuaternionZ_0() {
		return quaternionZ_0;
	}

	/**
	 * @return the quaternionZ_1
	 */
	public byte getQuaternionZ_1() {
		return quaternionZ_1;
	}

	/**
	 * @return the quaternionZ_2
	 */
	public byte getQuaternionZ_2() {
		return quaternionZ_2;
	}

	/**
	 * @return the quaternionZ_3
	 */
	public byte getQuaternionZ_3() {
		return quaternionZ_3;
	}

	/**
	 * @return the quaternionZ
	 */
	public float getQuaternionZ() {
		return quaternionZ;
	}

	/**
	 * @return the eulerX_0
	 */
	public byte getEulerX_0() {
		return eulerX_0;
	}

	/**
	 * @return the eulerX_1
	 */
	public byte getEulerX_1() {
		return eulerX_1;
	}

	/**
	 * @return the eulerX_2
	 */
	public byte getEulerX_2() {
		return eulerX_2;
	}

	/**
	 * @return the eulerX_3
	 */
	public byte getEulerX_3() {
		return eulerX_3;
	}

	/**
	 * @return the eulerX
	 */
	public float getEulerX() {
		return eulerX;
	}

	/**
	 * @return the eulerY_0
	 */
	public byte getEulerY_0() {
		return eulerY_0;
	}

	/**
	 * @return the eulerY_1
	 */
	public byte getEulerY_1() {
		return eulerY_1;
	}

	/**
	 * @return the eulerY_2
	 */
	public byte getEulerY_2() {
		return eulerY_2;
	}

	/**
	 * @return the eulerY_3
	 */
	public byte getEulerY_3() {
		return eulerY_3;
	}

	/**
	 * @return the eulerY
	 */
	public float getEulerY() {
		return eulerY;
	}

	/**
	 * @return the eulerZ_0
	 */
	public byte getEulerZ_0() {
		return eulerZ_0;
	}

	/**
	 * @return the eulerZ_1
	 */
	public byte getEulerZ_1() {
		return eulerZ_1;
	}

	/**
	 * @return the eulerZ_2
	 */
	public byte getEulerZ_2() {
		return eulerZ_2;
	}

	/**
	 * @return the eulerZ_3
	 */
	public byte getEulerZ_3() {
		return eulerZ_3;
	}

	/**
	 * @return the eulerZ
	 */
	public float getEulerZ() {
		return eulerZ;
	}

	/**
	 * @return the board_type
	 */
	public byte getBoard_type() {
		return board_type;
	}

	/**
	 * @return the who_am_i
	 */
	public byte getWho_am_i() {
		return who_am_i;
	}

	/**
	 * @return the fw_ver_0
	 */
	public byte getFw_ver_0() {
		return fw_ver_0;
	}

	/**
	 * @return the fw_ver_1
	 */
	public float getFw_ver_1() {
		return fw_ver_1;
	}

	/**
	 * @return the timestamp
	 */
	public byte getTimestamp() {
		return timestamp;
	}

	/**
	 * @return the registerIO
	 */
	public RegisterIO_I2C getRegisterIO() {
		return registerIO;
	}

	/**
	 * @return the ioThread
	 */
	public IOThread getIoThread() {
		return ioThread;
	}

	/**
	 * @param raw_gyro_x_0 the raw_gyro_x_0 to set
	 */
	public void setRaw_gyro_x_0(byte raw_gyro_x_0) {
		this.raw_gyro_x_0 = raw_gyro_x_0;
	}

	/**
	 * @param raw_gyro_x_1 the raw_gyro_x_1 to set
	 */
	public void setRaw_gyro_x_1(byte raw_gyro_x_1) {
		this.raw_gyro_x_1 = raw_gyro_x_1;
	}

	/**
	 * @param raw_gyro_x_2 the raw_gyro_x_2 to set
	 */
	public void setRaw_gyro_x_2(byte raw_gyro_x_2) {
		this.raw_gyro_x_2 = raw_gyro_x_2;
	}

	/**
	 * @param raw_gyro_x_3 the raw_gyro_x_3 to set
	 */
	public void setRaw_gyro_x_3(byte raw_gyro_x_3) {
		this.raw_gyro_x_3 = raw_gyro_x_3;
	}

	/**
	 * @param raw_gyro_x the raw_gyro_x to set
	 */
	public void setRaw_gyro_x(float raw_gyro_x) {
		this.raw_gyro_x = raw_gyro_x;
	}

	/**
	 * @param raw_gyro_y_0 the raw_gyro_y_0 to set
	 */
	public void setRaw_gyro_y_0(byte raw_gyro_y_0) {
		this.raw_gyro_y_0 = raw_gyro_y_0;
	}

	/**
	 * @param raw_gyro_y_1 the raw_gyro_y_1 to set
	 */
	public void setRaw_gyro_y_1(byte raw_gyro_y_1) {
		this.raw_gyro_y_1 = raw_gyro_y_1;
	}

	/**
	 * @param raw_gyro_y_2 the raw_gyro_y_2 to set
	 */
	public void setRaw_gyro_y_2(byte raw_gyro_y_2) {
		this.raw_gyro_y_2 = raw_gyro_y_2;
	}

	/**
	 * @param raw_gyro_y_3 the raw_gyro_y_3 to set
	 */
	public void setRaw_gyro_y_3(byte raw_gyro_y_3) {
		this.raw_gyro_y_3 = raw_gyro_y_3;
	}

	/**
	 * @param raw_gyro_y the raw_gyro_y to set
	 */
	public void setRaw_gyro_y(byte raw_gyro_y) {
		this.raw_gyro_y = raw_gyro_y;
	}

	/**
	 * @param raw_gyro_z_0 the raw_gyro_z_0 to set
	 */
	public void setRaw_gyro_z_0(byte raw_gyro_z_0) {
		this.raw_gyro_z_0 = raw_gyro_z_0;
	}

	/**
	 * @param raw_gyro_z_1 the raw_gyro_z_1 to set
	 */
	public void setRaw_gyro_z_1(byte raw_gyro_z_1) {
		this.raw_gyro_z_1 = raw_gyro_z_1;
	}

	/**
	 * @param raw_gyro_z_2 the raw_gyro_z_2 to set
	 */
	public void setRaw_gyro_z_2(byte raw_gyro_z_2) {
		this.raw_gyro_z_2 = raw_gyro_z_2;
	}

	/**
	 * @param raw_gyro_z_3 the raw_gyro_z_3 to set
	 */
	public void setRaw_gyro_z_3(byte raw_gyro_z_3) {
		this.raw_gyro_z_3 = raw_gyro_z_3;
	}

	/**
	 * @param raw_gyro_z the raw_gyro_z to set
	 */
	public void setRaw_gyro_z(byte raw_gyro_z) {
		this.raw_gyro_z = raw_gyro_z;
	}

	/**
	 * @param raw_accel_x_0 the raw_accel_x_0 to set
	 */
	public void setRaw_accel_x_0(byte raw_accel_x_0) {
		this.raw_accel_x_0 = raw_accel_x_0;
	}

	/**
	 * @param raw_accel_x_1 the raw_accel_x_1 to set
	 */
	public void setRaw_accel_x_1(byte raw_accel_x_1) {
		this.raw_accel_x_1 = raw_accel_x_1;
	}

	/**
	 * @param raw_accel_x_2 the raw_accel_x_2 to set
	 */
	public void setRaw_accel_x_2(byte raw_accel_x_2) {
		this.raw_accel_x_2 = raw_accel_x_2;
	}

	/**
	 * @param raw_accel_x_3 the raw_accel_x_3 to set
	 */
	public void setRaw_accel_x_3(byte raw_accel_x_3) {
		this.raw_accel_x_3 = raw_accel_x_3;
	}

	/**
	 * @param raw_accel_x the raw_accel_x to set
	 */
	public void setRaw_accel_x(byte raw_accel_x) {
		this.raw_accel_x = raw_accel_x;
	}

	/**
	 * @param raw_accel_y_0 the raw_accel_y_0 to set
	 */
	public void setRaw_accel_y_0(byte raw_accel_y_0) {
		this.raw_accel_y_0 = raw_accel_y_0;
	}

	/**
	 * @param raw_accel_y_1 the raw_accel_y_1 to set
	 */
	public void setRaw_accel_y_1(byte raw_accel_y_1) {
		this.raw_accel_y_1 = raw_accel_y_1;
	}

	/**
	 * @param raw_accel_y_2 the raw_accel_y_2 to set
	 */
	public void setRaw_accel_y_2(byte raw_accel_y_2) {
		this.raw_accel_y_2 = raw_accel_y_2;
	}

	/**
	 * @param raw_accel_y_3 the raw_accel_y_3 to set
	 */
	public void setRaw_accel_y_3(byte raw_accel_y_3) {
		this.raw_accel_y_3 = raw_accel_y_3;
	}

	/**
	 * @param raw_accel_y the raw_accel_y to set
	 */
	public void setRaw_accel_y(byte raw_accel_y) {
		this.raw_accel_y = raw_accel_y;
	}

	/**
	 * @param raw_accel_z_0 the raw_accel_z_0 to set
	 */
	public void setRaw_accel_z_0(byte raw_accel_z_0) {
		this.raw_accel_z_0 = raw_accel_z_0;
	}

	/**
	 * @param raw_accel_z_1 the raw_accel_z_1 to set
	 */
	public void setRaw_accel_z_1(byte raw_accel_z_1) {
		this.raw_accel_z_1 = raw_accel_z_1;
	}

	/**
	 * @param raw_accel_z_2 the raw_accel_z_2 to set
	 */
	public void setRaw_accel_z_2(byte raw_accel_z_2) {
		this.raw_accel_z_2 = raw_accel_z_2;
	}

	/**
	 * @param raw_accel_z_3 the raw_accel_z_3 to set
	 */
	public void setRaw_accel_z_3(byte raw_accel_z_3) {
		this.raw_accel_z_3 = raw_accel_z_3;
	}

	/**
	 * @param raw_accel_z the raw_accel_z to set
	 */
	public void setRaw_accel_z(byte raw_accel_z) {
		this.raw_accel_z = raw_accel_z;
	}

	/**
	 * @param cal_mag_x_0 the cal_mag_x_0 to set
	 */
	public void setCal_mag_x_0(byte cal_mag_x_0) {
		this.cal_mag_x_0 = cal_mag_x_0;
	}

	/**
	 * @param cal_mag_x_1 the cal_mag_x_1 to set
	 */
	public void setCal_mag_x_1(byte cal_mag_x_1) {
		this.cal_mag_x_1 = cal_mag_x_1;
	}

	/**
	 * @param cal_mag_x_2 the cal_mag_x_2 to set
	 */
	public void setCal_mag_x_2(byte cal_mag_x_2) {
		this.cal_mag_x_2 = cal_mag_x_2;
	}

	/**
	 * @param cal_mag_x_3 the cal_mag_x_3 to set
	 */
	public void setCal_mag_x_3(byte cal_mag_x_3) {
		this.cal_mag_x_3 = cal_mag_x_3;
	}

	/**
	 * @param cal_mag_x the cal_mag_x to set
	 */
	public void setCal_mag_x(byte cal_mag_x) {
		this.cal_mag_x = cal_mag_x;
	}

	/**
	 * @param cal_mag_y_0 the cal_mag_y_0 to set
	 */
	public void setCal_mag_y_0(byte cal_mag_y_0) {
		this.cal_mag_y_0 = cal_mag_y_0;
	}

	/**
	 * @param cal_mag_y_1 the cal_mag_y_1 to set
	 */
	public void setCal_mag_y_1(byte cal_mag_y_1) {
		this.cal_mag_y_1 = cal_mag_y_1;
	}

	/**
	 * @param cal_mag_y_2 the cal_mag_y_2 to set
	 */
	public void setCal_mag_y_2(byte cal_mag_y_2) {
		this.cal_mag_y_2 = cal_mag_y_2;
	}

	/**
	 * @param cal_mag_y_3 the cal_mag_y_3 to set
	 */
	public void setCal_mag_y_3(byte cal_mag_y_3) {
		this.cal_mag_y_3 = cal_mag_y_3;
	}

	/**
	 * @param cal_mag_y the cal_mag_y to set
	 */
	public void setCal_mag_y(byte cal_mag_y) {
		this.cal_mag_y = cal_mag_y;
	}

	/**
	 * @param cal_mag_z_0 the cal_mag_z_0 to set
	 */
	public void setCal_mag_z_0(byte cal_mag_z_0) {
		this.cal_mag_z_0 = cal_mag_z_0;
	}

	/**
	 * @param cal_mag_z_1 the cal_mag_z_1 to set
	 */
	public void setCal_mag_z_1(byte cal_mag_z_1) {
		this.cal_mag_z_1 = cal_mag_z_1;
	}

	/**
	 * @param cal_mag_z_2 the cal_mag_z_2 to set
	 */
	public void setCal_mag_z_2(byte cal_mag_z_2) {
		this.cal_mag_z_2 = cal_mag_z_2;
	}

	/**
	 * @param cal_mag_z_3 the cal_mag_z_3 to set
	 */
	public void setCal_mag_z_3(byte cal_mag_z_3) {
		this.cal_mag_z_3 = cal_mag_z_3;
	}

	/**
	 * @param cal_mag_z the cal_mag_z to set
	 */
	public void setCal_mag_z(byte cal_mag_z) {
		this.cal_mag_z = cal_mag_z;
	}

	/**
	 * @param world_linear_accel_x_0 the world_linear_accel_x_0 to set
	 */
	public void setWorld_linear_accel_x_0(byte world_linear_accel_x_0) {
		this.world_linear_accel_x_0 = world_linear_accel_x_0;
	}

	/**
	 * @param world_linear_accel_x_1 the world_linear_accel_x_1 to set
	 */
	public void setWorld_linear_accel_x_1(byte world_linear_accel_x_1) {
		this.world_linear_accel_x_1 = world_linear_accel_x_1;
	}

	/**
	 * @param world_linear_accel_x_2 the world_linear_accel_x_2 to set
	 */
	public void setWorld_linear_accel_x_2(byte world_linear_accel_x_2) {
		this.world_linear_accel_x_2 = world_linear_accel_x_2;
	}

	/**
	 * @param world_linear_accel_x_3 the world_linear_accel_x_3 to set
	 */
	public void setWorld_linear_accel_x_3(byte world_linear_accel_x_3) {
		this.world_linear_accel_x_3 = world_linear_accel_x_3;
	}

	/**
	 * @param world_linear_accel_x the world_linear_accel_x to set
	 */
	public void setWorld_linear_accel_x(byte world_linear_accel_x) {
		this.world_linear_accel_x = world_linear_accel_x;
	}

	/**
	 * @param world_linear_accel_y_0 the world_linear_accel_y_0 to set
	 */
	public void setWorld_linear_accel_y_0(byte world_linear_accel_y_0) {
		this.world_linear_accel_y_0 = world_linear_accel_y_0;
	}

	/**
	 * @param world_linear_accel_y_1 the world_linear_accel_y_1 to set
	 */
	public void setWorld_linear_accel_y_1(byte world_linear_accel_y_1) {
		this.world_linear_accel_y_1 = world_linear_accel_y_1;
	}

	/**
	 * @param world_linear_accel_y_2 the world_linear_accel_y_2 to set
	 */
	public void setWorld_linear_accel_y_2(byte world_linear_accel_y_2) {
		this.world_linear_accel_y_2 = world_linear_accel_y_2;
	}

	/**
	 * @param world_linear_accel_y_3 the world_linear_accel_y_3 to set
	 */
	public void setWorld_linear_accel_y_3(byte world_linear_accel_y_3) {
		this.world_linear_accel_y_3 = world_linear_accel_y_3;
	}

	/**
	 * @param world_linear_accel_y the world_linear_accel_y to set
	 */
	public void setWorld_linear_accel_y(byte world_linear_accel_y) {
		this.world_linear_accel_y = world_linear_accel_y;
	}

	/**
	 * @param world_linear_accel_z_0 the world_linear_accel_z_0 to set
	 */
	public void setWorld_linear_accel_z_0(byte world_linear_accel_z_0) {
		this.world_linear_accel_z_0 = world_linear_accel_z_0;
	}

	/**
	 * @param world_linear_accel_z_1 the world_linear_accel_z_1 to set
	 */
	public void setWorld_linear_accel_z_1(byte world_linear_accel_z_1) {
		this.world_linear_accel_z_1 = world_linear_accel_z_1;
	}

	/**
	 * @param world_linear_accel_z_2 the world_linear_accel_z_2 to set
	 */
	public void setWorld_linear_accel_z_2(byte world_linear_accel_z_2) {
		this.world_linear_accel_z_2 = world_linear_accel_z_2;
	}

	/**
	 * @param world_linear_accel_z_3 the world_linear_accel_z_3 to set
	 */
	public void setWorld_linear_accel_z_3(byte world_linear_accel_z_3) {
		this.world_linear_accel_z_3 = world_linear_accel_z_3;
	}

	/**
	 * @param world_linear_accel_z the world_linear_accel_z to set
	 */
	public void setWorld_linear_accel_z(byte world_linear_accel_z) {
		this.world_linear_accel_z = world_linear_accel_z;
	}

	/**
	 * @param mpu_temp_c_0 the mpu_temp_c_0 to set
	 */
	public void setMpu_temp_c_0(byte mpu_temp_c_0) {
		this.mpu_temp_c_0 = mpu_temp_c_0;
	}

	/**
	 * @param mpu_temp_c_1 the mpu_temp_c_1 to set
	 */
	public void setMpu_temp_c_1(byte mpu_temp_c_1) {
		this.mpu_temp_c_1 = mpu_temp_c_1;
	}

	/**
	 * @param mpu_temp_c_2 the mpu_temp_c_2 to set
	 */
	public void setMpu_temp_c_2(byte mpu_temp_c_2) {
		this.mpu_temp_c_2 = mpu_temp_c_2;
	}

	/**
	 * @param mpu_temp_c_3 the mpu_temp_c_3 to set
	 */
	public void setMpu_temp_c_3(byte mpu_temp_c_3) {
		this.mpu_temp_c_3 = mpu_temp_c_3;
	}

	/**
	 * @param mpu_temp_c the mpu_temp_c to set
	 */
	public void setMpu_temp_c(byte mpu_temp_c) {
		this.mpu_temp_c = mpu_temp_c;
	}

	/**
	 * @param quaternionW_0 the quaternionW_0 to set
	 */
	public void setQuaternionW_0(byte quaternionW_0) {
		this.quaternionW_0 = quaternionW_0;
	}

	/**
	 * @param quaternionW_1 the quaternionW_1 to set
	 */
	public void setQuaternionW_1(byte quaternionW_1) {
		this.quaternionW_1 = quaternionW_1;
	}

	/**
	 * @param quaternionW_2 the quaternionW_2 to set
	 */
	public void setQuaternionW_2(byte quaternionW_2) {
		this.quaternionW_2 = quaternionW_2;
	}

	/**
	 * @param quaternionW_3 the quaternionW_3 to set
	 */
	public void setQuaternionW_3(byte quaternionW_3) {
		this.quaternionW_3 = quaternionW_3;
	}

	/**
	 * @param quaternionW the quaternionW to set
	 */
	public void setQuaternionW(byte quaternionW) {
		this.quaternionW = quaternionW;
	}

	/**
	 * @param quaternionX_0 the quaternionX_0 to set
	 */
	public void setQuaternionX_0(byte quaternionX_0) {
		this.quaternionX_0 = quaternionX_0;
	}

	/**
	 * @param quaternionX_1 the quaternionX_1 to set
	 */
	public void setQuaternionX_1(byte quaternionX_1) {
		this.quaternionX_1 = quaternionX_1;
	}

	/**
	 * @param quaternionX_2 the quaternionX_2 to set
	 */
	public void setQuaternionX_2(byte quaternionX_2) {
		this.quaternionX_2 = quaternionX_2;
	}

	/**
	 * @param quaternionX_3 the quaternionX_3 to set
	 */
	public void setQuaternionX_3(byte quaternionX_3) {
		this.quaternionX_3 = quaternionX_3;
	}

	/**
	 * @param quaternionX the quaternionX to set
	 */
	public void setQuaternionX(byte quaternionX) {
		this.quaternionX = quaternionX;
	}

	/**
	 * @param quaternionY_0 the quaternionY_0 to set
	 */
	public void setQuaternionY_0(byte quaternionY_0) {
		this.quaternionY_0 = quaternionY_0;
	}

	/**
	 * @param quaternionY_1 the quaternionY_1 to set
	 */
	public void setQuaternionY_1(byte quaternionY_1) {
		this.quaternionY_1 = quaternionY_1;
	}

	/**
	 * @param quaternionY_2 the quaternionY_2 to set
	 */
	public void setQuaternionY_2(byte quaternionY_2) {
		this.quaternionY_2 = quaternionY_2;
	}

	/**
	 * @param quaternionY_3 the quaternionY_3 to set
	 */
	public void setQuaternionY_3(byte quaternionY_3) {
		this.quaternionY_3 = quaternionY_3;
	}

	/**
	 * @param quaternionY the quaternionY to set
	 */
	public void setQuaternionY(byte quaternionY) {
		this.quaternionY = quaternionY;
	}

	/**
	 * @param quaternionZ_0 the quaternionZ_0 to set
	 */
	public void setQuaternionZ_0(byte quaternionZ_0) {
		this.quaternionZ_0 = quaternionZ_0;
	}

	/**
	 * @param quaternionZ_1 the quaternionZ_1 to set
	 */
	public void setQuaternionZ_1(byte quaternionZ_1) {
		this.quaternionZ_1 = quaternionZ_1;
	}

	/**
	 * @param quaternionZ_2 the quaternionZ_2 to set
	 */
	public void setQuaternionZ_2(byte quaternionZ_2) {
		this.quaternionZ_2 = quaternionZ_2;
	}

	/**
	 * @param quaternionZ_3 the quaternionZ_3 to set
	 */
	public void setQuaternionZ_3(byte quaternionZ_3) {
		this.quaternionZ_3 = quaternionZ_3;
	}

	/**
	 * @param quaternionZ the quaternionZ to set
	 */
	public void setQuaternionZ(byte quaternionZ) {
		this.quaternionZ = quaternionZ;
	}

	/**
	 * @param eulerX_0 the eulerX_0 to set
	 */
	public void setEulerX_0(byte eulerX_0) {
		this.eulerX_0 = eulerX_0;
	}

	/**
	 * @param eulerX_1 the eulerX_1 to set
	 */
	public void setEulerX_1(byte eulerX_1) {
		this.eulerX_1 = eulerX_1;
	}

	/**
	 * @param eulerX_2 the eulerX_2 to set
	 */
	public void setEulerX_2(byte eulerX_2) {
		this.eulerX_2 = eulerX_2;
	}

	/**
	 * @param eulerX_3 the eulerX_3 to set
	 */
	public void setEulerX_3(byte eulerX_3) {
		this.eulerX_3 = eulerX_3;
	}

	/**
	 * @param eulerX the eulerX to set
	 */
	public void setEulerX(byte eulerX) {
		this.eulerX = eulerX;
	}

	/**
	 * @param eulerY_0 the eulerY_0 to set
	 */
	public void setEulerY_0(byte eulerY_0) {
		this.eulerY_0 = eulerY_0;
	}

	/**
	 * @param eulerY_1 the eulerY_1 to set
	 */
	public void setEulerY_1(byte eulerY_1) {
		this.eulerY_1 = eulerY_1;
	}

	/**
	 * @param eulerY_2 the eulerY_2 to set
	 */
	public void setEulerY_2(byte eulerY_2) {
		this.eulerY_2 = eulerY_2;
	}

	/**
	 * @param eulerY_3 the eulerY_3 to set
	 */
	public void setEulerY_3(byte eulerY_3) {
		this.eulerY_3 = eulerY_3;
	}

	/**
	 * @param eulerY the eulerY to set
	 */
	public void setEulerY(byte eulerY) {
		this.eulerY = eulerY;
	}

	/**
	 * @param eulerZ_0 the eulerZ_0 to set
	 */
	public void setEulerZ_0(byte eulerZ_0) {
		this.eulerZ_0 = eulerZ_0;
	}

	/**
	 * @param eulerZ_1 the eulerZ_1 to set
	 */
	public void setEulerZ_1(byte eulerZ_1) {
		this.eulerZ_1 = eulerZ_1;
	}

	/**
	 * @param eulerZ_2 the eulerZ_2 to set
	 */
	public void setEulerZ_2(byte eulerZ_2) {
		this.eulerZ_2 = eulerZ_2;
	}

	/**
	 * @param eulerZ_3 the eulerZ_3 to set
	 */
	public void setEulerZ_3(byte eulerZ_3) {
		this.eulerZ_3 = eulerZ_3;
	}

	/**
	 * @param eulerZ the eulerZ to set
	 */
	public void setEulerZ(byte eulerZ) {
		this.eulerZ = eulerZ;
	}

	/**
	 * @param board_type the board_type to set
	 */
	public void setBoard_type(byte board_type) {
		this.board_type = board_type;
	}

	/**
	 * @param who_am_i the who_am_i to set
	 */
	public void setWho_am_i(byte who_am_i) {
		this.who_am_i = who_am_i;
	}

	/**
	 * @param fw_ver_0 the fw_ver_0 to set
	 */
	public void setFw_ver_0(byte fw_ver_0) {
		this.fw_ver_0 = fw_ver_0;
	}

	/**
	 * @param fw_ver_1 the fw_ver_1 to set
	 */
	public void setFw_ver_1(byte fw_ver_1) {
		this.fw_ver_1 = fw_ver_1;
	}

	/**
	 * @param timestamp the timestamp to set
	 */
	public void setTimestamp(byte timestamp) {
		this.timestamp = timestamp;
	}

	/**
	 * @param registerIO the registerIO to set
	 */
	public void setRegisterIO(RegisterIO_I2C registerIO) {
		this.registerIO = registerIO;
	}

	/**
	 * @param ioThread the ioThread to set
	 */
	public void setIoThread(IOThread ioThread) {
		this.ioThread = ioThread;
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
