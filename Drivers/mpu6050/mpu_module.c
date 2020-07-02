/* Includes ------------------------------------------------------------------*/
//#include "inv_mpu20608.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "mpu_module.h"
#include "i2c.h"

//#define MPU_DEBUG        //PC�˵���

/* Private define ------------------------------------------------------------*/
#ifdef MPU_DEBUG

/* Data read from MPL. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)
#endif

/* Switch */
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

/* Starting sampling rate. */
#define DEFAULT_mpu_HZ  (100)           // ����DMP�������(��󲻳���200Hz)
#define TEMP_READ_TICK    (500)


/* Private typedef -----------------------------------------------------------*/
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char lp_6axis_mode;
    unsigned char sensors;
    unsigned char watermark;
    //volatile unsigned char new_sensor;
    unsigned char motion_int_mode;
    unsigned long next_temp_tick;
    unsigned int report;
	  unsigned char dmp_on;
		volatile unsigned char new_gyro;
};


/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s {
    signed char orientation[9];
};


/* Private variables ---------------------------------------------------------*/
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the driver(s).
 */
static struct platform_data_s gyro_pdata = {
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

//static struct hal_s hal = {0};
 static struct hal_s hal = {0};


unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
#ifdef MPU_DEBUG
/** 
 *  @brief ��MPL���ж����������
 *  @return ��
 *  @attention ͨ��UART����
 */
static void read_from_mpl(void)
{
    long data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};

    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
       /* Sends a quaternion packet to the PC. Since this is used by the Python
        * test app to visually represent a 3D quaternion, it's sent each time
        * the MPL has new data.
        */
        eMPL_send_quat(data);

        /* Specific data packets can be sent or suppressed using USB commands. */
        if (hal.report & PRINT_QUAT)
            eMPL_send_data(PACKET_DATA_QUAT, data);
    }
    if (hal.report & PRINT_ACCEL) {
        if (inv_get_sensor_type_accel(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ACCEL, data);
    }
    if (hal.report & PRINT_GYRO) {
        if (inv_get_sensor_type_gyro(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_GYRO, data);
    }
    if (hal.report & PRINT_EULER) {
        if (inv_get_sensor_type_euler(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_EULER, data);
    }
    if (hal.report & PRINT_ROT_MAT) {
        if (inv_get_sensor_type_rot_mat(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_ROT, data);
    }
    if (hal.report & PRINT_HEADING) {
        if (inv_get_sensor_type_heading(data, &accuracy,
            (inv_time_t*)&timestamp))
            eMPL_send_data(PACKET_DATA_HEADING, data);
    }
    if (hal.report & PRINT_LINEAR_ACCEL) {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy,
            (inv_time_t*)&timestamp))
        	MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
        			float_data[0], float_data[1], float_data[2]);
    }
    if (hal.report & PRINT_GRAVITY_VECTOR) {
            if (inv_get_sensor_type_gravity(float_data, &accuracy,
                (inv_time_t*)&timestamp))
            	MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
            			float_data[0], float_data[1], float_data[2]);
    }
}
#endif

/** 
 *  @brief�жϵ��ú�����ÿ��MPU6050���������ʱ������INT����Ӧ�ĵ�ƽ���
 *  @return 1 if data was updated. 
 *  @attention 
 */

void gyro_data_ready_cb(void)
 {
		hal.new_gyro = 1;
	 
//	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_SET);									//LED7....LED8
  }

	
/* USER CODE BEGIN 4 */


/* USER CODE END 4 */


/** 
 *  @brief��װinv_get_sensor_type_euler()����,����ŷ���� q16��ʽ
 *	Pitch: -180 to 180
 *  Roll: -90 to 90
 *  Yaw: -180 to 180
 *  @return 1 if data was updated. 
 *  @attention 
 */
int8_t mpu_read_euler(long *data, unsigned long *timestamp) {
  int8_t tmp,sum;
  sum = inv_get_sensor_type_euler(data, &tmp,(inv_time_t*)&timestamp);
  return sum;
}

/** 
 *  @brief MPUģ���ʼ������
 *  @return ��
 *  @attention 
 */
int module_mpu_init(void)
{ 

  inv_error_t result;
  unsigned char accel_fsr;
  unsigned short gyro_rate, gyro_fsr;
	struct int_param_s int_param;
   
  result = mpu_init(&int_param);				
  if (result) {
		
      printf("Could not initialize sensors.\n");
  }else{
	printf("initialize sensors  is succeed.\n");
	}

  result = inv_init_mpl();
  if (result) {
      printf("Could not initialize MPL.\n");
  }else{
			printf("initialize MPL is succeed.\n");
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_SET);									//LED3....LED4								
	}
  
  /* Compute 6-axis quaternions. ����6���ں����ݺ����Ԫ�� */
  inv_enable_quaternion();
  inv_enable_9x_sensor_fusion();

  /* Update gyro biases when not in motion.
   * �Զ����¾�ֹʱ��������ƫ�����������Ϊ���⺯����
   */
  inv_enable_fast_nomot();
  /* inv_enable_motion_no_motion(); */
  /* inv_set_no_motion_time(1000); */

  /* Update gyro biases when temperature changes. �¶ȱ仯�����������ƫ��*/
  inv_enable_gyro_tc();

  /* Allows use of the MPL APIs in read_from_mpl. ����������ṩ֧��*/
  inv_enable_eMPL_outputs();

  result = inv_start_mpl();
  if (result == INV_ERROR_NOT_AUTHORIZED) {
      while (1) {
           printf("Not authorized.\n");
      }
  }
  if (result) {
       printf("Could not start the MPL.\n");
		
  }else{
	
		printf("start the MPL is succeed.\n");
	}

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_mpu_HZ);
  /* Read back configuration in case it was set improperly. */
  mpu_get_sample_rate(&gyro_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);
  /* Sync driver configuration with MPL. */
  /* Sample rate expected in microseconds. */
  inv_set_gyro_sample_rate(1000000L / gyro_rate);
  inv_set_accel_sample_rate(1000000L / gyro_rate);
  /* ���� chip-to-body ԭ�����.
	 * ����Ӳ����λΪ dps/g's/degrees ����.
	 */
  inv_set_gyro_orientation_and_scale(
          inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)gyro_fsr<<15);
  inv_set_accel_orientation_and_scale(
          inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
          (long)accel_fsr<<15);
  /* Initialize HAL state variables. */
  hal.sensors = ACCEL_ON | GYRO_ON;
  hal.report = 0;
  hal.next_temp_tick = 0;
  hal.dmp_on = 0;
  return 0;
}



/** 
 *  @brief ��ʼ��DMP
 *  @return ��
 *  @attention ��

 */
//int module_dmp_init(void){
////	get_tick_count(&timestamp);
//  dmp_load_motion_driver_firmware();
//    dmp_set_orientation(
//        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
////    dmp_register_tap_cb(tap_cb);
//	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//����dmp����
//		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
//		    DMP_FEATURE_GYRO_CAL);
//	
//	
//	dmp_set_fifo_rate(DEFAULT_mpu_HZ);	//����DMP�������(��󲻳���200Hz)
//	 mpu_set_dmp_state(1);
//   hal.dmp_on = 1;
// 
//	return 0;
//}




//mpu6050,dmp��ʼ��
//����ֵ:0,����
//    ����,ʧ��
int module_dmp_init(void)
{
	uint8_t res=0;
//	struct int_param_s int_param;
	

//	if(mpu_init(&int_param)==0)	//��ʼ��MPU6050
//	{	 
		res=dmp_load_motion_driver_firmware();		//����dmp�̼�
		if(res)
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_SET);									//LED5....LED6
			return 4; 
		res=dmp_set_orientation(
										inv_orientation_matrix_to_scalar(gyro_pdata.orientation));//���������Ƿ���
		if(res)return 5; 
		res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|	//����dmp����
		    DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
		    DMP_FEATURE_GYRO_CAL);
		if(res)return 6; 
		res=dmp_set_fifo_rate(DEFAULT_mpu_HZ);	//����DMP�������(��󲻳���200Hz)
		if(res)return 7;   
		res=run_self_test();		//�Լ�
		if(res)return 8;    
		res=mpu_set_dmp_state(1);	//ʹ��DMP
		if(res)return 9;
//		dmp_register_tap_cb(tap_cb);
		
			 hal.dmp_on = 1;
//	}

	return 0;	
}

//MPU6050�Բ���
//����ֵ:0,����
//    ����,ʧ��
int run_self_test(void)
{
	int result;
	//char test_packet[4] = {0};
	long gyro[3], accel[3]; 
	result = mpu_run_self_test(gyro, accel);
	
	if (result == 0x7) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
	
		float sens;
		unsigned short accel_sens;
		mpu_get_gyro_sens(&sens);
		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);
		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
		
		
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_SET);									//LED5....LED6
		return 0;
	}else return 1;
}


/** 
 *  @brief ��MPU��FIFO�ж�ȡ���ݲ�����
           ����Ҳ������mpl����¶ȱ仯�Ĵ���
 *  @return ��
 *  @attention ��������Ӹ����ֵ
 */
int mpu_module_sampling()
{
  unsigned char new_temp = 0,new_data = 0;
  static unsigned long ticktime = 0;
  unsigned long sensor_timestamp;
            
  
  ticktime++;
  
  /* �¶�û�б�Ҫÿ�ζ���ȡ������һ��ʱ������ȡ */
  if (ticktime > hal.next_temp_tick) {
      hal.next_temp_tick = ticktime + TEMP_READ_TICK;
      new_temp = 1; //start task temp;
  }
  
  /* û���κ�sensor��ʱ */
  if (!hal.sensors) return -1;
  
  if(hal.new_gyro && hal.dmp_on)					//������DMP   
	{
    short gyro[3], accel_short[3],sensors;
   
		long accel[3], quat[4], temperature;
    unsigned char more;

    
    dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, 
									&sensors, &more);
		 if (!more)
       hal.new_gyro = 0;
    if (sensors & INV_XYZ_GYRO) {
        /* ����������MPL. */
        inv_build_gyro(gyro, sensor_timestamp);
        new_data = 1;
        if (new_temp) {
            new_temp = 0;
            /* Temperature only used for gyro temp comp. */
            mpu_get_temperature(&temperature, &sensor_timestamp);
            inv_build_temp(temperature, sensor_timestamp);
        }
    }
    if (sensors & INV_XYZ_ACCEL) {
        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];
        inv_build_accel(accel, 0, sensor_timestamp);
        new_data = 1;
    }
		if (sensors & INV_WXYZ_QUAT) {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
	}	else if (hal.new_gyro) {                       //û����DMP
            short gyro[3], accel_short[3];
            unsigned char sensors, more;
            long accel[3], temperature;
		hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,&sensors, &more);
//										HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3, GPIO_PIN_SET);										//LED7....LED8;
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }
    if (new_data) {
        if(inv_execute_on_data()) {
            MPL_LOGE("ERROR execute on data\n");
        }
        #ifdef MPU_DEBUG
          read_from_mpl(); 
        #endif
			}
	return 0;	


//  unsigned char sensors, more,new_temp = 0;
//  static unsigned long ticktime = 0;
//  unsigned long sensor_timestamp,cycletime = 0;
//  
//  int new_data = 0;
//  
//  ticktime++;
//  
//  /* �¶�û�б�Ҫÿ�ζ���ȡ������һ��ʱ������ȡ */
//  if (ticktime > hal.next_temp_tick) {
//      hal.next_temp_tick = ticktime + TEMP_READ_TICK;
//      new_temp = 1; //start task temp;
//  }
//  
//  /* û���κ�sensor��ʱ */
//  if (!hal.sensors) return 0;
//  
//  do {
//    short gyro[3], accel_short[3];
//    long accel[3], temperature;
//    
//    cycletime++;
//    
//    mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
//        &sensors, &more);
//    if (sensors & INV_XYZ_GYRO) {
//        /* ����������MPL. */
//        inv_build_gyro(gyro, sensor_timestamp);
//        new_data = 1;
//        if (new_temp) {
//            new_temp = 0;
//            /* Temperature only used for gyro temp comp. */
//            mpu_get_temperature(&temperature, &sensor_timestamp);
//            inv_build_temp(temperature, sensor_timestamp);
//        }
//    }
//    if (sensors & INV_XYZ_ACCEL) {
//        accel[0] = (long)accel_short[0];
//        accel[1] = (long)accel_short[1];
//        accel[2] = (long)accel_short[2];
//        inv_build_accel(accel, 0, sensor_timestamp);
//        new_data = 1;
//    }
//    
//    if (new_data) {
//        if(inv_execute_on_data()) {
//            MPL_LOGE("ERROR execute on data\n");
//        }
//        
//    }
//  }
//  while(more);
//  return cycletime;

	
}

													//--------------------------------------------------------------------------//
void MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
	Sensors_I2C_WriteRegister(MPU6050_ADDR ,reg_add,1,&reg_dat);
}
void MPU6050_ReadData(uint8_t reg_add,unsigned char* Read,uint8_t num)
{
	Sensors_I2C_ReadRegister(MPU6050_ADDR,reg_add,num,Read);
}

void MPU6050_Init(void)
{
	//�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
	HAL_Delay(100);
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //�������״̬
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //�����ǲ�����
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //���ü��ٶȴ�����������16Gģʽ
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	HAL_Delay(100);
}

/**
  * @brief   ��ȡMPU6050��ID
  * @param   
  * @retval  ��������1���쳣����0
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
	if(Re != 0x68)
	{
	printf("MPU6050 dectected error!\r\n��ⲻ��MPU6050ģ�飬����ģ���뿪����Ľ���");
		return 0;
	}
	else
	{
		
		printf("MPU6050 �ɹ�");
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11, GPIO_PIN_SET);										//LED1....LED2
//		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return 1;
	}
		
}
