#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <fstream>

extern "C" {
#include "mpu9150.h"
#include "linux_glue.h"
}

#define MPU_SUCCESS(X) ((X) > -1)

class ImuMpu9Dof
{
public:
  enum
  {
    IMU_DEFAULT_I2C_BUS = 1,
    IMU_DEFAULT_YAW_MIX_FACTOR = 4,
    IMU_DEFAULT_SAMPLING_RATE = 10
  };

  ImuMpu9Dof()
      : i2c_bus_id_(IMU_DEFAULT_I2C_BUS), driver_verbose_(false),
        yaw_mix_factor_(IMU_DEFAULT_YAW_MIX_FACTOR),
        sampling_rate_(double(IMU_DEFAULT_SAMPLING_RATE)),
        accel_cal_filename_("./accelcal.txt"),
        mag_cal_filename_("./magcal.txt"), frame_id_("imu_link")
  {
  }

  void init()
  {
    read_parameters();
    load_calibration_data();

    data_raw_pub_ = nh_.advertise<sensor_msgs::Imu>("data_raw", 1);
    mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("mag", 1);

    // Initialize the MPU-9150
    mpu9150_set_debug(driver_verbose_);
    if (!MPU_SUCCESS(
            mpu9150_init(i2c_bus_id_, sampling_rate_, yaw_mix_factor_)))
    {
      ROS_FATAL("mpu9150_init(...) failed");
      ROS_BREAK();
    }

    imu_msg_.header.frame_id = frame_id_;
    mag_msg_.header.frame_id = frame_id_;
  }

  void spin()
  {
    ros::Rate loop_rate(sampling_rate_);

    while (ros::ok())
    {
      if (MPU_SUCCESS(mpu9150_read(&mpu_reading_)))
      {
        on_new_sample();
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  enum {
    AK8975_DATA_MAX = 4095,
    AK8975_DATA_MIN = -4096,
    AK8975_uT_MAX = 1229,
    AK8975_uT_MIN = -1229,
    MPU6050_DATA_MAX = 32767,
    MPU6050_DATA_MIN = -32768,
    MPU6050_G_MAX = 2,
    MPU6050_G_MIN = -2,
    MPU6050_DPS_MAX = 2000,
    MPU6050_DPS_MIN = -2000
  };
  
  void read_parameters()
  {
    ros::NodeHandle priv_nh_("~");

    if (!priv_nh_.getParam("i2c_bus_id", i2c_bus_id_))
    {
      ROS_INFO("Using default i2c_bus_id=%d", i2c_bus_id_);
    }

    if (!priv_nh_.getParam("sampling_rate", sampling_rate_))
    {
      ROS_INFO("Using default sampling_rate=%f", sampling_rate_);
    }

    if (!priv_nh_.getParam("driver_verbose", driver_verbose_))
    {
      ROS_INFO("Using default driver_verbose=%d", int(driver_verbose_));
    }

    if (!priv_nh_.getParam("yaw_mix_factor", yaw_mix_factor_))
    {
      ROS_INFO("Using default yaw_mix_factor=%d", yaw_mix_factor_);
    }

    if (!priv_nh_.getParam("mag_cal_filename", mag_cal_filename_))
    {
      ROS_INFO("Using default mag_cal_filename=%s", mag_cal_filename_.c_str());
    }

    if (!priv_nh_.getParam("accel_cal_filename", accel_cal_filename_))
    {
      ROS_INFO("Using default accel_cal_filename=%s",
               accel_cal_filename_.c_str());
    }

    if (!priv_nh_.getParam("frame_id", frame_id_))
    {
      ROS_INFO("Using default frame_id=%s", frame_id_.c_str());
    }
  }

  void load_calibration_data()
  {
    caldata_t mag_cal;
    load_calibration_data(mag_cal_filename_.c_str(), &mag_cal);
    mpu9150_set_mag_cal(&mag_cal);

    ROS_INFO("Loaded magnetometer calibration data");

    caldata_t accel_cal;
    load_calibration_data(accel_cal_filename_.c_str(), &accel_cal);
    mpu9150_set_accel_cal(&accel_cal);

    ROS_INFO("Loaded accelerometer calibration data");
  }

  void load_calibration_data(const char* cal_filename, caldata_t* cal_data)
  {
    std::ifstream cal_file;

    cal_file.open(cal_filename);
    if (!cal_file.is_open())
    {
      ROS_FATAL("Can't locate calibration file %s", cal_filename);
      ROS_BREAK();
    }

    short values[6];
    int i = 0;
    for (; cal_file && i < 6; ++i)
    {
      cal_file >> values[i];
      ROS_DEBUG("%s: values[%d]=%d", cal_filename, i, int(values[i]));
    }

    if (i < 6)
    {
      ROS_FATAL("Incomplete calibration file %s", cal_filename);
      ROS_BREAK();
    }

    cal_data->offset[0] = (values[0] + values[1]) / 2;
    cal_data->offset[1] = (values[2] + values[3]) / 2;
    cal_data->offset[2] = (values[4] + values[5]) / 2;

    cal_data->range[0] = values[1] - cal_data->offset[0];
    cal_data->range[1] = values[3] - cal_data->offset[1];
    cal_data->range[2] = values[5] - cal_data->offset[2];

    ROS_INFO("Calibration: X:[%d,%d],Y:[%d,%d],Z:[%d,%d]",
      values[0], values[1], values[2], values[3], values[4], values[5]);

    cal_file.close();
  }

  void on_new_sample()
  {
    ros::Time current_time = ros::Time::now();

    imu_msg_.header.stamp = current_time;
    imu_msg_.angular_velocity.x = raw_gyro_to_rps(mpu_reading_.rawGyro[VEC3_X]);
    imu_msg_.angular_velocity.y = raw_gyro_to_rps(mpu_reading_.rawGyro[VEC3_Y]);
    imu_msg_.angular_velocity.z = raw_gyro_to_rps(mpu_reading_.rawGyro[VEC3_Z]);
    imu_msg_.linear_acceleration.x = raw_accel_to_mpss(mpu_reading_.calibratedAccel[VEC3_X]);
    imu_msg_.linear_acceleration.y = raw_accel_to_mpss(mpu_reading_.calibratedAccel[VEC3_Y]);
    imu_msg_.linear_acceleration.z = raw_accel_to_mpss(mpu_reading_.calibratedAccel[VEC3_Z]);
    data_raw_pub_.publish(imu_msg_);

    mag_msg_.header.stamp = current_time;
    mag_msg_.magnetic_field.x = raw_mag_to_tesla(mpu_reading_.calibratedMag[VEC3_X]);
    mag_msg_.magnetic_field.y = raw_mag_to_tesla(mpu_reading_.calibratedMag[VEC3_Y]);
    mag_msg_.magnetic_field.z = raw_mag_to_tesla(mpu_reading_.calibratedMag[VEC3_Z]);
    mag_pub_.publish(mag_msg_);
  }
  
  static double raw_gyro_to_rps(short raw_gyro)
  {
    if (raw_gyro > 0)
    {
      return (raw_gyro * MPU6050_DPS_MAX * M_PI) / (180.0 * MPU6050_DATA_MAX);
    }
    else
    {
      return (raw_gyro * MPU6050_DPS_MIN * M_PI) / (180.0 * MPU6050_DATA_MIN);
    }
  }
  
  static double raw_accel_to_mpss(short raw_accel)
  {
    static const double G_MPSS = 9.80665;
    
    if (raw_accel > 0)
    {
      return (raw_accel * MPU6050_G_MAX * G_MPSS) / MPU6050_DATA_MAX;
    }
    else
    {
      return (raw_accel * MPU6050_G_MIN * G_MPSS) / MPU6050_DATA_MIN;
    }
  }
  
  static double raw_mag_to_tesla(short raw_mag)
  {
    if (raw_mag > 0)
    {
      return (raw_mag * AK8975_uT_MAX) / (1000000.0 * AK8975_DATA_MAX);
    }
    else
    {
      return (raw_mag * AK8975_uT_MIN) / (1000000.0 * AK8975_DATA_MIN);
    }
  }

  
  // node parameters
  double sampling_rate_;
  int i2c_bus_id_;
  bool driver_verbose_;
  int yaw_mix_factor_;
  std::string mag_cal_filename_;
  std::string accel_cal_filename_;
  std::string frame_id_;

  ros::NodeHandle nh_;
  ros::Publisher data_raw_pub_;
  ros::Publisher mag_pub_;
  mpudata_t mpu_reading_;
  sensor_msgs::MagneticField mag_msg_;
  sensor_msgs::Imu imu_msg_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_mpu_9dof");

  // required to avoid compiler link errors with linux_glue
  // todo: fix bug
  (void)linux_get_ms(0);

  ImuMpu9Dof imu;

  imu.init();
  imu.spin();

  return 0;
}
