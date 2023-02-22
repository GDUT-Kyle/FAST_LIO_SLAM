#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

/**
 * @brief: 所使用的雷达类型
 */
enum LID_TYPE{AVIA = 1, VELO16, OUST64}; //{1, 2, 3}

/**
 * @brief: 局部特征的类型
 */
enum Feature{
  Nor,        // 正常点
  Poss_Plane, // 可能的平面点
  Real_Plane, // 确定的平面点
  Edge_Jump,  // 有跨越的边
  Edge_Plane, // 边上的平面点
  Wire,       // 线段 这个也许当了无效点？也就是空间中的小线段？
  ZeroPoint   // 无效点 程序中未使用
};

enum Surround{Prev, 
              Next};

/**
 * @brief: 表示有跨越边的类型
 */
enum E_jump{
  Nr_nor,  // 正常
  Nr_zero, // 0
  Nr_180,  // 180
  Nr_inf,  // 无穷大 跳变较远？
  Nr_blind // 在盲区？
};

/**
 * @brief: 用于存储激光雷达点的一些其他属性
 */
struct orgtype
{
  double range; // 点云在xy平面离雷达中心的距离
  double dista; // 当前点与后一个点之间的距离
  //假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
  double angle[2];  // 这个是角OAM和角OAN的cos值
  double intersect; // 这个是角MAN的cos值
  E_jump edj[2];    // 前后两点的类型
  Feature ftype;    // 点类型

  // 构造函数
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor; //默认为正常点
    intersect = 2;
  }
};

/**
 * @brief: 不同(旋转式)雷达对应不同的点类型，并且进行相应的类型注册
 */
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;                // 4D点坐标类型
      float intensity;                // 强度
      float time;                     // 时间
      uint16_t ring;                  // 点所属的圈数
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;                // 4D点坐标类型
      float intensity;                // 强度
      uint32_t t;                     // 时间
      uint16_t reflectivity;          // 反射率
      uint8_t ring;                   // 点所属的圈数
      uint16_t ambient;               // 没用到
      uint32_t range;                 // 距离
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

/**
 * @description: 对激光雷达传入的Scan进行预处理，主要工作是进行局部特征提取
 */
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  /**
   * @description: 对livox自定义的点云帧进行预处理
   * @param {ConstPtr} &msg
   * @param {Ptr} &pcl_out
   * @return {*}
   */  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  /**
   * @description: 对旋转式激光雷达的点云帧进行预处理
   * @param {ConstPtr} &msg
   * @param {Ptr} &pcl_out
   * @return {*}
   */  
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf; // 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS; // 雷达类型、采样间隔、扫描线数
  double blind; // 最小距离阈值(盲区)
  bool feature_enabled, given_offset_time; // 是否提取特征、是否进行时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn;  // 发布全部点、发布平面点、发布边缘点
    

  private:
  /**
   * @description: livox回调函数
   * @param {ConstPtr} &msg
   * @return {*}
   */  
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);

  /**
   * @description: ouster64回调函数
   * @param {ConstPtr} &msg
   * @return {*}
   */  
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @description: velodyne回调函数
   * @param {ConstPtr} &msg
   * @return {*}
   */  
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
