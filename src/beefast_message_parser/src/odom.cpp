#include "beefast_message_parser/odom.hpp"
namespace beefast_message_parser {
// double secs2;
// double secs3;
OdomNode::OdomNode( std::string name )
    : rclcpp::Node( name, "",  // 1
                    rclcpp::NodeOptions().allow_undeclared_parameters( true ) ) {
    delta_distance_ = 0;
    left_count_     = 0;
    right_count_    = 0;
    call_count_     = 0;

    auto qos = rclcpp::QoS( rclcpp::KeepLast( 10 ) ).transient_local().reliable();
    // odom_publisher_ =  create_publisher<nav_msgs::msg::Odometry>("odom_raw", qos);
    // imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu", qos);

    visual_publisher_ = create_publisher< geometry_msgs::msg::TransformStamped >( "/odom_visual", qos );

    tf_broadcaster_ = std::make_unique< tf2_ros::TransformBroadcaster >( this );
    // odom_visual_publisher_ = create_publisher<geometry_msgs::msg::TransformStamped>("/odom_visual", qos);
    RCLCPP_INFO( get_logger(), "OdomNode constructor init" );
    init();
    RCLCPP_INFO( get_logger(), "OdomNode constructor done" );
}

void OdomNode::load_params() {
    auto params = list_parameters( {}, 1 );
    RCLCPP_INFO( get_logger(), "params size :%ld", params.names.size() );
    // 声明参数默认值
    // declare_parameter("odom_topic", "/odom");
    // declare_parameter("imu_topic", "/imu");
    // declare_parameter("odom_angle_source", "odom");
    // declare_parameter("wheel_diameter", 0.065);
    // declare_parameter("pulse_resolution_per_turn", 1560);
    // declare_parameter("wheel_base", 0.16);
    get_parameter( "odom_topic", odom_topic_ );
    get_parameter( "imu_topic", imu_topic_ );
    get_parameter( "odom_angle_source", odom_angle_source_ );
    get_parameter( "wheel_diameter", wheel_diameter_ );
    // get_parameter("pulse_resolution_per_turn", pulse_resolution_);
    // get_parameter("rate", rate_);
    get_parameter( "wheel_base", wheel_base_ );
    RCLCPP_INFO( get_logger(), "wheel_diameter:%s, wheel_distance:%s", toString( wheel_diameter_, 3 ).c_str(), toString( wheel_base_, 3 ).c_str() );
    RCLCPP_INFO( get_logger(), "angle source:%s, odom topic:%s, imu topic:%s", odom_angle_source_.c_str(), odom_topic_.c_str(), imu_topic_.c_str() );

    auto qos        = rclcpp::QoS( rclcpp::KeepLast( 10 ) ).transient_local().reliable();
    odom_publisher_ = create_publisher< nav_msgs::msg::Odometry >( odom_topic_, qos );
    imu_publisher_  = create_publisher< sensor_msgs::msg::Imu >( imu_topic_, qos );
}

OdomNode::~OdomNode() {
    std::cout << "odom_pub destructor." << std::endl;
    // Clean up
}

int OdomNode::load_acc_params( std::string path ) {
    std::ifstream file( path );
    if ( file.is_open() ) {
        double mat[ 9 ] = { 0 };

        for ( int i = 0; i < 9; i++ )
            file >> mat[ i ];

        acc_mis_mat_ = Eigen::Map< const Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( mat );

        for ( int i = 0; i < 9; i++ )
            file >> mat[ i ];

        acc_scale_mat_ = Eigen::Map< const Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( mat );

        for ( int i = 0; i < 3; i++ )
            file >> mat[ i ];

        acc_bias_vec_ = Eigen::Map< const Eigen::Matrix< double, 3, 1 > >( mat );

        acc_ms_mat_ = acc_mis_mat_ * acc_scale_mat_;
    }

    return 0;
}

int OdomNode::load_gyro_params( std::string path ) {
    std::ifstream file( path );
    if ( file.is_open() ) {
        double mat[ 9 ] = { 0 };

        for ( int i = 0; i < 9; i++ )
            file >> mat[ i ];

        gyro_mis_mat_ = Eigen::Map< const Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( mat );

        for ( int i = 0; i < 9; i++ )
            file >> mat[ i ];

        gyro_scale_mat_ = Eigen::Map< const Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( mat );

        for ( int i = 0; i < 3; i++ )
            file >> mat[ i ];

        gyro_bias_vec_ = Eigen::Map< const Eigen::Matrix< double, 3, 1 > >( mat );

        gyro_ms_mat_ = gyro_mis_mat_ * gyro_scale_mat_;
    }

    return 0;
}

int OdomNode::load_gravity_params( std::string path ) {
    std::ifstream file( path );
    if ( file.is_open() ) {
        double mat[ 3 ] = { 0 };

        for ( int i = 0; i < 3; i++ )
            file >> mat[ i ];

        actual_g_ = Eigen::Vector3d( mat[ 0 ], mat[ 1 ], mat[ 2 ] );
    }

    return 0;
}

int OdomNode::init() {
    acc_mis_mat_.setIdentity();
    acc_scale_mat_.setIdentity();
    acc_bias_vec_.setZero();
    acc_ms_mat_.setIdentity();

    gyro_mis_mat_.setIdentity();
    gyro_scale_mat_.setIdentity();
    gyro_bias_vec_.setZero();
    gyro_ms_mat_.setIdentity();

    actual_g_ = Eigen::Vector3d( 0, 0, 0 );

    int ret = -1;
    ret     = load_acc_params( "/home/sunrise/sys_lib/car_ws/src/beefast_message_parser/etc/imu_acc.calib" );
    if ( ret != 0 ) {
        std::cout << "fail to load acc param" << std::endl;
        return -1;
    }

    ret = load_gyro_params( "/home/sunrise/sys_lib/car_ws/src/beefast_message_parser/etc/imu_gyro.calib" );
    if ( ret != 0 ) {
        std::cout << "fail to load acc param" << std::endl;
        return -1;
    }

    ret = load_gravity_params( "/home/sunrise/sys_lib/car_ws/src/beefast_message_parser/etc/gravity_imu_frame.calib" );
    if ( ret != 0 ) {
        std::cout << "fail to load acc param" << std::endl;
        return -1;
    }

    return 0;
}

void OdomNode::ResetPostion() {
    std::cout << " reset position  " << std::endl;
    this->x_  = 0;
    this->y_  = 0;
    this->th_ = 0;
}

void OdomNode::pub_tf( double secs ) {

    geometry_msgs::msg::TransformStamped transform;
    double                               seconds = secs;
    transform.header.stamp                       = rclcpp::Time( static_cast< uint64_t >( seconds * 1e9 ) );
    transform.header.frame_id                    = "odom";
    transform.child_frame_id                     = "base_link";
    transform.transform.translation.x            = this->x_;
    transform.transform.translation.y            = this->y_;
    transform.transform.translation.z            = 0.0;
    tf2::Quaternion quat;
    quat.setRPY( 0, 0, this->th_ );
    tf2::convert( quat, transform.transform.rotation );
    tf_broadcaster_->sendTransform( transform );

    geometry_msgs::msg::TransformStamped transform_imu;
    transform_imu.header.stamp            = rclcpp::Time( static_cast< uint64_t >( seconds * 1e9 ) );
    transform_imu.header.frame_id         = "base_link";
    transform_imu.child_frame_id          = "imu";
    transform_imu.transform.translation.x = 0.0;
    transform_imu.transform.translation.y = 0.0;
    transform_imu.transform.translation.z = 0.0;
    tf2::convert( quat, transform_imu.transform.rotation );
    tf_broadcaster_->sendTransform( transform_imu );
}

void OdomNode::reset_motion_data() {

    velocity_left_  = 0.0;
    velocity_right_ = 0.0;
    angle_x_        = 0.0;
    angle_y_        = 0.0;
    angle_z_        = 0.0;
    accel_x_        = 0.0;
    accel_y_        = 0.0;
    accel_z_        = 0.0;
    gyro_x_         = 0.0;
    gyro_y_         = 0.0;
    gyro_z_         = 0.0;
    quat_x_         = 0.0;
    quat_y_         = 0.0;
    quat_z_         = 0.0;
    quat_w_         = 0.0;
}

void OdomNode::set_motion_data( const beefast_message_parser::MotionMessage& message ) {
    velocity_left_  = message.velocity_left / 1000.0;
    velocity_right_ = message.velocity_right / 1000.0;
    angle_x_        = message.angle_x;
    angle_y_        = message.angle_y;
    angle_z_        = message.angle_z;
    accel_x_        = message.accel_x;
    accel_y_        = message.accel_y;
    accel_z_        = message.accel_z;
    gyro_x_         = message.gyro_x;
    gyro_y_         = message.gyro_y;
    gyro_z_         = message.gyro_z;
    quat_x_         = message.quat_x;
    quat_y_         = message.quat_y;
    quat_z_         = message.quat_z;
    quat_w_         = message.quat_w;
    //测试
    double angle_now = ( angle_z_ / 180 ) * M_PI;
    // RCLCPP_INFO( rclcpp::get_logger( "beefast_message_parser" ), "velocity_left_:%f, velocity_right_:%f , angle_: %f ", velocity_left_, velocity_right_, angle_z_ );
}

void OdomNode::publish_odometry( double secs ) {
    cumulate_odometry();

    auto odom            = nav_msgs::msg::Odometry();
    odom.header.stamp    = this->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    // 设置位置
    odom.pose.pose.position.x = this->x_;
    odom.pose.pose.position.y = this->y_;
    odom.pose.pose.position.z = 0;

    // 设置姿态
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg( tf2::Quaternion( tf2::Vector3( 0, 0, 1 ), this->th_ ) );
    odom.pose.pose.orientation               = odom_quat;

    // 设置速度
    odom.twist.twist.linear.x  = this->vx_;
    odom.twist.twist.angular.z = this->vth_;
    odom_publisher_->publish( odom );
    if ( call_count_ % 2 == 0 ) {
        geometry_msgs::msg::TransformStamped visual_msg;
        visual_msg.header.stamp            = this->get_clock()->now();
        visual_msg.header.frame_id         = "odom";
        visual_msg.transform.translation.x = this->x_;
        visual_msg.transform.translation.y = this->y_;
        visual_msg.transform.rotation.x    = velocity_left_;
        visual_msg.transform.rotation.y    = velocity_right_;
        visual_msg.transform.rotation.w    = this->th_;
        visual_publisher_->publish( visual_msg );
    }
}

void OdomNode::cumulate_odometry()  // odom算法
{

    // wheel_base_ = 0.345 ; // 两轮轴距  m
    // double hz = 20;  //hz

    rate_            = 20;
    double t_rate    = 1.0 / rate_;  // hz->秒
    double angle_now = ( angle_z_ / 180 ) * M_PI;
    this->vx_        = ( velocity_right_ + velocity_left_ ) / 2;
    this->vth_       = ( velocity_right_ - velocity_left_ ) / wheel_base_;

    double distance_t  = this->vx_ * t_rate;   //位移增量
    double delta_theta = this->vth_ * t_rate;  // odom角度增量

    if ( odom_angle_source_ == "imu" ) {
        this->x_  = this->x_ + distance_t * cos( angle_now );
        this->y_  = this->y_ + distance_t * sin( angle_now );
        this->th_ = angle_now;
    }
    else {
        this->x_ = this->x_ + distance_t * cos( this->th_ + 0.5 * delta_theta );
        this->y_ = this->y_ + distance_t * sin( this->th_ + 0.5 * delta_theta );
        this->th_ += delta_theta;
    }

    // double delta_theta = angle_now - this->th_;
    //世界坐标系x方向和y方向下的最终坐标为
    // RCLCPP_INFO(rclcpp::get_logger("beefast_message_parser"), "this->th_:%s delta_theta:%s
    // angle_now:%s",toString(this->th_,4).c_str(),toString(delta_theta,4).c_str(),toString(angle_now,4).c_str());
    // if (call_count_ % 2 == 0) {
    // geometry_msgs::msg::TransformStamped visual_msg;
    // visual_msg.header.stamp = this->get_clock()->now();
    // visual_msg.header.frame_id = "odom";
    // visual_msg.transform.translation.x = left_count_;
    // visual_msg.transform.translation.y = right_count_;
    // visual_msg.transform.rotation.x = distance_t*cos(angle_now);
    // visual_msg.transform.rotation.y = call_count_; //接收话题的书目
    // visual_msg.transform.rotation.w = delta_distance_;
    // odom_visual_publisher_->publish(visual_msg);
    // }

    // RCLCPP_INFO(rclcpp::get_logger("beefast_message_parser"), "vx:%s vth:%s x:%s y:%s
    // th:%s",toString(this->vx_,4).c_str(),toString(this->vth_,4).c_str(),toString(this->x_,4).c_str(),toString(this->y_,4).c_str(),toString(this->th_,4).c_str());
}

void OdomNode::publish_imu( double secs ) {
    // rclcpp::Time now = this->get_clock()->now() - rclcpp::Duration::from_seconds(0.4); // 获取时间戳 0.4s
    // rclcpp::Time now = this->get_clock()->now(); // 获取时间戳
    rclcpp::Time now = rclcpp::Time( static_cast< uint64_t >( secs * 1e9 ) );  // 获取时间戳

    sensor_msgs::msg::Imu imu_data;
    sensor_msgs::msg::Imu imu_offline_data;  //
    //------------------imu data----------------
    imu_data.header.stamp    = now;
    imu_data.header.frame_id = "imu";

    double acc_x = accel_x_ / 32768.00 * 2 * 9.8;  //线性加速度 x
    double acc_y = accel_y_ / 32768.00 * 2 * 9.8;  //线性加速度 y
    double acc_z = accel_z_ / 32768.00 * 2 * 9.8;  //线性加速度 z

    double gyro_x = gyro_x_ * M_PI * 2000 / 32768.00 / 180;  // 陀螺仪 x
    double gyro_y = gyro_y_ * M_PI * 2000 / 32768.00 / 180;  // 陀螺仪 y
    double gyro_z = gyro_z_ * M_PI * 2000 / 32768.00 / 180;  // 陀螺仪 z

    imu_data.orientation.x = quat_x_ / 1073741824.0;  // 四元数 x
    imu_data.orientation.y = quat_y_ / 1073741824.0;  // 四元数 y
    imu_data.orientation.z = quat_z_ / 1073741824.0;  // 四元数 z
    imu_data.orientation.w = quat_w_ / 1073741824.0;  // 四元数 w

    Eigen::Matrix< double, 3, 1 > tmp_acc( acc_x, acc_y, acc_z );
    Eigen::Matrix< double, 3, 1 > tmp_gyro( gyro_x, gyro_y, gyro_z );

    // 内参修正
    tmp_acc  = acc_correct( tmp_acc );
    tmp_gyro = gyro_correct( tmp_gyro );

    // acc 重力消除
    Eigen::Vector3d acc_1( tmp_acc );
    Eigen::Vector3d gyro_1( tmp_gyro );

    acc_1 = acc_1 - actual_g_;

    imu_data.linear_acceleration.x = acc_1.x();  //线性加速度 x
    imu_data.linear_acceleration.y = acc_1.y();  //线性加速度 y
    imu_data.linear_acceleration.z = acc_1.z();  //线性加速度 z

    imu_data.angular_velocity.x = gyro_1.x();  // 陀螺仪 x
    imu_data.angular_velocity.y = gyro_1.y();  // 陀螺仪 y
    imu_data.angular_velocity.z = gyro_1.z();  // 陀螺仪 z
    // RCLCPP_INFO(rclcpp::get_logger("beefast_message_parser"),"imu_data.angular_velocity.x:%f imu_data.angular_velocity.y:%f
    // imu_data.angular_velocity.z:%f",imu_data.angular_velocity.x,imu_data.angular_velocity.y,imu_data.angular_velocity.z);
    // RCLCPP_INFO(rclcpp::get_logger("beefast_message_parser"),"imu_data.linear_acceleration.x:%f imu_data.linear_acceleration.y:%f
    // imu_data.linear_acceleration.z:%f",imu_data.linear_acceleration.x,imu_data.linear_acceleration.y,imu_data.linear_acceleration.z);
    // RCLCPP_INFO(rclcpp::get_logger("beefast_message_parser"), "angle_x:%s angle_y:%s
    // angle_z:%s",toString(this->angle_x_,4).c_str(),toString(this->angle_y_,4).c_str(),toString(this->angle_z_,4).c_str());
    tf2::Quaternion q( imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w );
    tf2::Matrix3x3  m( q );
    double          roll, pitch, yaw;
    m.getRPY( roll, pitch, yaw );
    // RCLCPP_INFO(rclcpp::get_logger("beefast_message_parser"), "roll:%s pictch:%s yaw:%s",toString(roll,4).c_str(),toString(pitch,4).c_str(),toString(yaw,4).c_str());
    imu_publisher_->publish( imu_data );
}

void OdomNode::motion_callback( const beefast_message_parser::MotionMessage& message, double secs ) {
    call_count_++;
    reset_motion_data();
    set_motion_data( message );
    // double secs8 = this->get_clock()->now().seconds();
    // RCLCPP_INFO(rclcpp::get_logger("7messsge"),"time secs8 : %f",secs8);
    publish_odometry( secs );
    if ( odom_topic_ == "odom" ) {
        pub_tf( secs );
    }
    double secs9 = this->get_clock()->now().seconds();
    // RCLCPP_INFO(rclcpp::get_logger("7messsge"),"time secs9 : %f",secs9);
    // publish_imu(secs);
}

std::string OdomNode::toString( double val, int precision ) {
    std::ostringstream out;
    out.precision( precision );
    out << std::fixed << val;
    return out.str();
}
}  // namespace beefast_message_parser
