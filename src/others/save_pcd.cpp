/* 点群データを受け取る */

/* ROSの基本ヘッダ */
#include <ros/ros.h>                            
/* 入出力関連ヘッダ */
#include <iostream>
/* tf */
#include <tf/transform_listener.h>          //tfリスナーを定義し、座標フレームを処理する
/* Point Cloud Library */
#include <pcl_ros/point_cloud.h>            //pcl::PointCloud<T>をROSメッセージとしてPublishおよびSubscribeできる
#include <pcl_ros/transforms.h>             //ポイントクライドを任意の座標フレームに変換する
#include <pcl/point_types.h>                //PCLで実装されたすべてのPointTポイントタイプ構造体を定義する
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
/* sensor_msgs */
#include <sensor_msgs/PointCloud2.h>


typedef pcl::PointXYZ PointT; //pcl::PointXYZ → PointT　点群データ構造体
typedef pcl::PointCloud<PointT> PointCloud; //スマートポインタ　pcl::PointCloud<pcl::PointXYZ> → PointCloud　

class savePCLFileNode
{
    private:
        ros::NodeHandle nh_;                   //pub/sub
        ros::NodeHandle pnh_;                  //parameter
        /*  Subscriber*/
        ros::Subscriber sub_points_;           //点群データをSubscribeするためのサブスクライバ
        /* param */
        std::string target_frame_;             //現在のフレーム名
        std::string topic_name_;               //topic名
        /* tf */
        tf::TransformListener tf_listener_;    //tfリスナー
        /* Point Cloud */
        PointCloud::Ptr cloud_transformed_;    //座標系に変換した点群データを格納するためのPointCloud型変数（ポインター）


        /* 点群データを受け取ったときのコールバック関数 */
        void cbPoints(const sensor_msgs::PointCloud2ConstPtr &pcl_msg) 
        {
            try {
                /* sensor_msgs/PointCloud2データをpcl/PointCloudに変換 */
                PointCloud cloud_src; 
                pcl::fromROSMsg(*pcl_msg, cloud_src);

                /* 座標フレーム(原点)の変換　-> target_frameを基準にする  */
                if (this->target_frame_.empty() == false) {//フレームの有無   
                    try {
                        tf_listener_.waitForTransform(this->target_frame_, cloud_src.header.frame_id, ros::Time(0), ros::Duration(1.0));
                        pcl_ros::transformPointCloud(this->target_frame_, ros::Time(0), cloud_src, cloud_src.header.frame_id,  *this->cloud_transformed_, this->tf_listener_);
                    } 
                    catch (tf::TransformException ex) {
                        ROS_ERROR("%s", ex.what());
                        return;
                    }
                }
                /* y軸を中心にtheta回転させるコード */
                Eigen::Matrix4f rotation_matrix_y;
                float cos_theta = std::cos( -0.113446  );
                float sin_theta = std::sin( -0.113446 );
                //行列を作成する 4x4
                rotation_matrix_y << \
                 cos_theta,  0,    sin_theta, 0, \
                         0,  1,            0, 0, \
                -sin_theta,  0,    cos_theta, 0, \
                         0,  0,            0, 1;
                //回転
                pcl::transformPointCloud(*this->cloud_transformed_,*this->cloud_transformed_, rotation_matrix_y );
                
                ROS_INFO("width: %u, height: %u", this->cloud_transformed_->width, this->cloud_transformed_->height);
                // 作成したPointCloudをPCD形式で保存する
                std::cout << "savePCDFileASCII" << std::endl;
                pcl::io::savePCDFileASCII<pcl::PointXYZ> ("/home/sobit-x3/catkin_ws/src/pcl_tutorial/pcd/laboratory2_ascii.pcd", *this->cloud_transformed_); // テキスト形式で保存する
                std::cout << "savePCDFileBinary" << std::endl;
                pcl::io::savePCDFileBinary<pcl::PointXYZ> ("/home/sobit-x3/catkin_ws/src/pcl_tutorial/pcd/laboratory2_binary.pcd", *this->cloud_transformed_);  // バイナリ形式で保存する
            }

            catch (std::exception &e){
               ROS_ERROR("%s", e.what());
            }
        }

    public:
        savePCLFileNode()
        : nh_()
        , pnh_("~") //"~"を引数に初期化することで、このノードのプライベートな名前空間を使う設定になる
        {
            ros::param::get("target_frame", this->target_frame_); //得られた点群を処理しやすい座標系に変換する際の座標系の名前
            ros::param::get("topic_name", this->topic_name_); //センサが出力するPointCloudのトピック名
            ROS_INFO("target_frame = '%s'", this->target_frame_.c_str());
            ROS_INFO("topic_name = '%s'", this->topic_name_.c_str());

            /* 点群データをサブスクライブ */
            sub_points_ = nh_.subscribe(this->topic_name_, 5, &savePCLFileNode::cbPoints, this); //Cbはメンバ関数

            /* メモリを解放し、引数で与えられたポインタを扱う */
            cloud_transformed_.reset(new PointCloud());
        }
};

int main(int argc, char *argv[])
{
  //ノードの初期化
  ros::init(argc, argv, "savePCLFile_node");

  //savePCLFileNodeのインスタンスを作成
  savePCLFileNode savePCLFile;
  ros::spin();

}
