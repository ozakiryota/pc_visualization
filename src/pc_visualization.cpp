#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

class PcVisualization{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
        /*visulalizer*/
        pcl::visualization::PCLVisualizer visualizer_ {"pc_visualization"};
        /*list*/
        std::vector<std::string> field_list_;
		/*parameter*/
		double color_r_;
		double color_g_;
		double color_b_;
		double size_;

	public:
		PcVisualization();
		void listUpPointType(void);
		void callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		template<typename CloudPtr, typename PointType> void visualizePc(CloudPtr pc, PointType no_use);
		template<typename CloudPtr, typename PointType> void visualizeNc(CloudPtr nc, PointType no_use);
};

PcVisualization::PcVisualization()
	// : nh_private_("~")
{
	std::cout << "----- pc_visualization -----" << std::endl;
    /*parameter*/
	nh_private_.param("color_r", color_r_, 0.0);
	std::cout << "color_r_ = " << color_r_ << std::endl;
    nh_private_.param("color_g", color_g_, 0.0);
	std::cout << "color_g_ = " << color_g_ << std::endl;
    nh_private_.param("color_b", color_b_, 0.0);
	std::cout << "color_b_ = " << color_b_ << std::endl;
    nh_private_.param("size", size_, 1.0);
	std::cout << "size_ = " << size_ << std::endl;
	/*subscriber*/
	sub_ = nh_.subscribe("/point_cloud", 1, &PcVisualization::callback, this);
    /*initialization*/
	listUpPointType();
	visualizer_.setBackgroundColor(1.0, 1.0, 1.0);
	visualizer_.addCoordinateSystem(1.0, "axis");
}

void PcVisualization::listUpPointType(void)
{
	field_list_ = {
		"x y z",
		"x y z intensity",
		"x y z intensity ring time",	//velodyne
		"x y z strength",
		"x y z normal_x normal_y normal_z curvature",
		"x y z intensity normal_x normal_y normal_z curvature"
	};
}

void PcVisualization::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    std::string fields = pcl::getFieldsList(*msg);

	if(fields == field_list_[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *pc);
		visualizePc(pc, pc->points[0]);
	}
	else if(fields == field_list_[1] || fields == field_list_[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(*msg, *pc);
		visualizePc(pc, pc->points[0]);
	}
	else if(fields == field_list_[3]){
		pcl::PointCloud<pcl::InterestPoint>::Ptr pc (new pcl::PointCloud<pcl::InterestPoint>);
		pcl::fromROSMsg(*msg, *pc);
		visualizePc(pc, pc->points[0]);
	}
	else if(fields == field_list_[4]){
		pcl::PointCloud<pcl::PointNormal>::Ptr nc (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromROSMsg(*msg, *nc);
		visualizeNc(nc, nc->points[0]);
	}
	else if(fields == field_list_[5]){
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr nc (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::fromROSMsg(*msg, *nc);
		visualizeNc(nc, nc->points[0]);
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr, typename PointType>
void PcVisualization::visualizePc(CloudPtr pc, PointType no_use)
{
    visualizer_.removeAllPointClouds();

	visualizer_.addPointCloud<PointType>(pc, "pc");
	visualizer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color_r_, color_g_, color_b_, "pc");
	visualizer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_, "pc");

	visualizer_.spinOnce();
}

template<typename CloudPtr, typename PointType>
void PcVisualization::visualizeNc(CloudPtr nc, PointType no_use)
{
    visualizer_.removeAllPointClouds();

	visualizer_.addPointCloudNormals<PointType>(nc, 1, 1.0, "nc");
	visualizer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color_r_, color_g_, color_b_, "nc");
	visualizer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, size_, "nc");

	visualizer_.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_visualization");
	
	PcVisualization pc_visualization;

	ros::spin();
}