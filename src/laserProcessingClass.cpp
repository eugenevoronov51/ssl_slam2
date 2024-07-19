/**
  * Author of Fast_SSL_SLAM: Eugene Voronov
  * Email johnvoronov@gmail.com
  * c++ lib
**/
#include "laserProcessingClass.h"

using namespace std;

void LaserProcessingClass::init(lidar::Lidar lidar_param_in){
    
    lidar_param = lidar_param_in;

}

void LaserProcessingClass::featureExtraction(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_edge, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_surf
){

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);
   
    //coordinate transform
    for (int i = 0; i < (int) pc_in->points.size(); i++){
        double new_x = pc_in->points[i].z;
        double new_y = -pc_in->points[i].x;
        double new_z = -pc_in->points[i].y;
        pc_in->points[i].x = new_x;
        pc_in->points[i].y = new_y;
        pc_in->points[i].z = new_z;
    }


    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> laserClouds;

    // Get firsts coords
    double x = pc_in->points[0].x;
    double y = pc_in->points[0].y;
    double z = pc_in->points[0].z;

    double last_angle = atan2(z, y) * 180 / M_PI; // y, x
    //std::cerr << "Last angle: " << last_angle << std::endl;

    int points_in_batch = 0;
    int point_size = pc_in->points.size()-1;
    for (int i = 0; i < (int) pc_in->points.size(); i++)
    {
        // Current coords of points
        double x_i = pc_in->points[i].x;
        double y_i = pc_in->points[i].y;
        double z_i = pc_in->points[i].z;

        //pc_in->points[i].intensity = (double)i / pc_in->points.size();
        int scanID=0;
        double distance = sqrt(pow(x_i, 2) + pow(y_i, 2) + pow(z_i, 2)); // Pythagorean theorem
        double angle = atan2(x_i, z_i) * 180 / M_PI;

        points_in_batch++;

        // Get difference between last angel and current angel
        if(fabs(angle - last_angle) > 0.1){
            // std::cerr << "Difference between last angel and current angel: " << fabs(angle - last_angle) << std::endl;
            if (points_in_batch > 100) {
                //std::cerr << "Points in batch: " << points_in_batch << std::endl;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_temp(new pcl::PointCloud<pcl::PointXYZRGB>());

                for(int k=0;k<points_in_batch;k++){
                    pc_temp->push_back(pc_in->points[i-points_in_batch+k+1]);

                }
                if(pc_temp->points.size()>0)
                    laserClouds.push_back(pc_temp);
            }
            points_in_batch = 0;
            last_angle = angle;
        }

    }

    //ROS_WARN_ONCE("total points array %d", laserClouds.size());
    // std::cerr << "Total points array in laser clouds: " << laserClouds.size() << std::endl;

    for(int i = 0; i < (int) laserClouds.size(); i++) {
        std::vector<Double2d> cloudDifferences;
        int total_points = laserClouds[i]->points.size() - 10;

        double max_distance = 0.005;
        double min_distance = 0.001;

        for (int j = 5; j < (int) laserClouds[i]->points.size() - 5; j++) {
            double x_diff = laserClouds[i]->points[j - 5].x +
                            laserClouds[i]->points[j - 4].x +
                            laserClouds[i]->points[j - 3].x +
                            laserClouds[i]->points[j - 2].x +
                            laserClouds[i]->points[j - 1].x -
                            10 * laserClouds[i]->points[j].x +
                            laserClouds[i]->points[j + 1].x +
                            laserClouds[i]->points[j + 2].x +
                            laserClouds[i]->points[j + 3].x +
                            laserClouds[i]->points[j + 4].x +
                            laserClouds[i]->points[j + 5].x;

            double y_diff = laserClouds[i]->points[j - 5].y +
                            laserClouds[i]->points[j - 4].y +
                            laserClouds[i]->points[j - 3].y +
                            laserClouds[i]->points[j - 2].y +
                            laserClouds[i]->points[j - 1].y -
                            10 * laserClouds[i]->points[j].y +
                            laserClouds[i]->points[j + 1].y +
                            laserClouds[i]->points[j + 2].y +
                            laserClouds[i]->points[j + 3].y +
                            laserClouds[i]->points[j + 4].y +
                            laserClouds[i]->points[j + 5].y;

            double z_diff = laserClouds[i]->points[j - 5].z +
                            laserClouds[i]->points[j - 4].z +
                            laserClouds[i]->points[j - 3].z +
                            laserClouds[i]->points[j - 2].z +
                            laserClouds[i]->points[j - 1].z -
                            10 * laserClouds[i]->points[j].z +
                            laserClouds[i]->points[j + 1].z +
                            laserClouds[i]->points[j + 2].z +
                            laserClouds[i]->points[j + 3].z +
                            laserClouds[i]->points[j + 4].z +
                            laserClouds[i]->points[j + 5].z;

            //std::cerr << "X diff: " << x_diff << " Y diff: " << y_diff << " Z diff: " << z_diff << std::endl;

            Double2d distance(j, pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2)); 

            if (distance.value > min_distance and distance.value < max_distance) {
                cloudDifferences.push_back(distance);
                min_distance = pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2);
                // std::cerr << "Min distance: " << min_distance << std::endl;
            }
        }

        //std::chrono::time_point<std::chrono::system_clock> start, end;
        //start = std::chrono::system_clock::now();
        featureExtractionFromSector(laserClouds[i], cloudDifferences, pc_out_edge, pc_out_surf, min_distance);
        //end = std::chrono::system_clock::now();
        //std::chrono::duration<float> elapsed_seconds = end - start;
        //total_frame++;

        //float time_tmp = elapsed_seconds.count() * 1000;
        //total_time += time_tmp;

        //if(total_frame % 100 == 0)
            //ROS_INFO("Average feature extraction from sector time %f ms \n \n", total_time/total_frame);
    }


    //remove ground point
    /*
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for(int i=0;i<pc_out_edge->points.size();i++){
        if(pc_out_edge->points[i].z<=-0.55)
            inliers->indices.push_back(i);
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(pc_out_edge);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*pc_out_edge);
    */
}

void LaserProcessingClass::featureExtractionFromSector(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in,
    std::vector<Double2d>& cloudDifferences,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_edge,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out_surf,
    double min_distance
) {
    sort(cloudDifferences.begin(), cloudDifferences.end(), [](const Double2d& a, const Double2d& b){
        return a.value < b.value; 
    });

    int largest_picked_points_number = 0;
    vector<int> picked_points;

    for (int i = cloudDifferences.size() - 1; i >= 0; i--) {
        int diff_id = cloudDifferences[i].id;

        if (find(picked_points.begin(), picked_points.end(), diff_id) == picked_points.end()) {
            largest_picked_points_number++;
            picked_points.push_back(diff_id);

            pc_out_surf->push_back(pc_in->points[diff_id]);
            
            if (largest_picked_points_number <= 10) {
                pc_out_edge->push_back(pc_in->points[diff_id]);
            } else {
                break;
            }

            for (int k = 1; k <= 5; k++) {
                double x_diff = pc_in->points[diff_id + k].x - pc_in->points[diff_id + k - 1].x;
                double y_diff = pc_in->points[diff_id + k].y - pc_in->points[diff_id + k - 1].y;
                double z_diff = pc_in->points[diff_id + k].z - pc_in->points[diff_id + k - 1].z;

                if (pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2) > min_distance) {
                    break;
                }

                picked_points.push_back(diff_id + k);
            }

            for (int k = -1; k >= -5; k--) {
                double x_diff = pc_in->points[diff_id + k].x - pc_in->points[diff_id + k + 1].x;
                double y_diff = pc_in->points[diff_id + k].y - pc_in->points[diff_id + k + 1].y;
                double z_diff = pc_in->points[diff_id + k].z - pc_in->points[diff_id + k + 1].z;

                if (pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2) > min_distance) {
                    break;
                }

                picked_points.push_back(diff_id + k);
            }
        }
    }
}


LaserProcessingClass::LaserProcessingClass(){
    
}

Double2d::Double2d(int id_in, double value_in){
    id = id_in;
    value =value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in){
    layer = layer_in;
    time = time_in;
};
