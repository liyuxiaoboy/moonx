//
// Created by yxli on 18-11-9.
//

#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

const double camera_factor=1000;
const double camera_cx=325.5;
const double camera_cy=253.5;
const double camera_fx=518.0;
const double camera_fy=519.0;

int main(int argc, char** argv)
{
    cv::Mat rgb,depth;
    rgb=cv::imread("/home/yxli/roswork/Experimental_material/test/rgb/1538104680279619.png");
    depth=cv::imread("/home/yxli/roswork/Experimental_material/test/depth/1538104680317713.png",-1);
    int cont=0;
    cout<<"depth "<<depth.rows<<" "<<depth.cols<<endl;
    PointCloud::Ptr cloud(new PointCloud);
    for(int m=0;m<depth.rows;++m)
        { for(int n=0;n<depth.cols;++n)
            {
                ushort d=depth.ptr<ushort>(m)[n];
                if(d==0)
                {
                continue;
                }
                PointT p;

                p.z=double(d)/camera_factor;
                p.x=(n-camera_cx)/camera_fx*p.z;
                p.y=-(m-camera_cy)/camera_fy*p.z;

                p.b=rgb.ptr<uchar>(m)[n*3];
                p.g=rgb.ptr<uchar>(m)[n*3+1];
                p.r=rgb.ptr<uchar>(m)[n*3+2];

                cloud->points.push_back(p);
                cont++;
            }
        }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"cont "<<cont<<endl;
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "./testpointcloud.pcd", *cloud );
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}
