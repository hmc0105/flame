#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <pcl/common/projection_matrix.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <unistd.h>

#define filename "result.pcd"
#define grid 0.2

int main(int argc, char** argv)
{
  using PointT=pcl::PointXYZI;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr vectormap(new pcl::PointCloud<PointT>);
  PointT min_pt,max_pt;
  pcl::io::loadPCDFile<PointT> (filename,*cloud);
  pcl::getMinMax3D(*cloud,min_pt,max_pt);
  std::cout<<min_pt<<max_pt<<std::endl;
  int row_number1=(int)((max_pt.x-min_pt.x)/grid+1);
  int column_number1=(int)((max_pt.y-min_pt.y)/grid+1);
  int total_number=row_number1*column_number1;
  int vector_num=0;
  vectormap->width=1;
  vectormap->points.resize(total_number);
  vectormap->height=vectormap->points.size();
  
  for (int i=0;i<total_number;i++)
  {
    vectormap->points[i].z=0;
    vectormap->points[i].x=(i%row_number1)*(grid)+min_pt.x+0.5*grid;
    vectormap->points[i].y=(i/row_number1)*(grid)+min_pt.y+0.5*grid;
  }
  
  short hola[total_number]={0};
  for(pcl::PointCloud<PointT>::iterator it=cloud->begin();it!=cloud->end();it++)
  {
    int k=(int)((it->x-min_pt.x)/grid)+(int)(((it->y-min_pt.y)/grid))*row_number1;
    //if(k<1000) std::cout<<"YOU FUCKED"<<std::endl;
    vectormap->points[k].z++;
    //std::cout<<it->x<<"x"<<vectormap->points[k].x<<std::endl;
    //std::cout<<it->y<<"y"<<vectormap->points[k].y<<std::endl;
    if (k<0&&k>total_number) std::cout<<"INDEX OVERFLOW"<<std::endl;
  }

  /*
  for (int i=0;i<total_number;i++)
  {
    vectormap->points[i].z=hola[i];
    vectormap->points[i].x=(i%row_number1)*(grid)+min_pt.x+0.5*grid;
    vectormap->points[i].y=(i/row_number1)*(grid)+min_pt.y+0.5*grid;
  }
  */
  
  pcl::PointCloud<PointT>::iterator it = vectormap->begin();
  int k=0;
  while(it!=vectormap->end())
  {
    //  if(k%1000==0) std::cout<<k<<std::endl;
    if(it->z<1) {
      it=vectormap->erase(it);
    }
    else{
      it++;
    }
    k++;
    
  }
  vectormap->width=1;
  vectormap->height=vectormap->points.size();
  pcl::io::savePCDFile<PointT>("Fucking_result.pcd",*vectormap);
  pcl::getMinMax3D(*vectormap,min_pt,max_pt);
  std::cout<<min_pt<<max_pt<<std::endl;
  return 0;

}