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

#define filename "munji_1024.pcd"

using PointT=pcl::PointXYZI;
/*
struct cell
{
  pcl::PointCloud<PointT> cloud;
  int point_num=0;
} ;*/

class vectormap 
{
private:
  PointT min_pt,max_pt;
  int row_number, column_number;
  int cloud_rows, cloud_columns;
  int thrown;
public:
  vectormap(pcl::PointCloud<PointT>& cloud,int vectormapbox[],float grid);
};

vectormap::vectormap(pcl::PointCloud<PointT>& cloud,int vectormapbox[] ,float grid)
{
  std::cout<<"Making Vector Map Started"<<std::endl;
  cloud_rows=cloud.width;
  cloud_columns=cloud.height;
  pcl::getMinMax3D(cloud, min_pt, max_pt);
  row_number=(int)((max_pt.x-min_pt.x)/grid);
  column_number=(int)((max_pt.y-min_pt.y)/grid);
  /*
  for (int i=0;i<row_number;i++)
  {
    for (int j=0;j<column_number;j++)
    {
      vectormapbox[i*column_number+j].cloud(new pcl::PointCloud<PointT>);
    }
  }
  */
  for (int i=0;i<cloud.size();i++)
  {

      int rowidx=(cloud.points[i].x-min_pt.x)/grid;
      int colidx=(cloud.points[i].y-min_pt.y)/grid;
      if (((rowidx<0)&&(rowidx>this->row_number))&&((colidx<0)&&(colidx>this->column_number))) 
      {
        thrown++;
      }
      else
      {
        //vectormapbox[(rowidx*column_number+colidx)].cloud.push_back(copy);
        std::cout<<rowidx*column_number+colidx<<std::endl;
        vectormapbox[(rowidx*column_number+colidx)]++;

      }
    
  }
  std::cout<<99<<std::endl;
  std::cout<<thrown<<std::endl;
}



int main(int argc, char** argv)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  PointT min_pt,max_pt;
  pcl::io::loadPCDFile<PointT> ("munji_1024.pcd",*cloud);
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  int row_number1=(int)((max_pt.x-min_pt.x)/1);
  int column_number1=(int)((max_pt.y-min_pt.y)/1);
  std::cout<<row_number1<<"%"<<column_number1<<std::endl;
  int vectormapbox[(row_number1*column_number1)]={0};
  vectormap(*cloud,vectormapbox ,1);
  
  sleep(2);
  for(int i=0;i<row_number1*column_number1;i++)
  {
    std::cout<<vectormapbox[i]<<std::endl;
  }


  return(0);
}

