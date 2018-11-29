//
// Created by root on 18-4-3.
//

#ifndef ORB_SLAM2_SEGMENTATION_H
#define ORB_SLAM2_SEGMENTATION_H

// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>


// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// Segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

// Graph cuts
#include "gco-v3.0/GCoptimization.h"

// LSA TR Optimisation
#include "lsa_tr.h"

#include "segmentation_helpers.h"

// Boost
#include <boost/format.hpp>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

// Eigen
#include <Eigen/Core>


#include <config.h>

# define M_PI 3.14159265358979323846  /* pi */

/// *****  Type Definitions ***** ///
typedef pcl::PointXYZRGBA PointT;  // The point type used for input


namespace APC {


    class Segmentation{

    public:

        /**
        * @brief Segmentation
        */
        Segmentation();

        /**
        * @brief setConfig
        * @param config
        */
        void setConfig(const Config& config){config_=config;}

        /**
        * @brief getConfig
        * @return
        */
        Config getConfig(){return config_;}

        /**
        * @brief setPointCloud
        * @param input_cloud_ptr
        */
        void setPointCloud(pcl::PointCloud<PointT>::Ptr input_cloud_ptr){input_cloud_ptr_=input_cloud_ptr;}

        /**
        * @brief doSegmentation Main function for segmentation
        */
        void doSegmentation();

        /**
        * @brief segmentedPointCloud
        * @return the segmented point cloud
        */
        pcl::PointCloud<pcl::PointXYZL>::Ptr getSegmentedPointCloud(){return segmented_cloud_ptr_;}


    private:

        Config config_;

        pcl::PointCloud<PointT>::Ptr input_cloud_ptr_;
        pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud_ptr_;


    };

}

#endif //ORB_SLAM2_SEGMENTATION_H
