/*
* Copyright (C) 2016, Australian Centre for Robotic Vision, ACRV
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Osnabrück University nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "segmentation.h"
using namespace APC;

Segmentation::Segmentation(){}

void Segmentation::doSegmentation(){

  // Start the clock
  clock_t sv_start = clock();

  // Preparation of Input: Supervoxel Oversegmentation
  pcl::SupervoxelClustering<PointT> super (config_.voxel_resolution, config_.seed_resolution);
  super.setUseSingleCameraTransform (config_.use_single_cam_transform);
  super.setInputCloud (input_cloud_ptr_);
  super.setColorImportance (config_.color_importance);
  super.setSpatialImportance (config_.spatial_importance);
  super.setNormalImportance (config_.normal_importance);
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

  super.extract (supervoxel_clusters);

  if (config_.use_supervoxel_refinement)
  {
    PCL_INFO ("Refining supervoxels\n");
    super.refineSupervoxels (2, supervoxel_clusters);
  }
  std::cout << "Number of supervoxels: " << supervoxel_clusters.size () << "\n";

  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);
  clock_t sv_end = clock();
  printf("Super-voxel segmentation takes: %.2fms\n", (double)(sv_end - sv_start)/(CLOCKS_PER_SEC/1000));

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Constrained plane extraction
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* Generate plane hypotheses using super-voxels */
  std::vector<Eigen::Vector4f> planes_coeffs;
  std::vector<Eigen::Vector3f> planes_hough;
  double min_theta = 360; double max_theta = -360;
  double min_phi = 360; double max_phi = -360;
  double min_rho = 100; double max_rho = -100;

  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator cluster_itr_c = supervoxel_clusters.begin();
  for (; cluster_itr_c != supervoxel_clusters.end(); cluster_itr_c++){
    pcl::Supervoxel<PointT>::Ptr sv = cluster_itr_c->second;
    pcl::PointCloud<PointT>::Ptr cloud = sv->voxels_;
    float curvature;
    Eigen::Vector4f plane_par;
    Eigen::Vector3f hough_par;
    pcl::computePointNormal(*cloud, plane_par, curvature);
    if (curvature < config_.max_curvature){
      // Convert to Hough transform
      double theta = std::atan(plane_par(1)/plane_par(0))*180/M_PI;
      double phi = std::acos(plane_par(2))*180/M_PI;
      double rho = plane_par(3);
      if (std::isnan(theta) | std::isnan(phi) | std::isnan(rho)) continue;
      hough_par(0) = theta;
      hough_par(1) = phi;
      hough_par(2) = rho;
      if (theta < min_theta) min_theta = theta;
      if (theta > max_theta) max_theta = theta;
      if (phi < min_phi) min_phi = phi;
      if (phi > max_phi) max_phi = phi;
      if (rho < min_rho) min_rho = rho;
      if (rho > max_rho) max_rho = rho;
      planes_hough.push_back(hough_par);
      planes_coeffs.push_back(plane_par);
    }
  }

  // Plane hypothesis generation using random sampling
  if (config_.use_random_sampling){
    std::cout << "Randomly sampling plane hypotheses...\n";
    int max_random_hyps = 1000;
    int count = 0;
    int num_supervoxels = supervoxel_clusters.size ();
    srand(time(NULL));
    pcl::SampleConsensusModelPlane<pcl::PointNormal>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointNormal> (sv_centroid_normal_cloud));
    while (count < max_random_hyps){
      // random sample 4 points
      std::vector<int> samples;
      int iters;
      model_p->getSamples(iters, samples);
      Eigen::VectorXf plane_par;
      model_p->computeModelCoefficients(samples, plane_par);

      std::set<int> test_points;
      test_points.insert((int)(rand()%num_supervoxels+1));
      bool good_model = model_p->doSamplesVerifyModel(test_points, plane_par, config_.noise_threshold*0.5);
      if (good_model == false) continue;

      Eigen::Vector3f hough_par;
      double theta = std::atan(plane_par(1)/plane_par(0))*180/M_PI;
      double phi = std::acos(plane_par(2))*180/M_PI;
      double rho = plane_par(3);

      if (std::isnan(theta) | std::isnan(phi) | std::isnan(rho)) continue;

      planes_coeffs.push_back(plane_par);
      hough_par(0) = theta;
      hough_par(1) = phi;
      hough_par(2) = rho;

      if (theta < min_theta) min_theta = theta;
      if (theta > max_theta) max_theta = theta;
      if (phi < min_phi) min_phi = phi;
      if (phi > max_phi) max_phi = phi;
      if (rho < min_rho) min_rho = rho;
      if (rho > max_rho) max_rho = rho;
      planes_hough.push_back(hough_par);
      count++;
    }
  }

  // Assign points to planes.
  uint32_t node_ID = 0;
  std::map<uint32_t, uint32_t> label2index;
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator cluster_itr = supervoxel_clusters.begin();
  for (; cluster_itr != supervoxel_clusters.end(); cluster_itr++){
    label2index[cluster_itr->first] = node_ID;
    node_ID++;
  }
  uint32_t num_super_voxels = sv_centroid_normal_cloud->size();
  std::vector<int> sv_labels(num_super_voxels,0);
  int good_planes_count = 0;
  uint32_t outlier_label = 0;
  // Remove duplicated planes
  if (planes_coeffs.size() > 0){
    std::vector<Eigen::Vector4f> plane_candidates;
    double step_theta = 1;
    double step_phi = 1;
    double step_rho = 0.025;
    int theta_bins = round((max_theta - min_theta)/step_theta) + 1;
    int phi_bins = round((max_phi - min_phi)/step_phi) + 1;
    int rho_bins = round((max_rho - min_rho)/step_rho) + 1;

    unsigned char*** accumulator;
    accumulator = new unsigned char**[theta_bins];
    if (accumulator != NULL){
      for (int i=0; i<theta_bins; i++){
        accumulator[i] = new unsigned char*[phi_bins];
        if (accumulator[i] != NULL)
        for (int j=0; j<phi_bins; j++)
        accumulator[i][j] = new unsigned char[rho_bins];
      }
    }

    for (int i=0; i<planes_coeffs.size(); i++){
      Eigen::Vector3f hough_par = planes_hough.at(i);
      int b_theta = floor((hough_par(0) - min_theta + 0.0001)/step_theta);
      int b_phi = floor((hough_par(1) - min_phi + 0.0001)/step_phi);
      int b_rho = floor((hough_par(2) - min_rho + 0.0001)/step_rho);

      if (accumulator[b_theta][b_phi][b_rho] != 1){
        accumulator[b_theta][b_phi][b_rho] = 1;
        plane_candidates.push_back(planes_coeffs.at(i));
      }
    }
    // Free accumulator memory
    for (int i = 0; i < theta_bins; i++){
      for (int j = 0; j < phi_bins; j++){
        delete [] accumulator[i][j];
      }
      delete [] accumulator[i];
    }
    delete [] accumulator;
    std::cout << "Number of planes remained after hough-based filtering = " << plane_candidates.size() << "\n";
    // Compute plane unary costs
    float min_num_supervoxel_per_plane = config_.min_plane_area/(config_.seed_resolution*config_.seed_resolution/4/M_PI);
    std::vector<Eigen::Vector4f> good_planes;
    std::vector<Eigen::VectorXi> planes_inliers_idx;
    std::vector<float> unaries;
    uint32_t num_planes = plane_candidates.size();
    Eigen::MatrixXi inliers_mat(num_planes, num_super_voxels);
    Eigen::MatrixXf normals_mat(num_planes, num_super_voxels);
    Eigen::MatrixXf point2plane_mat(num_planes, num_super_voxels);
    int count_idx = 0;
    for (int j = 0; j<num_planes; ++j){
      Eigen::Vector4f p_coeffs = plane_candidates.at(j);

      Eigen::Vector3f p_normal;
      p_normal[0] = p_coeffs[0];
      p_normal[1] = p_coeffs[1];
      p_normal[2] = p_coeffs[2];
      Eigen::VectorXi inliers_idx(num_super_voxels);
      Eigen::VectorXf point2plane(num_super_voxels);
      inliers_idx = Eigen::VectorXi::Zero(num_super_voxels);
      int inliers_count = 0;
      float plane_score = 0;
      for (size_t i = 0; i < num_super_voxels; ++i){
        pcl::PointXYZ p;
        Eigen::Vector3f n;
        p.x = sv_centroid_normal_cloud->at(i).x;
        p.y = sv_centroid_normal_cloud->at(i).y;
        p.z = sv_centroid_normal_cloud->at(i).z;
        n[0] = sv_centroid_normal_cloud->at(i).normal_x;
        n[1] = sv_centroid_normal_cloud->at(i).normal_y;
        n[2] = sv_centroid_normal_cloud->at(i).normal_z;

        // Distance from a point to a plane is scaled with a weight measuring the difference between point and plane normals.
        float p2p_dis = pcl::pointToPlaneDistance(p,p_coeffs);
        if (std::isnan(p2p_dis)) p2p_dis = config_.noise_threshold;
        float dotprod = std::fabs(n.dot(p_normal));
        if (std::isnan(dotprod)) dotprod = 0;
        float normal_dis = dotprod < 0.8 ? 100 : 1;
        float data_cost = p2p_dis*normal_dis;
        plane_score += -std::exp(-data_cost/(2*config_.noise_threshold)); // -1 is best, 0 is worst
        point2plane(i) = data_cost;
        if (data_cost <= config_.noise_threshold) {
          inliers_idx(i) = 1;
          inliers_count++;
        }
      }
      plane_score = plane_score/(min_num_supervoxel_per_plane*10);
      float confidence_threshold = -0.1f;
      if (plane_score <= confidence_threshold){
        inliers_mat.row(count_idx) = inliers_idx;
        normals_mat.row(count_idx) << p_coeffs(0), p_coeffs(1), p_coeffs(2);
        point2plane_mat.row(count_idx) = point2plane;
        good_planes.push_back(p_coeffs);
        planes_inliers_idx.push_back(inliers_idx);
        double u_cost = plane_score;
        unaries.push_back(u_cost);
        count_idx++;
      }
    }
    clock_t plane_sampling_end = clock();
    printf("Plane generation takes: %.2fms\n", (double)(plane_sampling_end - sv_end)/(CLOCKS_PER_SEC/1000));
    num_planes = unaries.size();
    std::cout << "Number of plane candidates = " << num_planes << "\n";
    if (num_planes > 1){

      inliers_mat.conservativeResize(num_planes, num_super_voxels);
      normals_mat.conservativeResize(num_planes, num_super_voxels);
      point2plane_mat.conservativeResize(num_planes, num_super_voxels);

      Eigen::VectorXi inliers_count = inliers_mat.rowwise().sum();
      //Eigen::MatrixXi ov_mat = inliers_mat*inliers_mat.transpose();
      //Eigen::MatrixXf dot_mat = normals_mat*normals_mat.transpose();

      Eigen::MatrixXi temp1 = inliers_mat.transpose();
      Eigen::MatrixXi ov_mat = inliers_mat*temp1;
      Eigen::MatrixXf temp2 = normals_mat.transpose();
      Eigen::MatrixXf dot_mat = normals_mat*temp2;

      Eigen::VectorXf plane_unaries = Eigen::Map<Eigen::MatrixXf>(unaries.data(), num_planes, 1);
      Eigen::MatrixXf plane_pairwises = Eigen::MatrixXf::Zero(num_planes, num_planes);

      for (int i=0; i<num_planes-1; i++){
        for (int j=i+1; j<num_planes; j++){
          double ov_cost = (double)ov_mat(i,j)/std::min(inliers_count(i), inliers_count(j));
          //if (ov_cost > 0.75) ov_cost = 1.0;
          //else ov_cost = 0;
          double dot_prod = std::abs(dot_mat(i,j));
          dot_prod = dot_prod < 0.5 ? dot_prod : 1 - dot_prod;
          double angle_cost = 1 - exp(-dot_prod/0.25);
          double p_cost;
          //if (ov_cost == 0) p_cost = 0; // TODO: If the two planes do not intersect, we do not constraint them!
          //else p_cost = angle_cost; ;//0.5*angle_cost + 0.5*ov_cost;
          p_cost = angle_cost;
          plane_pairwises(i,j) = p_cost*0.5;
          plane_pairwises(j,i) = p_cost*0.5;
        }
      }

      Eigen::VectorXi initLabeling(num_planes);
      initLabeling = Eigen::VectorXi::Ones(num_planes);
      Eigen::VectorXi finalLabeling(num_planes);
      double finalEnergy = 0;
      LSA_TR(&finalEnergy, &finalLabeling, num_planes, plane_unaries, plane_pairwises, initLabeling);
      if (finalEnergy == 0){
        PCL_WARN("Optimization got stuck \n");
        finalLabeling = Eigen::VectorXi::Ones(num_planes);
      }
      int num_selected_planes = finalLabeling.sum();
      std::cout << "Number of supporting planes detected = " << num_selected_planes << "\t";
      std::cout << "(Note: This is not the true number of planes in the scene.)\n";
      std::vector<Eigen::Vector4f> selected_planes;
      Eigen::MatrixXf unary_matrix(num_selected_planes + 1, num_super_voxels);
      int sidx = 1;
      for (int i=0; i<num_planes; i++){
        if (finalLabeling(i) == 1) {
          selected_planes.push_back(good_planes.at(i));
          unary_matrix.row(sidx) = point2plane_mat.row(i)*config_.gc_scale;
          sidx++;
        }
      }

      // Outlier data cost
      int num_labels = num_selected_planes + 1;
      outlier_label = 0;
      unary_matrix.row(outlier_label) = Eigen::VectorXf::Ones(num_super_voxels)*((config_.noise_threshold*config_.gc_scale));
      Eigen::MatrixXi unary_matrix_int = unary_matrix.cast<int>();
      int *data_cost = new int[num_super_voxels*num_labels];
      Eigen::Map<Eigen::MatrixXi>(data_cost, unary_matrix_int.rows(), unary_matrix_int.cols() ) = unary_matrix_int;
      GCoptimizationGeneralGraph *gc = new GCoptimizationGeneralGraph(num_super_voxels, num_labels);
      gc->setDataCost(data_cost);

      for ( int l1 = 0; l1 < num_labels; l1++){
        for (int l2 = 0; l2 < num_labels; l2++){
          if (l1==0) gc->setSmoothCost(l1,l2,0);
          if (l1==l2) gc->setSmoothCost(l1,l2,0);
          else {
            gc->setSmoothCost(l1,l2,1);
          }
        }
      }

      std::multimap<uint32_t,uint32_t>::iterator adjacency_itr = supervoxel_adjacency.begin();
      float smooth_cost = config_.noise_threshold/2;
      for ( ; adjacency_itr != supervoxel_adjacency.end(); ++adjacency_itr)
      {
        uint32_t node1 = label2index[adjacency_itr->first];
        uint32_t node2 = label2index[adjacency_itr->second];
        Eigen::Vector3f n1;
        n1[0] = sv_centroid_normal_cloud->at(node1).normal_x;
        n1[1] = sv_centroid_normal_cloud->at(node1).normal_y;
        n1[2] = sv_centroid_normal_cloud->at(node1).normal_z;
        Eigen::Vector3f n2;
        n2[0] = sv_centroid_normal_cloud->at(node2).normal_x;
        n2[1] = sv_centroid_normal_cloud->at(node2).normal_y;
        n2[2] = sv_centroid_normal_cloud->at(node2).normal_z;
        float w = std::fabs(n1.dot(n2));
        if (w < 0.5) w = 0.0f;
        int edge_weight = (int)(w * config_.gc_scale * smooth_cost); // This works better than Potts smoothness model
        gc->setNeighbors(node1,node2,edge_weight);
      }
      try{
        gc->expansion(config_.max_num_iterations);
      }catch(GCException e){
        e.Report();
      }

      for (int i=0; i<num_super_voxels;i++){
        sv_labels.at(i) = gc->whatLabel(i);
      }
      // Free some memory
      delete gc;
      delete data_cost;
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////
      // End of constrained plane extraction
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    clock_t plane_fitting_end = clock();
    printf("Global plane extraction takes: %.2fms\n", (double)(plane_fitting_end - plane_sampling_end)/(CLOCKS_PER_SEC/1000));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Segment the point cloud into objects
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::cout << "Run graph based object segmentation using the extracted planes\n";
  using namespace boost;
  std::vector<uint32_t> supervoxel_labels;
  {
    typedef adjacency_list <vecS, vecS, undirectedS> Graph;

    Graph G;
    for (uint32_t i = 0; i < supervoxel_clusters.size(); ++i){
      add_vertex(G);
    }

    std::multimap<uint32_t,uint32_t>::iterator adjacency_itr = supervoxel_adjacency.begin();
    for ( ; adjacency_itr != supervoxel_adjacency.end(); ++adjacency_itr)
    {
      uint32_t from = label2index[adjacency_itr->first];
      uint32_t to = label2index[adjacency_itr->second];

      uint32_t label_from = sv_labels.at(from);
      uint32_t label_to = sv_labels.at(to);

      Eigen::Vector3f p1;
      p1[0] = sv_centroid_normal_cloud->at(from).x;
      p1[1] = sv_centroid_normal_cloud->at(from).y;
      p1[2] = sv_centroid_normal_cloud->at(from).z;

      Eigen::Vector3f n1;
      n1[0] = sv_centroid_normal_cloud->at(from).normal_x;
      n1[1] = sv_centroid_normal_cloud->at(from).normal_y;
      n1[2] = sv_centroid_normal_cloud->at(from).normal_z;

      Eigen::Vector3f p2;
      p2[0] = sv_centroid_normal_cloud->at(to).x;
      p2[1] = sv_centroid_normal_cloud->at(to).y;
      p2[2] = sv_centroid_normal_cloud->at(to).z;

      Eigen::Vector3f n2;
      n2[0] = sv_centroid_normal_cloud->at(to).normal_x;
      n2[1] = sv_centroid_normal_cloud->at(to).normal_y;
      n2[2] = sv_centroid_normal_cloud->at(to).normal_z;

      if (label_from != label_to) continue;
      if (label_from == label_to && label_from != outlier_label){
        add_edge(from,to,G);
        continue;
      }
      bool convex = isConvex(p1, n1, p2, n2, config_.seed_resolution, config_.voxel_resolution);
      if (convex == true) add_edge(from,to,G);
      //sv_centroid_normal_cloud
    }

    std::vector<uint32_t> component(num_vertices(G));
    uint32_t num = connected_components(G, &component[0]);
    std::cout << "Number of connected components: " << num <<"\n";

    int min_voxels_per_cluster = 2;
    int outlier_label = 0;
    std::map<uint32_t,uint32_t> label_list_map;
    int new_label = 1;
    for (uint32_t i = 0; i != component.size(); ++i){
      int count = std::count(component.begin(), component.end(), component[i]);
      int label = component[i];
      if (label_list_map.find(label) == label_list_map.end() && count >= min_voxels_per_cluster){ // label not found
        label_list_map[label] = new_label;
        new_label++;
      }
      if (count < min_voxels_per_cluster){ // minimum number of supervoxels in each component
        supervoxel_labels.push_back(outlier_label);
      }
      else
      supervoxel_labels.push_back(label_list_map.find(label)->second);
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // End of scene segmentation
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Re-label the point cloud
  segmented_cloud_ptr_ = super.getLabeledCloud ();
  std::map<uint32_t,uint32_t> label_to_seg_map;
  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr>::iterator cluster_itr_ = supervoxel_clusters.begin();
  uint32_t idx = 0;
  for (; cluster_itr_ != supervoxel_clusters.end(); cluster_itr_++){
    //label_to_seg_map[cluster_itr_->first] = sv_labels.at(idx); // Use this to plot plane segmentation only
    label_to_seg_map[cluster_itr_->first] = supervoxel_labels.at(idx);
    idx++;
  }
  typename pcl::PointCloud<pcl::PointXYZL>::iterator point_itr = (*segmented_cloud_ptr_).begin();
  uint32_t zero_label = 0;
    uint32_t count = 0;
  for (; point_itr != (*segmented_cloud_ptr_).end(); ++point_itr)
  {
    if (point_itr->label == 0){
      zero_label++;
      count++;

    }else{
      point_itr->label = label_to_seg_map[point_itr->label];
      count++;
    }

  }
  cout <<count<<endl;


  printf("All Time taken: %.2fms\n", (double)(clock() - sv_start)/(CLOCKS_PER_SEC/1000));


}
/// END main
