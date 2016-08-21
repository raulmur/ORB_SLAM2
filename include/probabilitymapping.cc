/*
 * =====================================================================================
 *
 *       Filename:  ProbabilityMapping.cc
 *
 *    Description:
 *
 *        Version:  0.1
 *        Created:  01/21/2016 10:39:12 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Josh Tang, Rebecca Frederick
 *   Organization:  Unkei
 *
 * =====================================================================================
 */

#include <cmath>
#include <opencv2/opencv.hpp>
#include <numeric>
#include "ProbabilityMapping.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "LocalMapping.h"
#include <stdint.h>
#include <stdio.h>

#define DEBUG 1
#define DBG(do_something) if (DEBUG) { do_something; }
#define SAVE_IMAGES 1
#define SAVE(do_something) if (SAVE_IMAGES) { do_something }

ProbabilityMapping::ProbabilityMapping() {}

void ProbabilityMapping::FirstLoop(ORB_SLAM::KeyFrame *kf, std::vector<std::vector<depthHo> > &ho){
  DBG(cout << "Enter the FirstLoop\n")

  if(kf->isBad())
    exit(0);
  std::vector<ORB_SLAM::KeyFrame*> closestMatches = kf->GetBestCovisibilityKeyFrames(covisN);

  DBG(cout << "Find Stereo Search Constraints\n")
  float max_depth;
  float min_depth;

  StereoSearchConstraints(kf, &min_depth, &max_depth);
  DBG(cout << "Found! min:" << min_depth << "  max:" << max_depth << "\n";
      cout << "Getting Image Gradient\n")

  cv::Mat gradx, grady, gradmag, gradth, really;
  cv::Mat image = kf->GetImage();
  SAVE(imwrite("/home/josh/Workspace/Scanner3D/mainImage.png",image);)
  GetGradientMagAndOri(image, &gradx, &grady, &gradmag, &gradth, &really);
  DBG(cout << "Got it!\n";
      cout << "Generating Depth Hypotheses...\n")

  std::vector<depthHo> depth_ho;

  std::vector<std::vector<depthHo> > temp_ho (image.rows, std::vector<depthHo>(image.cols, depthHo()) );
  depth_ho.clear();
  DBG(cout << "Closest Matches size:"<< closestMatches.size() << "\n")

  for(size_t i=0; i<closestMatches.size(); i++){
    ORB_SLAM::KeyFrame* kf2 = closestMatches[i];
    SAVE(imwrite("/home/josh/Workspace/Scanner3D/math"+ boost::lexical_cast<std::string>(i) +".png",image);)
    for(int x = 0; x < image.rows; x++){
      for(int y = 0; y < image.cols; y++){
        DBG(cout << "Pixel gradient magnitude: " << gradmag.at<float>(x,y) << "  lambdaG: " << lambdaG <<"\n")
        if(gradmag.at<float>(x,y) < lambdaG){
          continue;
        }

        depthHo dh;
        float pixel = image.at<uchar>(x,y); //maybe it should be cv::Mat
        EpipolarSearch(kf, kf2, x, y, pixel, gradmag, min_depth, max_depth, &dh);
        DBG(cout << "Depth: " << dh.depth << "\n")
        if (dh.supported)
            depth_ho.push_back(dh);

        DBG(printf("FirstLoop: found a set of %d hypotheseses for pixel %d,%d\n", (int)(depth_ho.size()), x, y))
        if (depth_ho.size()) {
          depthHo dh;
          DBG(cout << "Calculating Inverse Depth Hypothesis\n")
          InverseDepthHypothesisFusion(depth_ho, &dh);
          dh.supported = true;
          temp_ho[x][y] = dh;
        } else {
            temp_ho[x][y].supported = false;
        }
      }
    }
  }
  ho = temp_ho;
}

void ProbabilityMapping::StereoSearchConstraints(ORB_SLAM::KeyFrame* kf, float* min_depth, float* max_depth){
  std::vector<float> orb_depths = kf->GetAllPointDepths(20);

  float sum = std::accumulate(orb_depths.begin(), orb_depths.end(), 0.0);
  float mean = sum / orb_depths.size();

  std::vector<float> diff(orb_depths.size());
  std::transform(orb_depths.begin(), orb_depths.end(), diff.begin(), std::bind2nd(std::minus<float>(), mean));
  float variance = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0)/orb_depths.size();
  float stdev = std::sqrt(variance);

  *max_depth = mean + 2 * stdev;
  *min_depth = mean - 2 * stdev;
}

void ProbabilityMapping::EpipolarSearch(ORB_SLAM::KeyFrame* kf1, ORB_SLAM::KeyFrame *kf2, int x, int y, float pixel, cv::Mat grad,
    float min_depth, float max_depth, depthHo *dh) {

  cv::Mat image = kf2->GetImage();
  cv::Mat image_stddev, image_mean;
  cv::meanStdDev(image,image_mean,image_stddev);

  cv::Mat F12 = ComputeFundamental(kf1,kf2);
  float a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
  float b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
  float c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
  DBG(cout << "Line Equation a:" << a << "  b:" << b << "  c:" << c << endl)
  float old_err = 1000.0;
  float best_photometric_err = 0.0;
  float best_gradient_modulo_err = 0.0;
  int best_pixel = 0;

  int vj;

  cv::Mat gradx2, grady2, grad2mag, grad2th, really2;
  DBG(cout << "Getting Image Gradient\n")
  GetGradientMagAndOri(image, &gradx2, &grady2, &grad2mag, &grad2th, &really2);
  DBG(cout << "Getting Gradient Orientation\n")
  for(int x1 = 0; x1 < image.cols; x1++){
    for(int y1 = 0; y1 < image.rows; y1++){
      DBG(cout << "x: " << x1 << endl)
      DBG(cout << "y: " << y1 << endl)
      DBG(cout << "Gradient Modulo: " << grad2mag.at<float>(x1,y1) << endl)
      DBG(cout << "Gradient Orientation: " << grad2th.at<float>(x1,y1) << endl)
      if((grad2mag.at<float>(x1, y1) < 0)){
        DBG(cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n")
        exit(0);
      }
    }
  }
  for(int uj = 0; uj < image.cols; uj++){ // FIXME should use  min and max depth
    vj = (a/b)*uj+(c/b);
    DBG(cout << "uj: " << uj << endl;
        cout << "vj: " << vj << endl)
    if(!kf2 -> IsInImage(uj,vj)){
      DBG(cout << "not in image\n")
      continue;
    }
    DBG(cout << "IN LOOP\n";
        cout << "Pixel value2:" << static_cast<unsigned>(image.at<uchar>(uj,vj)) << endl;
        cout << "Grad2mag:" << grad2mag.at<float>(uj,vj) << endl)
        //cout << "manual magnitude:" << really2.at<float>(uj,vj) << endl)

    DBG(cout << "calculating epipolar line angle\n");
    float th_epipolar_line = cv::fastAtan2(uj,vj);
    DBG(cout << "theta epipolar line: " << th_epipolar_line << endl)
    DBG(cout << "grad2th: " << grad2th.at<float>(uj,vj) << endl)

    if(grad2mag.at<float>(uj,vj) < lambdaG){
      DBG(cout << "low gradient\n")
      continue;
    }

//FIXME ASAP
    if(abs(grad2th.at<float>(uj,vj) - th_epipolar_line + 180) < lambdaL){
      cout << "low angle\n";
      continue;
    }
    if(abs(grad2th.at<float>(uj,vj) - th_epipolar_line - 180) < lambdaL){
      cout << "high angle\n";
      continue;
    }
    //if(abs(th_grad - ( th_pi + th_rot )) < lambdaTheta)
      //continue;

    float photometric_err = pixel - image.at<uchar>(uj,vj); //FIXME properly calculate photometric error
    float gradient_modulo_err = grad.at<float>(uj,vj)  - grad2mag.at<float>(uj,vj);
    float err = (photometric_err*photometric_err + (gradient_modulo_err*gradient_modulo_err)/0.23)/(image_stddev.at<float>(uj,vj));

    DBG(cout << "testing error\n")
    if(abs(err) < abs(old_err)){
      best_pixel = uj;
      old_err = err;
      best_photometric_err = photometric_err;
      best_gradient_modulo_err = gradient_modulo_err;
    }
  }

  if(old_err >= 1000.0){
    DBG(cout << "no best pixel\n")
  }
  else{
    DBG(cout << "LEFT THE LOOP,should have a best pixel\n";
        cout << "best pixel x:" << best_pixel << endl;
        cout << "err: " << old_err << endl;
        cout << "photo metric error: " << best_photometric_err << endl)
    int best_vj = (a/b)*best_pixel + (c/b);
    int uj_plus = best_pixel + 1;
    int vj_plus = (a/b)*uj_plus + (c/b);
    int uj_minus = best_pixel - 1;
    int vj_minus = (a/b)*uj_minus + (c/b);

    float g = (image.at<uchar>(uj_plus, vj_plus) - image.at<uchar>(uj_minus, vj_minus))/2.0;

    float q = (grad2mag.at<float>(uj_plus, vj_plus) - grad2mag.at<float>(uj_minus, vj_plus))/2.0;

    float ustar = best_pixel + (g*best_photometric_err + (1/0.23)*q*best_gradient_modulo_err)/(g*g + (1/0.23)*q*q);
    float ustar_var = (2*image_stddev.at<float>(best_pixel,best_vj)*image_stddev.at<float>(best_pixel,best_vj)/(g*g + (1/0.23)*q*q));

    DBG(cout << "Computing Inverse Depth Hypothesis\n")
    ComputeInvDepthHypothesis(kf1, best_pixel, ustar, ustar_var, a, b, c, dh);
  }
}

void ProbabilityMapping::IntraKeyFrameDepthChecking(std::vector<std::vector<depthHo> >& ho, int imrows, int imcols) {

    std::vector<std::vector<depthHo> > ho_new;
    for (size_t i = 0; i < ho.size(); i++) {
        struct depthHo dhtemp;
        std::vector<depthHo> temp(ho[i].size(), dhtemp);
        ho_new.push_back(temp);
    }

    for (int px = 1; px < (imrows - 1); px++) {
        for (int py = 1; py < (imcols - 1); py++) {
            if (ho[px][py].supported == false) {
                // check if this pixel is surrounded by at least two pixels that are compatible to each other.
                std::vector<std::vector<depthHo> > compatible_neighbor_neighbor_ho;

                PixelNeighborNeighborSupport(ho, px, py, compatible_neighbor_neighbor_ho);

                unsigned int max_support = 0;
                unsigned int max_support_index = 0;
                for (size_t c = 0; c < compatible_neighbor_neighbor_ho.size(); c++) {
                    if (compatible_neighbor_neighbor_ho[c].size() > max_support) {
                        max_support = compatible_neighbor_neighbor_ho[c].size();
                        max_support_index = c;
                    }
                }

                // potentially grow the reconstruction density
                if (max_support >= 2) {
                    // assign this previous NULL depthHo the average depth and min sigma of its compatible neighbors
                    depthHo fusion;
                    float min_sigma;

                    GetFusion(compatible_neighbor_neighbor_ho[max_support_index], &fusion, &min_sigma);

                    ho_new[px][py].depth = fusion.depth;
                    ho_new[px][py].sigma = min_sigma;
                    ho_new[px][py].supported = true;
                }

            } else {
                // calculate the support of the pixel's  8 neighbors
                std::vector<depthHo> compatible_neighbor_ho;

                PixelNeighborSupport(ho, px, py, compatible_neighbor_ho);

                if (compatible_neighbor_ho.size() < 2) {
                    // average depth of the retained pixels
                    // set sigma to minimum of neighbor pixels
                    depthHo fusion;
                    float min_sigma = 0;

                    GetFusion(compatible_neighbor_ho, &fusion, &min_sigma);

                    ho_new[px][py].depth = fusion.depth;
                    ho_new[px][py].sigma = min_sigma;
                    ho_new[px][py].supported = true;

                } else {
                    ho_new[px][py] = ho[px][py];
                }
            }
        }
    }

    ho.assign(ho_new.begin(), ho_new.end());
    //for (int x = 0; x < imrows; x++) {
    //    for (int y = 0; y < imcols; y++) {
    //        ho[x][y] = ho_new[x][y];
    //    }
    //}
}

void ProbabilityMapping::InverseDepthHypothesisFusion(const std::vector<depthHo>& h, depthHo* dist) {
    dist->depth = 0;
    dist->sigma = 0;
    dist->supported = false;

    std::vector<depthHo> compatible_ho;
    std::vector<depthHo> compatible_ho_temp;
    float chi = 0;

    for (size_t a=0; a < h.size(); a++) {
        compatible_ho_temp.clear();

        for (size_t b=0; b < h.size(); b++) {
            // test if the hypotheses a and b are compatible
            if (ChiTest(h[a], h[b], &chi)) {
                compatible_ho_temp.push_back(h[b]);
            }
        }
        // test if hypothesis 'a' has the required support
        if (compatible_ho_temp.size()-1 >= lambdaN && compatible_ho_temp.size() > compatible_ho.size()) {
            compatible_ho_temp.push_back(h[a]);
            compatible_ho = compatible_ho_temp;
        }
    }

    // calculate the parameters of the inverse depth distribution by fusing hypotheses
    if (compatible_ho.size() >= lambdaN) {
        GetFusion(compatible_ho, dist, &chi);
    }
}

void ProbabilityMapping::InterKeyFrameDepthChecking(const cv::Mat& im, ORB_SLAM::KeyFrame* currentKf, std::vector<std::vector<depthHo> >& h) {
    std::vector<ORB_SLAM::KeyFrame*> neighbors;

    // option1: could just be the best covisibility keyframes
    neighbors = currentKf->GetBestCovisibilityKeyFrames(covisN);

    // option2: could be found in one of the LocalMapping SearchByXXX() methods
    //ORB_SLAM::LocalMapping::SearchInNeighbors(); //mpCurrentKeyFrame->updateConnections()...AddConnection()...UpdateBestCovisibles()...
    //ORB_SLAM::LocalMapping::mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(covisN); //mvpOrderedConnectedKeyFrames()

    // for each pixel of keyframe_i, project it onto each neighbor keyframe keyframe_j
    // and propagate inverse depth
    for (int px = 0; px < im.rows; px++) {
        for (int py = 0; py < im.cols; py++) {
            if (h[px][py].supported == false) continue;

            float depthp = h[px][py].depth;
            // count of neighboring keyframes in which there is at least one compatible pixel
            int compatible_neighbor_keyframes_count = 0;
            // keep track of compatible pixels for the gauss-newton step
            std::vector<depthHo> compatible_pixels_by_frame[neighbors.size()];
            int n_compatible_pixels = 0;

            for(size_t j=0; j<neighbors.size(); j++) {
                ORB_SLAM::KeyFrame* pKFj = neighbors[j];

                cv::Mat kj = pKFj->GetCalibrationMatrix();
                cv::Mat xp;
                GetXp(kj, px, py, &xp);

                cv::Mat rcwj = pKFj->GetRotation();
                cv::Mat tcwj = pKFj->GetTranslation();

                // Eq (12)
                // compute the projection matrix to map 3D point from original image to 2D point in neighbor keyframe
                cv::Mat temp;
                float denom1, denom2;

                temp = rcwj.row(2) * xp;
                denom1 = temp.at<float>(0,0);
                temp = depthp * tcwj.at<float>(2);
                denom2 = temp.at<float>(0,0);
                float depthj = depthp / (denom1 + denom2);

                cv::Mat xj2d = (kj * rcwj * (1 / depthp) * xp) + (kj * tcwj);
                float xj = xj2d.at<float>(0,0);
                float yj = xj2d.at<float>(1,0);

                std::vector<depthHo> compatible_pixels;
                // look in 4-neighborhood pixel p_j,n around xj for compatible inverse depth
                int pxn = floor(xj);
                int pyn = floor(yj);
                for (int nx = pxn-1; nx <= pxn + 1; nx++) {
                    for (int ny = pyn-1; ny < pyn + 1; ny++) {
                        if ((nx == ny) || ((nx - pxn) && (ny - pyn))) continue;
                        if (!h[nx][ny].supported) continue;
                        // Eq (13)
                        float depthjn = h[nx][ny].depth;
                        float sigmajn = h[nx][ny].sigma;
                        float test = pow((depthj - depthjn), 2) / pow(sigmajn, 2);
                        if (test < 3.84) {
                            compatible_pixels.push_back(h[nx][ny]);
                        }
                    }
                }
                compatible_pixels_by_frame[j] = compatible_pixels; // is this a memory leak?
                n_compatible_pixels += compatible_pixels.size();

                // at least one compatible pixel p_j,n must be found in at least lambdaN neighbor keyframes
                if (compatible_pixels.size()) {
                    compatible_neighbor_keyframes_count++;
                }
            } // for j = 0...neighbors.size()-1

            // don't retain the inverse depth distribution of this pixel if not enough support in neighbor keyframes
            if (compatible_neighbor_keyframes_count < lambdaN) {
                h[px][py].supported = false;
            } else {
                // gauss-newton step to minimize depth difference in all compatible pixels
                // need 1 iteration since depth propagation eq. is linear in depth
                float argmin = depthp;
                cv::Mat J(n_compatible_pixels, 1, CV_32F);
                cv::Mat R(n_compatible_pixels, 1, CV_32F);
                int n_compat_index = 0;
                int iter = 1;
                for (int k = 0; k < iter; k++) {
                    for (size_t j = 0; j < neighbors.size(); j++) {
                        cv::Mat xp;
                        GetXp(neighbors[j]->GetCalibrationMatrix(), px, py, &xp);

                        cv::Mat rji = neighbors[j]->GetRotation();
                        cv::Mat tji = neighbors[j]->GetTranslation();
                        for (size_t i = 0; i < compatible_pixels_by_frame[j].size(); i++) {
                            float ri = 0;
                            Equation14(compatible_pixels_by_frame[j][i], argmin, xp, rji, tji, &ri);
                            R.at<float>(n_compat_index, 0) = ri;

                            cv::Mat tempm = rji.row(2) * xp;
                            float tempf = tempm.at<float>(0,0);
                            depthHo tempdH = compatible_pixels_by_frame[j][i];
                            J.at<float>(n_compat_index, 0) = -1 * tempf / (pow(tempdH.depth, 2) * tempdH.sigma);

                            n_compat_index++;
                        }
                    }
                    cv::Mat temp = J.inv(cv::DECOMP_SVD) * R;
                    argmin = argmin - temp.at<float>(0,0);
                }
                h[px][py].depth = argmin;
            }
        } // for py = 0...im.cols-1
    } // for px = 0...im.rows-1
}

void ProbabilityMapping::Equation14(depthHo& dHjn, float& depthp, cv::Mat& xp, cv::Mat& rji, cv::Mat& tji, float* res) {
    cv::Mat tempm = rji.row(2) * xp;
    float tempf = tempm.at<float>(0,0);
    float tji_z = tji.at<float>(2);
    *res = pow((dHjn.depth - (depthp * tempf) - tji_z) / (pow(dHjn.depth, 2) * dHjn.sigma), 1);
}


////////////////////////
// Utility functions
////////////////////////

void ProbabilityMapping::ComputeInvDepthHypothesis(ORB_SLAM::KeyFrame* kf, int pixel_x, float ustar, float ustar_var, float a, float b, float c,
    ProbabilityMapping::depthHo *dh) {
  int pixel_y = (a/b) * pixel_x + (c/b);
  float inv_pixel_depth =  0.0;
  DBG(cout << "getting pixel depth\n")
  GetPixelDepth(pixel_x,pixel_y,kf, inv_pixel_depth);
  //(inv_frame_rot.row(2)*corrected_image.at<float>(ujcx,vjcx)-fx*inv_frame_rot.row(0)*corrected_image.at<float>(ujcx,vjcx))/(transform_data.row(2)*ujcx[vjcx]+fx*transform_data[0]);

  DBG(cout << "ustar: " << ustar << "   ustar_var: " << ustar_var << endl)
  int ustar_min = ustar - sqrt(ustar_var);
  int vstar_min = (a/b)*ustar_min + (c/b);

  float inv_depth_min = 0.0;
  DBG(cout << "getting min pixel depth\n")
  GetPixelDepth(ustar_min,vstar_min,kf, inv_depth_min);
  //(inv_frame_rot[2]*corrected_image.at<float>(ustarcx_min ,vstarcx_min)-fx*inv_frame_rot[0]*corrected_image.at<float>(ujcx,vjcx))/(-transform_data[2][ustarcx_min][vstarcx_min]+fx*transform_data[0]);

  int ustar_max = ustar +  sqrt(ustar_var);
  int vstar_max = (a/b)*ustar_max + (c/b);

  float inv_depth_max = 0.0;
  DBG(cout << "getting max pixel depth \n")
  GetPixelDepth(ustar_max,vstar_max,kf, inv_depth_max);
  //(inv_frame_rot[2]*corrected_image.at<float>(ustarcx_max ,vstarcx_max)-fx*inv_frame_rot[0]*corrected_image.at<float>(ujcx,vjcx)/)/(-transform_data[2][ustarcx_max][vstarcx_max]+fx*transform_data[0]);
  DBG(cout << "Got Depths! \n";
      cout << "max inv depth: " << inv_depth_max << endl;
      cout << "min inv depth: " << inv_depth_min << endl)

  // Equation 9
  float sigma_depth = cv::max(abs(inv_depth_max), abs(inv_depth_min));

  dh->depth = inv_pixel_depth;
  dh->sigma = sigma_depth;
  DBG(cout << "pixel depth: " << inv_pixel_depth << endl;
      cout << "sigma depth: " << sigma_depth << endl;
      cout << "return from compute Inv depth ho\n")
}

void ProbabilityMapping::GetGradientMagAndOri(const cv::Mat& image, cv::Mat* gradx, cv::Mat* grady, cv::Mat* mag, cv::Mat* ori, cv::Mat* really) {
  DBG(cout << "Going through Scharr convolution\n")
  *gradx = cv::Mat::zeros(image.rows, image.cols, CV_32F);
  *grady = cv::Mat::zeros(image.rows, image.cols, CV_32F);
  *mag =  cv::Mat::zeros(image.rows, image.cols, CV_32F);
  *ori = cv::Mat::zeros(image.rows, image.cols, CV_32F);

  //For built in version
  cv::Scharr(image, *gradx, CV_32F, 1, 0);
  cv::Scharr(image, *grady, CV_32F, 0, 1);

  cv::magnitude(*gradx,*grady,*mag);
  cv::phase(*gradx,*grady,*ori,true);


  SAVE(cout << "printing results of grad mag and theta" << endl;
    std::vector<FILE*> files;
    FILE* fmag_cv = fopen("/home/josh/Workspace/Scanner3D/cv_grad_mag.csv", "w");
    FILE* ftheta_cv = fopen("/home/josh/Workspace/Scanner3D/cv_grad_theta.csv", "w");
    files.push_back(fmag_cv);
    files.push_back(ftheta_cv);
    for (int k=0; k < files.size(); k++) {
      if (files[k]) {
        cv::Mat* m;
        switch (k) {
          case 1: m = mag; break; // cv magnitude
          default: m = ori; break; // cv theta
        }
        for (int i=0; i < m->rows; i++) {
          for (int j=0; j < m->cols; j++) {
            float f = m->at<float>(i,j);
            fprintf(files[k], "%3.3f, ", f);
          }
          fprintf(files[k], "\n");
        }
        fclose(files[k]);
      }
    })
  //For manual version

  //cv::Mat gradx2 = cv::Mat::zeros(image.rows, image.cols, CV_32F);
  //cv::Mat grady2 = cv::Mat::zeros(image.rows, image.cols, CV_32F);
  //cv::Mat sum = cv::Mat::zeros(image.rows, image.cols, CV_32F);
  //cv::pow(*gradx, 2.0, gradx2);
  //cv::pow(*grady, 2.0, grady2);
  //cv::addWeighted(gradx2, 1.0, grady2, 1.0, 0, sum);
  //cv::sqrt(sum, *really);

  //cv::Scharr(image, *gradx, CV_16S, 1, 0);
  //cv::Scharr(image, *grady, CV_16S, 0, 1);

  //cv::Mat absgradx, absgrady;
  //cv::convertScaleAbs(*gradx, absgradx);
  //cv::convertScaleAbs(*grady, absgrady);
  //cv::addWeighted(absgradx2, 0.5, absgrady2, 0.5, 0, *test);


  //DBG(cout << "gradx dump: " << *gradx << endl;
    //  cout << "Type: " << grad->depth() << endl)
}

//might be a good idea to store these when they get calculated during ORB-SLAM.
void ProbabilityMapping::GetInPlaneRotation(ORB_SLAM::KeyFrame* k1, ORB_SLAM::KeyFrame* k2, float* th) {
  std::vector<cv::KeyPoint> vKPU1 = k1->GetKeyPointsUn();
  DBoW2::FeatureVector vFeatVec1 = k1->GetFeatureVector();
  std::vector<ORB_SLAM::MapPoint*> vMapPoints1 = k1->GetMapPointMatches();
  cv::Mat Descriptors1 = k1->GetDescriptors();

  std::vector<cv::KeyPoint> vKPU2 = k2->GetKeyPointsUn();
  DBoW2::FeatureVector vFeatVec2 = k2->GetFeatureVector();
  std::vector<ORB_SLAM::MapPoint*> vMapPoints2 = k2 ->GetMapPointMatches();
  cv::Mat Descriptors2 = k2->GetDescriptors();

  std::vector<int> rotHist[histo_length];
  for(int i=0;i<histo_length;i++)
    rotHist[i].reserve(500);//DescriptorDistance

  const float factor = 1.0f;//histo_length;

  DBoW2::FeatureVector::iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::iterator f2end = vFeatVec2.end();

  while(f1it != f1end && f2it != f2end) {
    if(f1it->first == f2it->first){
      for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++){
        size_t index1 = f1it->second[i1];

        ORB_SLAM::MapPoint* pMP1 = vMapPoints1[index1];
        if(!pMP1)
          continue;
        if(pMP1->isBad())
          continue;

        cv::Mat d1 = Descriptors1.row(index1);

        int bestDist1 = INT_MAX;
        int bestIndex2 = -1;
        int bestDist2 = INT_MAX;
        size_t index2;
        for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++){
          index2 = f2it->second[i2];

          ORB_SLAM::MapPoint* pMP2 = vMapPoints2[index2];
          if(!pMP2)
            continue;
          if(pMP2->isBad())
            continue;

          cv::Mat d2 = Descriptors2.row(index2);

          int dist = ORB_SLAM::ORBmatcher::DescriptorDistance(d1,d2);

          if(dist<bestDist1){
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIndex2 = index2;
          }
          else if(dist<bestDist2){
            bestDist2 = dist;
          }
        }
        if(bestDist1<th_low){
          if(static_cast<float>(bestDist1)<NNRATIO*static_cast<float>(bestDist2)){
            float rot = vKPU1[index1].angle - vKPU2[index2].angle;
            if(rot<0.0)
              rot+=360.0f;
            int bin = round(rot*factor);
            if(bin==histo_length)
              bin=0;
            rotHist[bin].push_back(index1);
          }
        }
      }
    }
  }
  //calculate the median angle
  size_t size = 0;
  for(int i=0;i<histo_length;i++)
    size += rotHist[i].size();

  size_t count = 0;
  for(int i=0;i<histo_length;i++) {
    for (size_t j=0; j < rotHist[i].size(); j++) {
        if (count==(size/2))
            *th = 360 * (float)(i) / histo_length;
        count++;
    }
  }
  //if(size % 2 == 0){
  //  *th = (rotHist[size/2 - 1] + rotHist[size/2])/2;
  //}
  //else{
  //  *th = rotHist[size/2];
  //}
}


void ProbabilityMapping::PixelNeighborSupport(std::vector<std::vector<depthHo> > H, int px, int py, std::vector<depthHo>& support) {
    support.clear();
    float chi = 0;
    for (int x = px - 1; x <= px + 1; x++) {
        for (int y = py - 1; y <= py + 1; y++) {
            if (x == px && y == py) continue;
            if (ChiTest(H[x][y], H[px][py], &chi)) {
                support.push_back(H[px][py]);
            }
        }
    }
}

void ProbabilityMapping::PixelNeighborNeighborSupport(std::vector<std::vector<depthHo> > H, int px, int py, std::vector<std::vector<depthHo> >& support) {
    support.clear();
    float chi = 0;
    for (int x = px - 1; x <= px + 1; x++) {
        for (int y = py - 1; y <= py + 1; y++) {
            if (x == px && y == py) continue;
            std::vector<depthHo> tempSupport;
            for (int nx = px - 1; nx <= px + 1; nx++) {
                for (int ny = py - 1; ny <= py + 1; ny++) {
                    if ((nx == px && ny == py) || (nx == x && ny == y)) continue;
                    if (ChiTest(H[x][y], H[nx][ny], &chi)) {
                        tempSupport.push_back(H[nx][ny]);
                    }
                }
            }
            support.push_back(tempSupport);
        }
    }
}

void ProbabilityMapping::GetIntensityGradient_D(const cv::Mat& ImGrad, float a, float b, float c, int px, float* q) {
    int uplusone = px + 1;
    int vplusone = (a/b)*uplusone + (c/b);
    int uminone = px - 1;
    int vminone = (a/b)*uminone + (c/b);
    *q = (ImGrad.at<float>(uplusone,vplusone) - ImGrad.at<float>(uminone,vminone))/2;
}

void ProbabilityMapping::GetTR(ORB_SLAM::KeyFrame* kf, cv::Mat* t, cv::Mat* r) {

    cv::Mat Rcw2 = kf->GetRotation();
    cv::Mat Rwc2 = Rcw2.t();
    cv::Mat tcw2 = kf->GetTranslation();
    cv::Mat Tcw2(3,4,CV_32F);
    Rcw2.copyTo(Tcw2.colRange(0,3));
    tcw2.copyTo(Tcw2.col(3));

    *t = Tcw2;
    *r = Rcw2;
}

void ProbabilityMapping::GetParameterization(const cv::Mat& F12, const int x, const int y, float* a, float* b, float* c) {
    // parameterization of the fundamental matrix (function of horizontal coordinate)
    // could probably use the opencv built in function instead
    *a = x*F12.at<float>(0,0)+y*F12.at<float>(1,0)+F12.at<float>(2,0);
    *b = x*F12.at<float>(0,1)+y*F12.at<float>(1,1)+F12.at<float>(2,1);
    *c = x*F12.at<float>(0,2)+y*F12.at<float>(1,2)+F12.at<float>(2,2);
}
//Xp = K-1 * xp (below Equation 8)
// map 2D pixel coordinate to 3D point
void ProbabilityMapping::GetXp(const cv::Mat& k, int px, int py, cv::Mat* xp) {

    cv::Mat xp2d = cv::Mat(3,1,CV_32F);

    xp2d.at<float>(0,0) = px;
    xp2d.at<float>(1,0) = py;
    xp2d.at<float>(2,0) = 1;

    DBG(cout << "calculating Xp\n")
    *xp = k.inv() * xp2d;
}


// Equation (8)
void ProbabilityMapping::GetPixelDepth(int px, int py, ORB_SLAM::KeyFrame* kf, float &p) {

    float fx = kf->fx;
    float cx = kf->cx;

    int ucx = px - cx;

    cv::Mat rcw = kf->GetRotation();
    cv::Mat tcw = kf->GetTranslation();

    cv::Mat xp;
    DBG(cout << "Getting XP\n")
    GetXp(kf->GetCalibrationMatrix(), px, py, &xp);


    cv::Mat temp = rcw.row(2) * xp;
    double num1 = temp.at<float>(0,0);
    temp = fx * (rcw.row(0) * xp);
    double num2 = temp.at<float>(0,0);
    double denom1 = -tcw.at<float>(2) * ucx;
    double denom2 = fx * tcw.at<float>(0);

    DBG(cout << "calculate depth\n";
        cout << "num1:" << num1 << endl;
        cout << "num2:" << num2 << endl;
        cout << "denom1:" << denom1 << endl;
        cout << "denom2:" << denom2 << endl)


    p = (num1 - num2) / (denom1 + denom2);
    DBG(cout << "depth: " << p << endl)
}

bool ProbabilityMapping::ChiTest(const depthHo& ha, const depthHo& hb, float* chi_val) {
    float chi_test = (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma) + (ha.depth - hb.depth)*(ha.depth - hb.depth) / (ha.sigma*ha.sigma);
    if (chi_val)
        *chi_val = chi_test;
    return (chi_test < 5.99);
}

void ProbabilityMapping::GetFusion(const std::vector<depthHo>& compatible_ho, depthHo* hypothesis, float* min_sigma) {
    hypothesis->depth = 0;
    hypothesis->sigma = 0;

    float temp_min_sigma = 0;
    float pjsj =0; // numerator
    float rsj =0; // denominator

    for (size_t j = 0; j < compatible_ho.size(); j++) {
        pjsj += compatible_ho[j].depth / pow(compatible_ho[j].sigma, 2);
        rsj += 1 / pow(compatible_ho[j].sigma, 2);
        if (pow(compatible_ho[j].sigma, 2) < pow(temp_min_sigma, 2)) {
            temp_min_sigma = compatible_ho[j].sigma;
        }
    }

    hypothesis->depth = pjsj / rsj;
    hypothesis->sigma = sqrt(1 / rsj);
    if (min_sigma) {
        *min_sigma = temp_min_sigma;
    }
}

cv::Mat ProbabilityMapping::ComputeFundamental(ORB_SLAM::KeyFrame *&pKF1, ORB_SLAM::KeyFrame *&pKF2) {
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = GetSkewSymmetricMatrix(t12);

    cv::Mat K1 = pKF1->GetCalibrationMatrix();
    cv::Mat K2 = pKF2->GetCalibrationMatrix();


    return K1.t().inv()*t12x*R12*K2.inv();
}

cv::Mat ProbabilityMapping::GetSkewSymmetricMatrix(const cv::Mat &v) {
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                                  v.at<float>(2),               0,-v.at<float>(0),
                                 -v.at<float>(1),  v.at<float>(0),              0);
}
