//
// Created by root on 18-4-3.
//

#ifndef ORB_SLAM2_LSA_TR_H
#define ORB_SLAM2_LSA_TR_H

#include <eigen3/Eigen/Dense>

#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <time.h>
#include <algorithm>
#include <boost/format.hpp>

// Boost
#include <boost/config.hpp>
#include <iostream>
#include <vector>
#include <utility>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/math/special_functions/fpclassify.hpp>


const double LAMBDA_LAGRANGIAN = 0.1;
const double REDUCTION_RATIO_THRESHOLD = 0.25;
const double MAX_LAMBDA_LAGRANGIAN = 1e5;
const double LAMBDA_MULTIPLIER = 1.5;
const double PRECISION_COMPARE_GEO_LAMBDA = 1e-9;
const double LAMBDA_LAGRANGIAN_RESTART = 0.1;

inline double computeEnergy(const Eigen::VectorXf &unary, const Eigen::MatrixXf &pairwise, const Eigen::VectorXi &labelling);
inline void computeApproxUnaryTerms(uint32_t size_n, Eigen::VectorXf* approxUnary, const Eigen::VectorXf &unary, const Eigen::MatrixXf &pairwise, const Eigen::VectorXi &currLabeling);
inline double computeApproxEnergy(const Eigen::VectorXf &approxUnary, const Eigen::VectorXi &labeling);
inline void computeApproxLabeling(uint32_t size_n, Eigen::VectorXi* lambdaLabeling, double lambda, const Eigen::VectorXf &approxUnary, const Eigen::VectorXi &currLabeling);
inline void findMinimalChangeBreakPoint(uint32_t size_n, double* bestLambda, Eigen::VectorXi* bestLabeling, const Eigen::VectorXf &approxUnary, const Eigen::VectorXi &currLabeling, double currlambda);
inline void LSA_TR(double* outputEnergy, Eigen::VectorXi* outputLabeling, uint32_t size_n, const Eigen::VectorXf &unary, const Eigen::MatrixXf &pairwise, const Eigen::VectorXi &initLabeling);

/*
int main (int argc, char ** argv){
	uint32_t size_n = 3;

	float* pUnary;
	pUnary = new float[3];
	pUnary[0] = -0.5;
	pUnary[1] = -0.25;
	pUnary[2] = -0.75;

	Eigen::VectorXf unary = Eigen::Map<Eigen::MatrixXf> (pUnary, size_n, 1);
        Eigen::MatrixXf pairwise(size_n,size_n);
        pairwise = Eigen::MatrixXf::Zero(size_n, size_n);

	//Eigen::VectorXf unary(size_n);
	//unary(0) = -0.5;
	//unary(1) = -0.25;
	//unary(2) = -0.75;
        pairwise(0,1) = 1;
	pairwise(1,0) = 1;
        pairwise(0,2) = 1;
	pairwise(2,0) = 1;

        Eigen::VectorXf initLabeling(size_n);
	initLabeling = Eigen::VectorXf::Ones(size_n);
	//initLabeling(1) = 1;
        Eigen::VectorXf finalLabeling(size_n);
	double finalEnergy = 0;
        LSA_TR(&finalEnergy, &finalLabeling, size_n, unary, pairwise, initLabeling);
	std::cout << "Final energy " << finalEnergy << "\n";
	std::cout << "Final labeling " << finalLabeling.transpose() << "\n";
	return 0;
}
*/

// unary is a nx2 matrix, pairwise is a nxn matrix
void LSA_TR(double* outputEnergy, Eigen::VectorXi* outputLabeling, uint32_t size_n,
	    const Eigen::VectorXf &unary, const Eigen::MatrixXf &pairwise, const Eigen::VectorXi &initLabeling){

	Eigen::VectorXi currLabeling;
	currLabeling = initLabeling;
	Eigen::VectorXi lambdaLabeling;
	double currEnergy = computeEnergy(unary, pairwise, currLabeling);
	double lambdaEnergy = 0;
	Eigen::VectorXf approxUnary(size_n);
	computeApproxUnaryTerms(size_n, &approxUnary, unary, pairwise, currLabeling);
	double lambda = LAMBDA_LAGRANGIAN;
	double actualReduction = 0;
	double predictedReduction = 0;
        bool stopFlag = false;
 	while (stopFlag==false){
                //std::cout << ".";

                //Compute lamda Labelling
		computeApproxLabeling(size_n, &lambdaLabeling, lambda, approxUnary, currLabeling);
                double currApproxE = computeApproxEnergy(approxUnary, currLabeling);
                double lambdaApproxE = computeApproxEnergy(approxUnary, lambdaLabeling);
		predictedReduction = currApproxE - lambdaApproxE;
		if (predictedReduction < 0){
			std::cout << "Negative reduction \n";
			stopFlag = true;
		}
		bool updateSolutionFlag = false;

		// there is no updates, find another breaking point (ie smaller lambda)
		if (lambdaLabeling == currLabeling){
                        //std::cout << "\n Finding new break point \n";
			findMinimalChangeBreakPoint(size_n, &lambda, &lambdaLabeling, approxUnary, currLabeling, lambda);
			lambdaApproxE = computeApproxEnergy(approxUnary, lambdaLabeling);
			lambdaEnergy = computeEnergy(unary, pairwise, lambdaLabeling);
			double temp = computeEnergy(unary, pairwise, currLabeling);


			predictedReduction = currApproxE - lambdaApproxE;
			if (predictedReduction < 0){
				std::cout << "Negative predicted reduction \n";
				stopFlag = true;
			}
			actualReduction = currEnergy - lambdaEnergy;
			if (actualReduction <= 0 || lambdaLabeling.sum()== 0 || lambdaLabeling.sum()== size_n){
				stopFlag = true;
				//std::cout << "Optimization done! \n";
			}else{
				if (lambda == 0) lambda = LAMBDA_LAGRANGIAN_RESTART;
				updateSolutionFlag = true;
			}

		}else{
			// Compute actual energy with lambda labeling
			lambdaEnergy = computeEnergy(unary, pairwise, lambdaLabeling);
			actualReduction = currEnergy - lambdaEnergy;
			if (actualReduction <= 0) updateSolutionFlag = false;
			else{
				 updateSolutionFlag = true;
			}
		}

		// If we don't stop, update solution, re-adjust lamdba parameter.
		if (stopFlag == false){
			double reductionRatio = actualReduction/predictedReduction;
			if (reductionRatio < REDUCTION_RATIO_THRESHOLD){
				if (lambda < MAX_LAMBDA_LAGRANGIAN)
					lambda *= LAMBDA_MULTIPLIER;
			}
			else{
				if (lambda > PRECISION_COMPARE_GEO_LAMBDA)
					lambda /= LAMBDA_MULTIPLIER;
			}
  			// Update solution
			if (updateSolutionFlag == true){
				currLabeling = lambdaLabeling;
				currEnergy =  lambdaEnergy;
   				computeApproxUnaryTerms(size_n, &approxUnary, unary, pairwise, currLabeling);
			}
		}
	}
        //std::cout << "\n";
	*outputEnergy = currEnergy;
	*outputLabeling = currLabeling;
}

double computeEnergy(const Eigen::VectorXf &unary, const Eigen::MatrixXf &pairwise, const Eigen::VectorXi &labeling){
	Eigen::VectorXf labeling_f = labeling.cast <float> ();
	double UE = unary.dot(labeling_f);
        double PE = 0;
        Eigen::VectorXf temp = labeling_f.transpose()*pairwise;
        PE = temp.dot(labeling_f);
        return UE + PE;
}

double computeApproxEnergy(const Eigen::VectorXf &approxUnary, const Eigen::VectorXi &labeling){

	Eigen::VectorXf labeling_f = labeling.cast <float> ();
	double E = approxUnary.dot(labeling_f);
        return E;
}

void computeApproxLabeling(uint32_t size_n, Eigen::VectorXi* lambdaLabeling, double lambda,
			  const Eigen::VectorXf &approxUnary, const Eigen::VectorXi &currLabeling){

	// Hamming distance from the current labelling
        Eigen::VectorXf currLabeling_f = currLabeling.cast <float> ();
	Eigen::MatrixXf distUE(2,size_n);
	distUE = Eigen::MatrixXf::Zero(2,size_n);
	distUE.row(0) = currLabeling_f;
	distUE.row(1) = Eigen::VectorXf::Ones(size_n) - currLabeling_f;

	Eigen::MatrixXf approxUnaryAll(2, size_n);
	approxUnaryAll.row(0) = lambda*distUE.row(0);
	approxUnaryAll.row(1) = approxUnary.transpose() + lambda*distUE.row(1);

	Eigen::VectorXf temp = approxUnaryAll.row(0) - approxUnaryAll.row(1);
	*lambdaLabeling = (temp.array() < 0).select(Eigen::VectorXi::Zero(size_n),Eigen::VectorXi::Ones(size_n));
}

void computeApproxUnaryTerms(uint32_t size_n, Eigen::VectorXf* approxUnary,
			     const Eigen::VectorXf &unary, const Eigen::MatrixXf &pairwise, const Eigen::VectorXi &currLabeling){
        Eigen::VectorXf currLabeling_f = currLabeling.cast <float> ();
        Eigen::VectorXf approxPairwise = currLabeling_f.transpose()*pairwise;
	*approxUnary = unary.transpose() + 2*approxPairwise.transpose();
}


void findMinimalChangeBreakPoint(uint32_t size_n, double* bestLambda, Eigen::VectorXi* bestLabeling,
				 const Eigen::VectorXf &approxUnary, const Eigen::VectorXi &currLabeling, double currlambda){

        Eigen::VectorXf currLabeling_f = currLabeling.cast <float> ();
	bool foundLambda = false;

	// Hamming distance from the current labelling
	Eigen::MatrixXf distUE(2,size_n);
	distUE = Eigen::MatrixXf::Zero(2,size_n);
	distUE.row(0) = currLabeling_f;
	distUE.row(1) = Eigen::VectorXf::Ones(size_n) - currLabeling_f;

	double topLambda = currlambda;
	Eigen::VectorXi topLabeling;
	computeApproxLabeling(size_n, &topLabeling, topLambda, approxUnary, currLabeling);

	while (topLabeling != currLabeling){
		topLambda *= LAMBDA_MULTIPLIER;
		computeApproxLabeling(size_n, &topLabeling, topLambda, approxUnary, currLabeling);
	}

	double bottomLambda = PRECISION_COMPARE_GEO_LAMBDA;
	Eigen::VectorXi bottomLabeling;
	computeApproxLabeling(size_n, &bottomLabeling, bottomLambda, approxUnary, currLabeling);

	while (foundLambda==false){
		double middleLambda = 0.5*topLambda + 0.5*bottomLambda;

		Eigen::VectorXi middleLabeling;
		computeApproxLabeling(size_n, &middleLabeling, middleLambda, approxUnary, currLabeling);
		if (middleLabeling != topLabeling){
			bottomLambda = middleLambda;
			bottomLabeling = middleLabeling;
		}else if (middleLabeling != bottomLabeling){
			topLambda = middleLambda;
			topLabeling = middleLabeling;
		}else{
			foundLambda = true;
		}
		if ((topLambda - bottomLambda) < PRECISION_COMPARE_GEO_LAMBDA)	foundLambda = true;
	}
	*bestLambda = bottomLambda;
	*bestLabeling = bottomLabeling;
}




#endif //ORB_SLAM2_LSA_TR_H

