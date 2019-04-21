#ifndef DEEPLCD_H
#define DEEPLCD_H

#include "caffe/caffe.hpp"

#include "opencv2/core/core.hpp"

#include <Eigen/Core>

#include <list>
#include <vector>
#include <iostream>


namespace deeplcd
{

typedef Eigen::Map<Eigen::VectorXf> Vector; // This is just a bit more sleek

struct descriptor
{
	uint32_t id;	
	Vector descr;

	descriptor(size_t id_, Vector descr_) : id(id_), descr(descr_) {}
	descriptor(size_t id_, float* descr_, int p) : id(id_), descr( Vector(descr_, p, 1) )
	{
		descr.normalize(); // Normalize now so that we only have to take the dot to get cosine similarity, reducing the overall query time	
	}
	friend std::ostream& operator << (std::ostream& stream, const descriptor& d)
	{
		stream << "descriptor: ID=" << d.id;
		return stream;
	}

};

class Database : public std::list<descriptor>
{

};

struct query_result
{
	uint32_t id; // Same id as above
	float score;

	query_result(float score_, uint32_t id_) : id(id_), score(score_) {}

	query_result() // Only needed for vector construction in QueryResults
	{
		query_result(-1.0, 0);
	}
	
	friend std::ostream& operator << (std::ostream& stream, const query_result& q)
	{
		stream << "query_result: ID=" << q.id << ", score=" << q.score;
		return stream;
	}
};

class QueryResults : public std::vector<query_result>
{

public:
	QueryResults(int sz) : std::vector<query_result>(sz) {}
	QueryResults() : std::vector<query_result>(1) {}
	void insertInPlace(const query_result &q); 
	friend std::ostream& operator << (std::ostream& stream, const QueryResults& Q)
	{
		stream << "QueryResults: {";
		for (query_result q : Q)
			stream << "\n\t" << q;
		stream << "\n}\n";
		return stream;
	}
	void invalidate() 
	{
		for (size_t i = 0; i < size(); i++)
			at(i) = query_result(-1.0, 0); // This is needed so old results wont interfere with new ones
		clear();	
	}
};

// Deep Loop Closure Detector
class DeepLCD
{

public:

	Database db; // database of image descriptors
	uint32_t curr_id = 0; // Increment unbounded. We will never get near a billion images, so it is ok to do so
	int n_exclude = 0; // Number of most recent frames to exclude from search space

	caffe::Net<float>* autoencoder; // the deploy autoencoder
	caffe::Blob<float>* autoencoder_input; // The encoder's input blob
	caffe::Blob<float>* autoencoder_output; // The encoder's input blob

	// If gpu_id is -1, the cpu will be used
	DeepLCD(const std::string& network_definition_file="calc_model/deploy.prototxt", const std::string& pre_trained_model_file="calc_model/calc.caffemodel", int gpu_id=-1);
	
	~DeepLCD()
	{
		delete autoencoder;
	}
	
	// input image, calculate and save image descriptor to database, then return its internal ID
	uint32_t add(const cv::Mat& im);

	/***********************************
	* Use these if max_res nearest neighbors are desired
	****************************************/
	// If add_after, the descriptor from im will be added to the database. 
	void query(const cv::Mat& im, QueryResults& results, size_t max_res=1, bool add_after=1);

	// This is used internally by the above overload, but it is made public for convenience. 
	void query(const descriptor& descr, QueryResults& results, size_t max_res=1, bool add_after=1);
	/*****************************************
	* Use these if only one nearest neighbor is desired. They will be less expensive
	******************************************/
	// If add_after, the descriptor from im will be added to the database. 
	query_result query(const cv::Mat& im, bool add_after=1);

	// This is used internally by the above overload, but it is made public for convenience. 
	query_result query(const descriptor& descr, bool add_after=1);


	const float score(const Vector& d1, const Vector& d2);

	const descriptor calcDescr(const cv::Mat& im); // make a forward pass through the net, return the descriptor
}; // end class DeepLCD


} // end namespace 

#endif //DEEPLCD_H