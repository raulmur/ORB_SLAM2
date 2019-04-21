#include "deeplcd.h"

namespace deeplcd
{


void QueryResults::insertInPlace(const query_result& q) 
{
	if (q.score > at(size()-1).score)
	{	
		pop_back();
		QueryResults::iterator itToInsert = std::lower_bound(begin(), end(), q,
			[](const query_result& q1, const query_result& q2){return q1.score > q2.score;});
		insert(itToInsert, q);
	}
}


DeepLCD::DeepLCD(const std::string& network_definition_file, 
	const std::string& pre_trained_model_file,
	int gpu_id)
	: db( Database() )
{

	std::string mode = "CPU";
        if(gpu_id >= 0)
        {
                caffe::Caffe::set_mode(caffe::Caffe::GPU);
                caffe::Caffe::SetDevice(gpu_id);
		mode = "GPU";
        }
	else 
	{
		caffe::Caffe::set_mode(caffe::Caffe::CPU);
	}
	clock_t begin = clock();
        autoencoder = new caffe::Net<float>(network_definition_file, caffe::TEST);	
        autoencoder->CopyTrainedLayersFrom(pre_trained_model_file);
        clock_t end = clock();
	std::cout << "\nCaffe mode = " << mode << "\n";
        std::cout << "Loaded model in " <<   double(end - begin) / CLOCKS_PER_SEC << " seconds\n";
	autoencoder_input = autoencoder->input_blobs()[0]; // construct the input blob shared_ptr
	autoencoder_output = autoencoder->output_blobs()[0]; // construct the output blob shared_ptr
}

uint32_t DeepLCD::add(const cv::Mat &im)
{
	descriptor descr = calcDescr(im); // Calculate the descriptor
	db.push_back(descr); // push to the database
	return descr.id;
}


void DeepLCD::query(const cv::Mat& im, QueryResults& results, size_t max_res, bool add_after)
{
	descriptor descr = calcDescr(im);
	query(descr, results, max_res, add_after);
}


void DeepLCD::query(const descriptor& descr, QueryResults& results, size_t max_res, bool add_after)
{
	if (max_res > db.size())
		throw std::runtime_error("max_res requested is greater than database size");

	results.invalidate(); // This prevents old results from interfering with current results
	if (results.size() != max_res)
		results.resize(max_res);

	int i = db.size();
	for (descriptor d : db)
	{
		query_result q(score(d.descr, descr.descr), d.id);
		results.insertInPlace(q);
		if (--i == n_exclude)
			break;
	}

	if (add_after)
		db.push_back(descr);

}


query_result DeepLCD::query(const cv::Mat& im, bool add_after)
{
	descriptor descr = calcDescr(im);
	return query(descr, add_after);
}

query_result DeepLCD::query(const descriptor& descr, bool add_after)
{

	query_result q(-1.0, 0);
	float s;
	int i = db.size();
	for (descriptor d : db)
	{
		s = score(d.descr, descr.descr);
		if (s > q.score)
		{
			q.score = s;
			q.id = d.id;
		}	
		if (--i == n_exclude)
			break;
	}

	if (add_after)
		db.push_back(descr);
	return q;
}

const float DeepLCD::score(const Vector& d1, const Vector& d2)
{
	return d1.dot(d2);
}


const descriptor DeepLCD::calcDescr(const cv::Mat& im_)
{

	std::vector<cv::Mat> input_channels(1); //We need this wrapper to place data into the net. Allocate space for at most 3 channels	
	int w = autoencoder_input->width();
	int h = autoencoder_input->height();
	float* input_data = autoencoder_input->mutable_cpu_data();
	cv::Mat channel(h, w, CV_32FC1, input_data);
	input_channels.emplace(input_channels.begin(), channel);
	input_data += w * h;
	cv::Mat im(im_.size(), CV_32FC1);
	im_.convertTo(im, CV_32FC1, 1.0/255.0); // convert to [0,1] grayscale. Place in im instead of im_
	// This will write the image to the input layer of the net
	cv::split(im, input_channels);
	autoencoder->Forward(); // Calculate the forward pass
	const float* tmp_descr;
	tmp_descr = autoencoder_output->cpu_data(); 
	int p = autoencoder_output->channels(); // Flattened layer get the major axis in channels dimension

	// We need to copy the data, or it will be overwritten on the next Forward() call
	// We may have a TON of desciptors, so allocate on the heap to avoid stack overflow
	int sz = p * sizeof(float);
	float* descr_ = (float*)std::malloc(sz);
	std::memcpy(descr_, tmp_descr, sz);

	descriptor descr(curr_id++, descr_, p);	

	return descr;

}

} // end namespace















