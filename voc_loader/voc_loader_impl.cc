#undef NDEBUG
#include "ORBVocabulary.h"
#include "record.pb.h"

using namespace ORB_SLAM2;

static bool file_exist(const std::string &fileName) {
	std::ifstream infile(fileName);
	return infile.good();
}

bool ORBVocabulary::loadFromTextFile2(const std::string &filename) {
	std::string bin_file_name = filename.substr(0, filename.find_last_of(".")) + ".bin";
	if (!file_exist(bin_file_name)) {
		bool ret = super::loadFromTextFile(filename);
		std::cout << "saving binary form ..." << std::endl;
		saveBinary(bin_file_name);
		return ret;
	}
	loadBinary(bin_file_name);
	return true;
}

void ORBVocabulary::saveBinary(std::string fileName) {

	file_t file;

	header_t *header = file.mutable_header();
	// write bag info
	header->set_m_k(m_k);
	header->set_m_l(m_L);
	header->set_m_scoring(m_scoring);
	header->set_m_weighting(m_weighting);


	for (auto &node : m_nodes) {
		if (node.id == 0) {
			// skip root node
			continue;
		}
		record_t *record = file.add_nodes();
		record->set_id(node.id);
		record->set_parent(node.parent);
		record->set_data(node.descriptor.ptr(0), node.descriptor.cols);
		record->set_weight(node.weight);
	}

	{
		std::ofstream fs(fileName, std::ios::out | std::ios::binary);
		file.SerializeToOstream(&fs);
		fs.close();
	}
}

void ORBVocabulary::loadBinary(std::string fileName) {

	file_t file;
	{
		std::ifstream fs(fileName, std::ios::in | std::ios::binary);
		assert(file.ParseFromIstream(&fs));
		fs.close();
	}

	const header_t &header = file.header();

	m_k = header.m_k();
	m_L = header.m_l();

	// Create scoring object
	m_scoring = (DBoW2::ScoringType) header.m_scoring();
	m_weighting = (DBoW2::WeightingType)header.m_weighting();
	createScoringObject();

	// Expected nodes
	int expected_nodes = (int)((pow((double)m_k, (double)m_L + 1) - 1) / (m_k - 1));

	// Create tree structure, allocate space
	m_nodes.reserve(expected_nodes);
	m_words.reserve(pow((double)m_k, (double)m_L + 1));

	// line by line read
	for (auto &record : file.nodes()) {

		// Update tree structure
		unsigned int nid = record.id();
		unsigned int pid = record.parent();
		if (m_nodes.size() <= std::max(nid, pid)) {
			m_nodes.resize(std::max(nid, pid) + 1);
		}
		m_nodes[nid].id = nid;
		m_nodes[nid].parent = pid;
		m_nodes[pid].children.push_back(nid);
		m_nodes[nid].weight = record.weight();

		// Create descriptor
		if (!record.data().empty()) {
			assert(record.data().size() == 32);
			m_nodes[nid].descriptor.create(1, record.data().size(), CV_8U);
			unsigned char *p = m_nodes[nid].descriptor.ptr<unsigned char>();
			record.data().copy((char *)p, record.data().size(), 0);
		}

	}

	for (auto &node : m_nodes) {
		if (node.isLeaf()) {
			int wid = m_words.size();
			m_words.resize(wid + 1);

			node.word_id = wid;
			m_words[wid] = &node;
		}

	}

}