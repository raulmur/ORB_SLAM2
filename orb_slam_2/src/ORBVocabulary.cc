#include <DBoW2/TemplatedVocabulary.h>
#include <glog/logging.h>

#include "DBoW2/FORB.h"
#include "orb_vocabulary.pb.h"

namespace DBoW2 {

template<>
void TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>::loadProto(
    const std::string& file_name)
{
  std::ifstream file(file_name.c_str());
  CHECK(file.is_open()) << "Couldn't open " << file_name;

  google::protobuf::io::IstreamInputStream raw_in(&file);
  google::protobuf::io::CodedInputStream coded_in(&raw_in);
  coded_in.SetTotalBytesLimit(std::numeric_limits<int>::max(), -1);

  proto::ORBVocabulary vocabulary_proto;
  CHECK(vocabulary_proto.ParseFromCodedStream(&coded_in));

  m_k = vocabulary_proto.k();
  m_L = vocabulary_proto.l();
  m_scoring = static_cast<ScoringType>(vocabulary_proto.scoring_type());
  m_weighting = static_cast<WeightingType>(vocabulary_proto.weighting_type());

  createScoringObject();

  // nodes
  m_nodes.resize(vocabulary_proto.nodes_size() + 1); // +1 to include root
  m_nodes[0].id = 0;

  for(const proto::ORBNode& node : vocabulary_proto.nodes())
  {
    const NodeId node_id = node.node_id();
    const NodeId parent_id = node.parent_id();

    m_nodes[node_id].id = node.node_id();
    m_nodes[node_id].parent = node.parent_id();
    m_nodes[node_id].weight = node.weight();

    m_nodes[parent_id].children.push_back(node_id);

    // For now only works with ORB.
    CHECK_EQ(node.node_descriptor().length(), FORB::L);
    m_nodes[node_id].descriptor = cv::Mat(
        1, FORB::L, CV_8U, const_cast<char*>(
            node.node_descriptor().c_str())).clone();
  }

  m_words.resize(vocabulary_proto.words_size());

  for(const proto::ORBWord& word : vocabulary_proto.words())
  {
    const WordId word_id = word.word_id();
    const NodeId node_id = word.node_id();

    m_nodes[node_id].word_id = word_id;
    m_words[word_id] = &m_nodes[node_id];
  }
}

template<>
void TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>::saveProto(
    const std::string& file_name) const
{
  std::ofstream file(file_name);
  CHECK(file.is_open()) << "Couldn't open " << file_name;
  proto::ORBVocabulary vocabulary_proto;

  vocabulary_proto.set_k(m_k);
  vocabulary_proto.set_l(m_L);
  vocabulary_proto.set_scoring_type(m_scoring);
  vocabulary_proto.set_weighting_type(m_weighting);

  vector<NodeId> parents, children;
  vector<NodeId>::const_iterator pit;

  parents.push_back(0); // root

  while(!parents.empty())
  {
    NodeId pid = parents.back();
    parents.pop_back();

    const Node& parent = m_nodes[pid];
    children = parent.children;

    for(pit = children.begin(); pit != children.end(); pit++)
    {
      const Node& child = m_nodes[*pit];

       proto::ORBNode* node_proto = vocabulary_proto.add_nodes();
      CHECK_NOTNULL(node_proto);
      node_proto->set_node_id(child.id);
      node_proto->set_parent_id(pid);
      node_proto->set_weight(child.weight);

      // For now only works with BRIEF.
      char descriptor_array[FORB::L];
      CHECK_EQ(child.descriptor.rows, 1);
      CHECK_EQ(child.descriptor.cols, FORB::L);
      memcpy(descriptor_array, child.descriptor.data, FORB::L);
      node_proto->set_node_descriptor(std::string(descriptor_array, FORB::L));

      // add to parent list
      if(!child.isLeaf())
      {
        parents.push_back(*pit);
      }
    }
  }

  typename vector<Node*>::const_iterator wit;
  for(wit = m_words.begin(); wit != m_words.end(); wit++)
  {
    WordId id = wit - m_words.begin();

    proto::ORBWord* word_proto = vocabulary_proto.add_words();
    CHECK_NOTNULL(word_proto);
    word_proto->set_word_id(id);
    word_proto->set_node_id((*wit)->id);
  }

  CHECK(vocabulary_proto.SerializeToOstream(&file));
}

}  // namespace DBoW2
