#include <glog/logging.h>

#include"DBoW2/FORB.h"
#include"DBoW2/TemplatedVocabulary.h"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
ORBVocabulary;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  CHECK_EQ(argc, 2) << "Usage: " << argv[0] << " <classical-format vocab>";
  const std::string file_name(argv[1]);

  ORBVocabulary vocabulary;
  vocabulary.loadFromTextFile(file_name);
  vocabulary.saveProto(file_name + ".proto");
}
