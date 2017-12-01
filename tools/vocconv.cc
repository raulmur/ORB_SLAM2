#include "ORBVocabulary.h"

#include <time.h>
#include <memory>
#include <exception>

namespace vocconv
{

bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

bool load_as_text(ORB_SLAM2::ORBVocabulary *voc, const std::string infile)
{
    clock_t tStart = clock();
    bool res = voc->loadFromTextFile(infile);
    printf("Loading fom text: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
    return res;
}

void load_as_xml(ORB_SLAM2::ORBVocabulary *voc, const std::string infile)
{
    clock_t tStart = clock();
    voc->load(infile);
    printf("Loading fom xml: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void load_as_binary(ORB_SLAM2::ORBVocabulary *voc, const std::string infile)
{
    clock_t tStart = clock();
    voc->loadFromBinaryFile(infile);
    printf("Loading fom binary: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void save_as_xml(ORB_SLAM2::ORBVocabulary *voc, const std::string outfile)
{
    clock_t tStart = clock();
    voc->save(outfile);
    printf("Saving as xml: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void save_as_text(ORB_SLAM2::ORBVocabulary *voc, const std::string outfile)
{
    clock_t tStart = clock();
    voc->saveToTextFile(outfile);
    printf("Saving as text: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void save_as_binary(ORB_SLAM2::ORBVocabulary *voc, const std::string outfile)
{
    clock_t tStart = clock();
    voc->saveToBinaryFile(outfile);
    printf("Saving as binary: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
}

void load_vocabulary(ORB_SLAM2::ORBVocabulary *voc, const std::string infile)
{
    if (has_suffix(infile, ".txt"))
    {
        load_as_text(voc, infile);
    }
    else if (has_suffix(infile, ".xml"))
    {
        load_as_xml(voc, infile);
    }
    else if (has_suffix(infile, ".bin"))
    {
        load_as_binary(voc, infile);
    }
    else
    {
        throw std::invalid_argument(infile + " is not supported");
    }
}

void save_vocabulary(ORB_SLAM2::ORBVocabulary *voc, const std::string outfile)
{
    if (has_suffix(outfile, ".txt"))
    {
        save_as_text(voc, outfile);
    }
    else if (has_suffix(outfile, ".xml"))
    {
        save_as_xml(voc, outfile);
    }
    else if (has_suffix(outfile, ".bin"))
    {
        save_as_binary(voc, outfile);
    }
    else
    {
        throw std::invalid_argument(outfile + " is not supported");
    }
}

} // namespace vocconv

int main(int argc, char **argv)
{
    cout << "BoW load/save benchmark" << endl;
    std::unique_ptr<ORB_SLAM2::ORBVocabulary> voc = std::make_unique<ORB_SLAM2::ORBVocabulary>();

    if (argc != 3)
    {
        cout << "Invalid arguments\n";
        cout << "Usage: bin_vocabulary <input-path> <output-path>\n";
        cout << "Only .xml, .txt, .bin extentions are supported\n";
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    vocconv::load_vocabulary(voc.get(), input_file);
    vocconv::save_vocabulary(voc.get(), output_file);

    return 0;
}
