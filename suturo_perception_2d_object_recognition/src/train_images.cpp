#include <stdio.h>
#include <iostream>
#include <boost/program_options.hpp>
#include <algorithm>
#include <iterator>

#include "object_matcher.h"

// Include available Matcher Classes
#include "simple_matcher.h"
#include "nndr_matcher.h"
#include "symmetry_nndr_matcher.h"
 
using namespace boost;
using namespace std;

namespace po = boost::program_options;

/** @function main */
int main( int argc, char** argv )
{

  // "HashMap" for program parameters
   po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("train-img,t", po::value<std::vector<std::string> >()->required(), "A training image, of which the keypoints, detectors and image should be stored")
      ("label,l", po::value<std::vector<std::string> >()->required(), "A label for a given training image. For every --train-img you pass, you must also specify a --label")
      ("database-file,f", po::value<std::string>()->required(), "The filename for the training database")
    ;

    po::store(po::parse_command_line(argc, argv, desc), vm); // Without positional options

    if (vm.count("help")) 
    {
      cout << "Usage: train_images [-t image1 -l label] -f filename_for_training_data..." << endl << endl;
      cout << desc << "\n";
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

    // if(vm.count("train-img"))
    // {
        // Take the first train-image in normal matching mode
    if(vm["train-img"].as<std::vector<std::string> >().size() != vm["label"].as<std::vector<std::string> >().size() )
        throw po::validation_error(po::validation_error::invalid_option_value, " Different amount of training images and labels");

  }
  catch(std::exception& e)
  {
    cout << "Usage: train_images [-t image1 -l label] -f filename_for_training_data..." << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 
  int train_img_count = vm["train-img"].as<std::vector<std::string> >().size();
  string database_file = vm["database-file"].as<string>();
  cout << "Train " << train_img_count << " image(s) to the database at " << database_file << endl;
  ObjectMatcher om; // Create ObjectMatcher with SURF as default
  om.trainImagesToDatabase(database_file, vm["train-img"].as<std::vector<std::string> >(), vm["label"].as<std::vector<std::string> >() );
  om.readTrainImagesFromDatabase(database_file);

  // ObjectMatcher om(detector,extractor);
  return 0;
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
