// TODO: Use KD-Tree
// https://code.google.com/p/find-object/source/browse/trunk/find_object/example/main.cpp
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
  bool headless_mode = false;
  std::string train_image;
  std::vector<std::string> test_images;
  int min_good_matches;
  std::string matcher;
  bool database_mode = false;
  std::string database_file;

  // Should we match multiple training images against the test images?
  bool multi_train_mode=false;

  // "HashMap" for program parameters
   po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("train-img,t", po::value<std::vector<std::string> >(), "The training image, that should be matched in the test images")
      ("test-img,i", po::value<std::vector<std::string> >()->required(), "A set of images where train-img should be recognized")
      // zero_tokens tells program_options to not require an argument to a parameter switch
      // This gives flag-like behaviour
      ("headless,h", po::value<bool>()->zero_tokens(), "Headless mode -- Dont show images")
      ("min-good-matches,m", po::value<int>(&min_good_matches)->default_value(0), "The minimum amount of good matches which must be present to perform object recognition")
      ("matcher,a", po::value<std::string>(&matcher), "Choose the strategy for matching the extracted Descriptors. Possible Values: simple, nndr, symmetry_nndr")
      ("database-file,f", po::value<std::string>(&database_file), "Give a database file with training images and their keypoints/descriptors. By using a database, you don't need to specify your training images every time you call this node")
    ;

    // Use positional_options to allow the passing of test-img filenames without giving
    // the parameters explicitly. Like surf_keypoints testimg1 testimg2 instead of
    // surf_keypoints -i testimg1 -i testimg2 etc.
    po::positional_options_description p;
    p.add("test-img", -1);

    // po::store(po::parse_command_line(argc, argv, desc), vm); // Without positional options
    po::store(po::command_line_parser(argc, argv).
          options(desc).positional(p).run(), vm); 

    if (vm.count("help")) {
      cout << "Usage: surf_keypoints [OPTIONS] test-img1 test-img2..." << endl << endl;
      cout << desc << "\n";
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

    if(vm.count("test-img"))
    {
      test_images = vm["test-img"].as<std::vector<std::string> >();
    }

    // Check for valid values for matcher
    if (vm.count("matcher") && 
        (!(matcher == "simple" || matcher == "nndr" || matcher == "symmetry_nndr")))
            throw po::validation_error(po::validation_error::invalid_option_value, "matcher");

    if(vm.count("database-file"))
    {
      database_mode = true;
    }

    if(vm.count("train-img"))
    {
      if(vm["train-img"].as<std::vector<std::string> >().size()>1)
      {
        multi_train_mode = true;
      }else{
        // Take the first train-image in normal matching mode
        train_image = vm["train-img"].as<std::vector<std::string> >().at(0);
      }
    }else{
      // No train images given. Are we in database mode? Otherwise, abort program
      if(!database_mode)
            throw po::validation_error(po::validation_error::at_least_one_value_required, "matcher");

    }
    if(vm.count("headless"))
    {
      headless_mode = true;
      cout << "Running headless" << endl;
    }
    else
    {
      headless_mode = false;
      cout << "Running with GUI" << endl;
    }

  }
  catch(std::exception& e)
  {
    cout << "Usage: node [OPTIONS] test-img1 test-img2..." << endl << endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  ObjectMatcher om; // Create ObjectMatcher with SURF as default
  // ObjectMatcher om(detector,extractor);
  std::cout << "Starting with min_good_matches=" << min_good_matches << std::endl;
  om.setMinGoodMatches(min_good_matches);

  // Set the desired matcher, if the user picked one
  if(matcher=="simple")
  {
    om.setMatcher(new SimpleMatcher);
  } else if(matcher=="nndr")
  {
    om.setMatcher(new NNDRMatcher);
  } else if(matcher=="symmetry_nndr")
  {
    om.setMatcher(new SymmetryNNDRMatcher);
  }

  int positives = 0;
  if(database_mode)
  {
    cout << "Running in database mode" << endl;
    // Train images from file
		om.readTrainImagesFromDatabase(database_file);
  }
  else if(multi_train_mode)
  {
    // Train multiple images from the command line
    std::cout << "Running in multi training mode" << std::endl;
    om.trainImages(vm["train-img"].as<std::vector<std::string> >());
  }
  else
  {
    std::cout << "Running in single training mode" << std::endl;
  }
  std::cout << "Starting matching phase" << std::endl;
  // Run all test images against Training image
  for(int i=0;i<test_images.size();i++)
  {
    if(multi_train_mode || database_mode)
    {
      ObjectMatcher::ExecutionResult res = om.recognizeTrainedImages(test_images.at(i), headless_mode);
      if(res.object_recognized)
        positives++;
      if(!res.label.empty()){
        cout << "Recognized a object with the label " << res.label << endl;
      }
      else
      {
        cout << "Empty label" << endl;
      }
    }else{
      ObjectMatcher::ExecutionResult res = om.execute(train_image,test_images.at(i),headless_mode);
      if(res.object_recognized)
        positives++;
    }
    cout << endl;
  }
    std::cout << "Found " << positives << " positives in " << test_images.size() << " test images" << std::endl;

  return 0;
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
