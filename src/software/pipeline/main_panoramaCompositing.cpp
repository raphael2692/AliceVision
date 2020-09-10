// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image stuff
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

// Logging stuff
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = boost::filesystem;

typedef struct {
  size_t offset_x;
  size_t offset_y;
  std::string img_path;
  std::string mask_path;
  std::string weights_path;
} ConfigView;

template <int C, int TILESIZE>
bool create_tiled_image_float(const std::string & filename, unsigned int width, unsigned int height) {
  
  if (width % TILESIZE) {
    return false;
  }

  if (height % TILESIZE) {
    return false;
  }

  std::unique_ptr<float> empty_data(new float[TILESIZE * TILESIZE * C]);
  std::memset(empty_data.get(), 0, TILESIZE * TILESIZE * C * sizeof(float));

  oiio::ImageSpec spec(width, height, C, oiio::TypeDesc::FLOAT);
  spec.tile_width = TILESIZE;
  spec.tile_height = TILESIZE;
  spec.attribute("compression", "piz");

  std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(filename);
  if (out == nullptr) {
    return false;
  }

  out->open(filename, spec);
  for (int i = 0; i < height; i+=TILESIZE) {
    for (int j = 0; j < width; j+=TILESIZE) {
      out->write_tile(j, i, 0, oiio::TypeDesc::FLOAT, empty_data.get());
    }
  }
  out->close();

  return true;
}

class Compositer {
public:
  Compositer(oiio::ImageCache * cache) : 
  _cache(cache)
  {
  }

  virtual bool initialize(const std::string filename, size_t outputWidth, size_t outputHeight) {
    bool res = create_tiled_image_float<4, 256>(filename, outputWidth, outputHeight);
    if (!res) {
      return false;
    }

    
    _filename = filename;

    return true;
  }

  virtual bool append(const aliceVision::image::Image<image::RGBfColor> & color, const aliceVision::image::Image<unsigned char> & inputMask, const aliceVision::image::Image<float> & inputWeights, size_t offset_x, size_t offset_y) {

    image::Image<image::RGBAfColor> dest_color(color.Width(), color.Height());

    image::RGBAfColor * buffer = dest_color.data();

    std::cout << "ok" << std::endl;

    std::cout << _cache->resolve_filename(_filename.string()) << std::endl;

    int data = 5;
    bool test = _cache->get_image_info(oiio::ustring("panorama.exr"), 0, 0, oiio::ustring("exists"), oiio::TypeDesc::FLOAT, (void*)&data);
    std::cout << test << std::endl;

    std::cout << data << std::endl;
    /*if (!_cache->get_pixels(_filename, 0, 0, 0, 10, 0, 10, 0, 1, 0, 2, oiio::TypeDesc::FLOAT, (float*)&buffer)) {
      return false;
    }
    std::cout << "ok2" << std::endl;*/

    /*for (size_t i = 0; i < color.Height(); i++) {

      size_t pano_i = offset_y + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < color.Width(); j++) {

        if (!inputMask(i, j)) {
          continue;
        }

        size_t pano_j = offset_x + j;
        if (pano_j >= _panorama.Width()) {
          pano_j = pano_j - _panorama.Width();
        }

        _panorama(pano_i, pano_j).r() = color(i, j).r();
        _panorama(pano_i, pano_j).g() = color(i, j).g();
        _panorama(pano_i, pano_j).b() = color(i, j).b();
        _panorama(pano_i, pano_j).a() = 1.0f;
      }
    }*/

    return true;
  }

  virtual bool terminate() {
    return true;
  }

protected:
  oiio::ImageCache * _cache;
  oiio::ustring _filename;
};

int aliceVision_main(int argc, char **argv)
{
  std::string sfmDataFilepath;
  std::string warpingFolder;
  std::string outputPanorama;

  std::string compositerType = "multiband";
  std::string overlayType = "none";
  bool useGraphCut = true;
  bool showBorders = false;
  bool showSeams = false;

  system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

  // Program description
  po::options_description allParams (
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
    "AliceVision PanoramaCompositing"
  );

  // Description of mandatory parameters
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
    ("warpingFolder,w", po::value<std::string>(&warpingFolder)->required(), "Folder with warped images.")
    ("output,o", po::value<std::string>(&outputPanorama)->required(), "Path of the output panorama.");
  allParams.add(requiredParams);

  // Description of optional parameters
  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("compositerType,c", po::value<std::string>(&compositerType)->required(), "Compositer Type [replace, alpha, multiband].")
    ("overlayType,c", po::value<std::string>(&overlayType)->required(), "Overlay Type [none, borders, seams, all].")
    ("useGraphCut,c", po::value<bool>(&useGraphCut)->default_value(useGraphCut), "Do we use graphcut for ghost removal ?");
  allParams.add(optionalParams);

  // Setup log level given command line
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
  allParams.add(logParams);


  // Effectively parse command line given parse options
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  // Set verbose level given command line
  system::Logger::get()->setLogLevel(verboseLevel);


  // load input scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::EXTRINSICS|sfmDataIO::INTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
    return EXIT_FAILURE;
  }

  oiio::ImageCache * cache = oiio::ImageCache::create();
  cache->attribute("max_memory_MB", 8000.0f);
  cache->attribute("autotile", 256);

  std::pair<int, int> panoramaSize;
  {
      const IndexT viewId = *sfmData.getValidViews().begin();
      const std::string viewFilepath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
      ALICEVISION_LOG_TRACE("Read panorama size from file: " << viewFilepath);

      oiio::ParamValueList metadata = image::readImageMetadata(viewFilepath);
      panoramaSize.first = metadata.find("AliceVision:panoramaWidth")->get_int();
      panoramaSize.second = metadata.find("AliceVision:panoramaHeight")->get_int();

      if(panoramaSize.first == 0 || panoramaSize.second == 0)
      {
        oiio::ImageCache::destroy(cache);
        ALICEVISION_LOG_ERROR("The output panorama size is empty.");
        return EXIT_FAILURE;
      }
      ALICEVISION_LOG_INFO("Output panorama size set to " << panoramaSize.first << "x" << panoramaSize.second);
  }

  std::unique_ptr<Compositer> compositer;
  bool isMultiBand = false;
  compositer = std::unique_ptr<Compositer>(new Compositer(cache));

  if (!compositer->initialize("panorama.exr", panoramaSize.first, panoramaSize.second)) 
  {
    oiio::ImageCache::destroy(cache);
    ALICEVISION_LOG_ERROR("Error while initializing compositer");
    return EXIT_FAILURE;
  }

  // Compute seams
  std::vector<std::shared_ptr<sfmData::View>> viewsToDraw;
  for (auto& viewIt : sfmData.getViews())
  {
    if(!sfmData.isPoseAndIntrinsicDefined(viewIt.second.get()))
    {
        // skip unreconstructed views
        continue;
    }
    
    viewsToDraw.push_back(viewIt.second);
  }
  

  oiio::ParamValueList outputMetadata;

  // Do compositing
  for (const auto & view : viewsToDraw)
  {
    IndexT viewId = view->getViewId();

    if(!sfmData.isPoseAndIntrinsicDefined(view.get()))
    {
        // skip unreconstructed views
        continue;
    }

    // Load image and convert it to linear colorspace
    const std::string imagePath = (fs::path(warpingFolder) / (std::to_string(viewId) + ".exr")).string();
    ALICEVISION_LOG_INFO("Load image with path " << imagePath);
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

    oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
    if(outputMetadata.empty())
    {
        // the first one will define the output metadata (random selection)
        outputMetadata = metadata;
    }
    const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int();
    const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int();

    // Load mask
    const std::string maskPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_mask.exr")).string();
    ALICEVISION_LOG_INFO("Load mask with path " << maskPath);
    image::Image<unsigned char> mask;
    image::readImage(maskPath, mask, image::EImageColorSpace::NO_CONVERSION);

    // Load Weights
    const std::string weightsPath = (fs::path(warpingFolder) / (std::to_string(viewId) + "_weight.exr")).string();
    ALICEVISION_LOG_INFO("Load weights with path " << weightsPath);
    image::Image<float> weights;
    image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);
    
    compositer->append(source, mask, weights, offsetX, offsetY);
  }

  // Build image
  compositer->terminate();
  
  


  // Remove Warping-specific metadata
  /*outputMetadata.remove("AliceVision:offsetX");
  outputMetadata.remove("AliceVision:offsetY");
  outputMetadata.remove("AliceVision:panoramaWidth");
  outputMetadata.remove("AliceVision:panoramaHeight");*/

  // Store output
  /*ALICEVISION_LOG_INFO("Write output panorama to file " << outputPanorama);
  const aliceVision::image::Image<image::RGBAfColor> & panorama = compositer->getPanorama();
  image::writeImage(outputPanorama, panorama, image::EImageColorSpace::AUTO, outputMetadata);*/

  oiio::ImageCache::destroy(cache);

  return EXIT_SUCCESS;
}
