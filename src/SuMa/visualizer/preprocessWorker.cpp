#include <SuMa/visualizer/preprocessWorker.h>

using namespace rv;
using namespace glow;

PreprocessWorker::PreprocessWorker(const rv::ParameterList& params) 
    : width_(params["data_width"])
    , height_(params["data_height"])
//    ,currentFrame_(new Frame(width_, height_))
//     ,  preprocessor_(params)
{
    std::printf("creating preprocess worker!!\n");
    setParameters(params);
    std::printf("construction done!!\n");
}

uint32_t PreprocessWorker::height() const { return height_; }

uint32_t PreprocessWorker::width() const { return width_; }

uint32_t PreprocessWorker::timestamp() const { return timestamp_; }

void PreprocessWorker::setParameters(const ParameterList& const_params)
{
    rv::ParameterList params = const_params;
    
    width_ = params["data_width"];
    height_ = params["data_height"];
    std::cout << "depth image width set as " << width_ << ", height set as " << height_ << std::endl;
    
    params_ = params;
    
    // preprocessor_.setParameters(params);
    std::printf("set params for preprocessor.\n");

}

void PreprocessWorker::reset(const ParameterList& params)
{
//    currentFrame_ = std::make_shared<Frame>(width_, height_);
    timestamp_ = 0;
    
    setParameters(params);
}

// void PreprocessWorker::processScan(const Laserscan& scan)
// {
//     initialize(scan);
// 
//     preprocess();
// 
//     timestamp_ += 1;
// }

void PreprocessWorker::initialize(const Laserscan& scan)
{
    current_pts_.assign(scan.points());
}

// void PreprocessWorker::preprocess()
// {
// 
//     preprocessor_.process(current_pts_, *currentFrame_);
// }



