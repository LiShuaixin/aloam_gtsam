#ifndef SRC_VISUALIZER_VISUALIZERWINDOW_H_
#define SRC_VISUALIZER_VISUALIZERWINDOW_H_

#include <SuMa/rv/Laserscan.h>
#include <SuMa/rv/ParameterList.h>
#include "SuMa/core/Preprocessing.h"
#include "SuMa/opengl/Model.h"

class PreprocessWorker {

public:
    
    PreprocessWorker(const PreprocessWorker& worker);
    PreprocessWorker(const rv::ParameterList& params);      
      
    /** \brief set parameters of the approach. **/
    void setParameters(const rv::ParameterList& params);

    /** \brief reset everything. **/
    void reset(const rv::ParameterList& params);
// 
//     /** \brief process a scan and update model, i.e., set data, pre-process it, updatePose, updateMap. **/
//     void processScan(const rv::Laserscan& scan);
// 
    uint32_t timestamp() const;
    
    uint32_t width() const;
    uint32_t height() const;
    
    /** \brief initialize point buffer, etc. **/
    void initialize(const rv::Laserscan& scan);
// 
//     /** \brief pre-process data, i.e., perform projection, etc. **/
//     void preprocess();

public:
//     Preprocessing preprocessor_;
    
    rv::ParameterList params_;

    uint32_t timestamp_{0};
    
    uint32_t width_, height_;
//     
    glow::GlBuffer<rv::Point3f> current_pts_{glow::BufferTarget::ARRAY_BUFFER, glow::BufferUsage::DYNAMIC_READ};
//    Frame::Ptr currentFrame_;
};

#endif /* SRC_VISUALIZER_VISUALIZERWINDOW_H_ */
