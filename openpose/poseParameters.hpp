#ifndef OPENPOSE_POSE_POSE_PARAMETERS_HPP
#define OPENPOSE_POSE_POSE_PARAMETERS_HPP

#include <map>
#include <array>
#include <memory> // std::shared_ptr, std::unique_ptr
#include <string>
#include <vector>
//#include <openpose/core/common.hpp>
//#include <openpose/pose/enumClasses.hpp>



#define BODY_25 0
#define COCO_18 1         /**< COCO model + neck, with 18+1 components (see poseParameters.hpp for details). */
#define MPI_15 2          /**< MPI model, with 15+1 components (see poseParameters.hpp for details). */
#define MPI_15_4 3        /**< Variation of the MPI model, reduced number of CNN stages to 4: faster but less accurate.*/
#define BODY_19 4         /**< Experimental. Do not use. */
#define BODY_19_X2 5      /**< Experimental. Do not use. */
#define BODY_19N 6        /**< Experimental. Do not use. */
#define BODY_25E 7        /**< Experimental. Do not use. */
#define CAR_12 8          /**< Experimental. Do not use. */
#define BODY_25D 9        /**< Experimental. Do not use. */
#define BODY_23 10        /**< Experimental. Do not use. */
#define CAR_22 11         /**< Experimental. Do not use. */
#define BODY_19E 12       /**< Experimental. Do not use. */
#define BODY_25B 13       /**< Experimental. Do not use. */
#define BODY_135 14       /**< Experimental. Do not use. */

namespace op
{
    // Constant Global Parameters
    // For OpenCL-NMS in Ubuntu, (POSE_MAX_PEOPLE+1)*3(x,y,score) must be divisible by 32. Easy fix:
    // POSE_MAX_PEOPLE = 32n - 1
    // For OpenCL-NMS in Windows, it must be by 64, so 64n - 1
    const auto POSE_MAX_PEOPLE = 127u;

    // Model functions
    const std::map<unsigned int, std::string>& getPoseBodyPartMapping(const int poseModel);
    const std::string& getPoseProtoTxt(const int poseModel);
    const std::string& getPoseTrainedModel(const int poseModel);
    unsigned int getPoseNumberBodyParts(const int poseModel);
    const std::vector<unsigned int>& getPosePartPairs(const int poseModel);
    const std::vector<unsigned int>& getPoseMapIndex(const int poseModel);
    unsigned int getPoseMaxPeaks();
    float getPoseNetDecreaseFactor(const int poseModel);
    unsigned int poseBodyPartMapStringToKey(const int poseModel, const std::string& string);
    unsigned int poseBodyPartMapStringToKey(const int poseModel, const std::vector<std::string>& strings);

    // Default NSM and body connector parameters
    float getPoseDefaultNmsThreshold(const int poseModel, const bool maximizePositives = false);
    float getPoseDefaultConnectInterMinAboveThreshold(const bool maximizePositives = false);
    float getPoseDefaultConnectInterThreshold(const int poseModel, const bool maximizePositives = false);
    unsigned int getPoseDefaultMinSubsetCnt(const bool maximizePositives = false);
    float getPoseDefaultConnectMinSubsetScore(const bool maximizePositives = false);
    bool addBkgChannel(const int poseModel);
}

#endif // OPENPOSE_POSE_POSE_PARAMETERS_HPP
