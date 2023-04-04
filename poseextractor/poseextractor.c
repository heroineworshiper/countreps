




void poseExtractor(const std::vector<const float*>& sourcePtrs)
{
    std::vector<const T*> sourcePtrs(bottom.size());
    for (auto i = 0u ; i < sourcePtrs.size() ; i++)
        sourcePtrs[i] = bottom[i]->gpu_data();
    resizeAndMergeGpu(top.at(0)->mutable_gpu_data(), 
        sourcePtrs, 
        mTopSize, 
        mBottomSizes,
        mScaleRatios);
}




