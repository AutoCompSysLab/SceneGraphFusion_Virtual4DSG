//
// Created by jw on 11/06/24.
//

#ifndef LIBSURFELRECONSTRUCTION_DATASETVIRTUAL4DSG_H
#define LIBSURFELRECONSTRUCTION_DATASETVIRTUAL4DSG_H

#include "dataset_base.h"

namespace PSLAM {
    class Virtual4DSGDataset : public DatasetDefinitionBase {
    public:
        Virtual4DSGDataset(INPUTE_TYPE type, const std::string &path)  {
            datasetType = type;
            rotate_pose_img = false;
            folder = path;
            frame_index_counter = 1;
            number_length = 1;
            prefix_pose = "Action_";
            prefix_depth = "Action_";
            prefix_rgb = "Action_";

            suffix_depth = "_0_depth.exr";
            suffix_rgb = "_0_normal.png";
            suffix_pose = "_0_pose.txt";

            min_pyr_level = 3;
            number_pose = 4;
            number_length = 4;
        }
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATASETVIRTUAL4DSG_H
