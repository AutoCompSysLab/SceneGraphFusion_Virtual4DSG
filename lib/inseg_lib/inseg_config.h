#ifndef INSEG_LIB_INSEG_CONFIG_H_
#define INSEG_LIB_INSEG_CONFIG_H_

namespace inseg_lib {

struct InSegConfig {
    // 동적 포인트 제거 비활성화 (제약 완화)
    bool remove_dynamic_point = false; 

    // Surfels이 무효화되기까지의 프레임 수를 크게 설정 (제약 완화)
    int surfel_invalid_frame_threshold = 100; 

    // Surfels이 안정화되기 위한 프레임 수를 최소화 (제약 완화)
    int surfel_stable_frame_threshold = 1; 

    // 맵 업데이트 단계에서 정상 벡터의 각도 임계값을 크게 설정 (더 많은 포인트 허용)
    float update_point_angle_threshold = 180.0f; 

    // 정상 벡터의 내적 값 임계값을 낮게 설정 (제약 완화)
    float update_point_dot_product_threshold = -1.0f; 

    // 라벨 병합 신뢰도 임계값을 낮게 설정 (더 쉽게 병합)
    int label_merge_confidence_threshold = 1; 

    // 라벨 병합을 위한 겹침 비율 임계값을 낮게 설정 (더 쉽게 병합)
    float label_merge_overlap_ratio = 0.0f; 

    // 최소 라벨 크기 임계값을 낮게 설정 (노이즈 허용)
    int label_min_size = 1; 

    // 레이블 전파 단계에서 점의 정상 벡터 임계값을 높게 설정 (더 많은 점 허용)
    float label_point_dot_product_threshold = 180.0f; 

    // 깊이 엣지 임계값을 넓게 설정 (더 느슨한 분할)
    float depth_edge_threshold = 0.5f; 

    // 라벨 병합을 위한 최소 라벨 크기 임계값 낮게 설정 (노이즈 허용)
    int label_size_threshold = 1; 

    // 라벨 병합을 위한 지역 비율 낮게 설정 (더 쉽게 병합)
    float label_region_ratio = 0.0f; 
};
// // Settings for the InSegReconstruction class.
// struct InSegConfig {
//     // Defines if dynamic points should be removed.
//     // This is useful if the scene is not static.
//     // Might remove noisy depth input on object borders.
//     bool remove_dynamic_point = false;

//     // Threshold when a surfel is set to invalid (in frames).
//     int surfel_invalid_frame_threshold = 30;

//     // Frame threshold when a point is considered to be stable.
//     int surfel_stable_frame_threshold = 2;

//     // Angle threshold of normal vectors in the map update stage.
//     // Point are considered invalid if angle (degrees) is higher than threshold.
//     float update_point_angle_threshold = 75;

//     // Threshold for the dot product of the normal of the surfels.
//     // If dot product is less than the threshold we have a valid normal.
//     float update_point_dot_product_threshold = 30;

//     // Confidence value frame threshold when two label are merged.
//     // This happens when they are when overlapping.
//     int label_merge_confidence_threshold = 3; 

//     // Overlap threshold to identify correct segments in propagation step.
//     // If two labels overlap by this ratio they are merged.
//     float label_merge_overlap_ratio = 0.3; // 0.2

//     // Smalles size of a segment to be labled.
//     // This threshold helps to remove noise.
//     int label_min_size = 10;
    
//     // This angle threshold is used to determine segment correspondences in the
//     // label propagation step. If vertices are close (along viewing ray) and the
//     // angle between the corresponding normals are lower than this threshold
//     // the segments are considered to overlap.
//     float label_point_dot_product_threshold = 10;
    
//     // Threshold for the dot product of edges in segmentation step.
//     // This defines how the surfaces are segmented.
// //    float depth_edge_threshold = 0.98f; // segments_0 0.992f
//     //TODO: add a IO for this
//     float depth_edge_threshold = 0.98f;// for 3RScan
// //    float depth_edge_threshold = 0.90f;// for ScanNet

//     // Label size threshold used for the merging process.
//     int label_size_threshold = 15;

//     // Label region ratio for the merging process.
//     // If two labels overlap by this ratio they will be merged.
//     float label_region_ratio = 0.3;
// };
}  // namespace inseg_lib

#endif  // INSEG_LIB_INSEG_CONFIG_H_
