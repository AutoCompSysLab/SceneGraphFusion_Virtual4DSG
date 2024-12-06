//
// Created by jw on 11/24/24.
//
#include "../include/dataLoader/dataloader_virtual4dsg.h"
#include "../include/dataLoader/util.h"
#include <ORUtils/Logging.h>

#include <utility>
#include <dataLoader/datasetVirtual4DSG.h>
#include <ORUtils/PathTool.hpp>

using namespace PSLAM;

static const std::vector<std::string> split(const std::string s, const std::string delim) {
    std::vector<std::string> list;
    auto start = 0U;
    auto end = s.find(delim);
    while (true) {
        list.push_back(s.substr(start, end - start));
        if (end == std::string::npos)
            break;
        start = end + delim.length();
        end = s.find(delim, start);
    }
    return list;
}

static bool LoadInfoIntrinsics(const std::string& filename,
                               const bool depth_intrinsics,
                               CameraParameters& intrinsics) {
    const std::string search_tag = depth_intrinsics ? "m_calibrationDepthIntrinsic" : "m_calibrationColorIntrinsic";
    const std::string search_tag_w = depth_intrinsics? "m_depthWidth":"m_colorWidth";
    const std::string search_tag_h = depth_intrinsics? "m_depthHeight":"m_colorHeight";
    const std::string search_tag_p = "m_projectionMatrix";
    std::string line{""};
    std::ifstream file(filename);
    int width,height;
    // float fx,fy,cx,cy;
    double fx,fy,cx,cy;
    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    if (file.is_open()) {
        while (std::getline(file,line)) {
            if (line.rfind(search_tag_w, 0) == 0)
                width = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
                // height = std::stoi(line.substr(line.find("= ")+2, std::string::npos)); 
            else if (line.rfind(search_tag_h, 0) == 0)
                height = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
                // width = std::stoi(line.substr(line.find("= ")+2, std::string::npos)); // for Virtual4DSG
            else if (line.rfind(search_tag, 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                const auto parts = split(model, " ");
                // fx = std::stof(parts[0]);
                // fy = std::stof(parts[5]);
                // cx = std::stof(parts[2]);
                // cy = std::stof(parts[6]);
                fx = std::stod(parts[0]);
                fy = std::stod(parts[5]);
                cx = std::stod(parts[2]);
                cy = std::stod(parts[6]);
            }
            else if (line.rfind(search_tag_p, 0) == 0) {
                const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                const auto parts = split(model, " ");
                // proj(0, 0) = std::stod(parts[0]);
                // proj(2, 0) = std::stod(parts[2]);   // column-major
                // proj(1, 1) = std::stod(parts[5]);
                // proj(2, 1) = std::stod(parts[6]);
                // proj(2, 2) = std::stod(parts[10]); // ConvertLHtoRH
                // proj(3, 2) = std::stod(parts[11]);
                // proj(2, 3) = std::stod(parts[14]);
                proj(0, 0) = std::stod(parts[0]);
                proj(0, 2) = std::stod(parts[2]);   // column-major
                proj(1, 1) = std::stod(parts[5]);
                proj(1, 2) = std::stod(parts[6]);
                proj(2, 2) = std::stod(parts[10]); // ConvertLHtoRH
                proj(2, 3) = std::stod(parts[11]);
                proj(3, 2) = std::stod(parts[14]);
            }
        }
        file.close();
        // intrinsics.Set(width,height,fx,fy,cx,cy,1.f);
        intrinsics.Set(width,height,fx,fy,cx,cy,proj,1.f);
        return true;
    }

    return false;
}

DatasetLoader_Virtual4DSG::DatasetLoader_Virtual4DSG(std::shared_ptr<DatasetDefinitionBase> dataset):
        DatasetLoader_base(std::move(dataset)) {
    if(!LoadInfoIntrinsics(m_dataset->folder+"_info.txt",true,m_cam_param_d))
        throw std::runtime_error("unable to open _info file");
    if(!LoadInfoIntrinsics(m_dataset->folder+"_info.txt",false,m_cam_param_rgb))
        throw std::runtime_error("unable to open _info file");
}

const std::string DatasetLoader_Virtual4DSG::GetFileName(const std::string& folder,
                                                    const std::string& subfolder,
                                                    const std::string& prefix,
                                                    const std::string& suffix,
                                                    int number_length = -1) const {
    std::stringstream filename;
    const std::string path = (folder == "/" ? "" : folder) +
                             (subfolder == "/" ? "" : subfolder) +
                             (prefix == "/" ? "" : prefix);
    if (number_length < 0)
        filename << path << (suffix == "/" ? "" : suffix);
    else
        filename << path << std::setfill('0') << std::setw(number_length) << frame_index << (suffix == "/" ? "" : suffix);
    std::string s(filename.str());
    return s;
}


bool DatasetLoader_Virtual4DSG::Retrieve() {
    int max_attempts = 3;
    int attempt = 0;
    bool isExist = false;
    std::string depthFilename, colorFilename;

    while (attempt < max_attempts) {
        depthFilename = GetFileName(m_dataset->folder,
                                                m_dataset->folder_depth,
                                                m_dataset->prefix_depth,
                                                m_dataset->suffix_depth,
                                                m_dataset->number_length);
        colorFilename = GetFileName(m_dataset->folder,
                                                m_dataset->folder_rgb,
                                                m_dataset->prefix_rgb,
                                                m_dataset->suffix_rgb,
                                                m_dataset->number_length);
        pose_file_name_ = GetFileName(m_dataset->folder,
                                    m_dataset->folder_pose,
                                    m_dataset->prefix_pose,
                                    m_dataset->suffix_pose,
                                    m_dataset->number_length);
        isExist = isFileExist(depthFilename);
        if (isExist) {
            break; // Exit loop if file is found
        }
        attempt++;
        frame_index += m_dataset->frame_index_counter; // Move to the next file
        SCLOG(VERBOSE) << "Attempt " << attempt << ": Cannot find path:\n" << depthFilename << "\n" << colorFilename;
    }

    if (!isExist) {
        frame_index = 0;
        SCLOG(VERBOSE) << "Failed to retrieve data after " << max_attempts << " attempts.";

        return false;
    }

    // m_d = cv::imread(depthFilename, -1);
    // // mask depth
    // for(size_t c=0;c<(size_t)m_d.cols;++c){
    //     for(size_t r=0;r<(size_t)m_d.rows;++r){
    //         if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
    //             m_d.at<unsigned short>(r,c) = 0;
    //     }
    // }
    // m_d = cv::imread(depthFilename, cv::IMREAD_ANYDEPTH);
    // int type = m_d.type();
    m_d = cv::imread(depthFilename, cv::IMREAD_ANYDEPTH);
    m_d = m_d * 1000.0f; // m -> mm
    // mask depth
    for(size_t c=0;c<(size_t)m_d.cols;++c){
        for(size_t r=0;r<(size_t)m_d.rows;++r){
            if(m_d.at<float>(r,c)>=m_dataset->max_depth)
                m_d.at<float>(r,c) = 0;
    }
    }

    if (isFileExist(colorFilename.c_str())) {
        m_rgb = cv::imread(colorFilename, -1);
    }

    LoadPose(m_pose, pose_file_name_, m_dataset->rotate_pose_img);
    frame_index += m_dataset->frame_index_counter; 
       
    if (m_dataset->rotate_pose_img) {
        cv::rotate(m_d, m_d, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::rotate(m_rgb, m_rgb, cv::ROTATE_90_COUNTERCLOCKWISE); 
        float angle_degrees = -90.0f;
        float angle_radians = angle_degrees * M_PI / 180.0f;
        Eigen::Matrix4f rotationZ = DatasetLoader_Virtual4DSG::rotation_matrix_Z(angle_radians);
        m_pose = m_pose * rotationZ;
    }

    if (m_dataset->convert_coordinate) {
        // Eigen::Matrix4f ConvertLHToRH = Eigen::Matrix4f::Identity();
        // ConvertLHToRH(1, 1) = -1;  // Y축 반전
        // ConvertLHToRH(2, 2) = -1;  // Z축 반전
        Eigen::Matrix4f ConvertLHToRH;
        // ConvertLHToRH << 1,  0,  0,  0, // Z-up 일 때
        //                  0,  0,  1,  0,
        //                  0,  1,  0,  0,
        //                  0,  0,  0,  1;
        ConvertLHToRH << -1,  0,  0,  0, // Y-up 일 때
                         0,  1,  0,  0,
                         0,  0, -1,  0,
                         0,  0,  0,  1;
        m_pose = ConvertLHToRH * m_pose * ConvertLHToRH.inverse(); // 행렬의 기하학적 성질을 올바르게 적용하기 위해 앞뒤로 곱함.
        cv::flip(m_rgb, m_rgb, 1); // x축 반전된 pose와 맞추기 위해 좌우반전
        cv::flip(m_d, m_d, 1); 

    }
    // std::cout << "Converted pose\n"<< m_pose << "\n";

    // cv::imshow("m_rgb", m_rgb);
    // cv::imshow("m_d", m_d);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    return true;
}



void DatasetLoader_Virtual4DSG::Reset() {
    frame_index = 0;
}

Eigen::Matrix<float,4,4> DatasetLoader_Virtual4DSG::rotation_matrix_Z(const float rot) {
    Eigen::Matrix<float,4,4> res = Eigen::Matrix<float,4,4>::Identity();
    res << cos(rot), -sin(rot), 0, 0,
            sin(rot), cos(rot), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return res;
}