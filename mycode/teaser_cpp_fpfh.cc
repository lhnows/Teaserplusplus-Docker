// An example showing TEASER++ registration with FPFH features with the Stanford bunny model


#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>   // For std::floor (used in voxel grid, can be removed if only random)
#include <random>  // For std::random_device, std::mt19937, std::shuffle
#include <numeric> // For std::iota
#include <algorithm> // For std::shuffle, std::min

#include <Eigen/Core>

#include <teaser/ply_io.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>



const float DOWNSAMPLE_LEAF_SIZE = 0.2f; // 示例：20厘米
// 定义体素索引的类型，使用 std::tuple<int, int, int>
// 需要为 std::tuple 自定义哈希函数才能用于 std::unordered_set 或 std::unordered_map
// 这里为了简单，我们使用 std::set，它不需要自定义哈希。
using VoxelKey = std::tuple<int, int, int>;

inline double getAngularError(Eigen::Matrix3d R_exp, Eigen::Matrix3d R_est) {
  return std::abs(std::acos(fmin(fmax(((R_exp.transpose() * R_est).trace() - 1) / 2, -1.0), 1.0)));
}

// 不依赖PCL的简单体素格下采样函数
teaser::PointCloud downsamplePointCloud(const teaser::PointCloud& input_cloud, float leaf_size) {
    if (leaf_size <= 0) {
        std::cerr << "警告: leaf_size 无效 (" << leaf_size << ")，返回原始点云。" << std::endl;
        return input_cloud;
    }
    teaser::PointCloud downsampled_cloud;
    std::set<VoxelKey> occupied_voxels; // 使用 std::set 来存储已经有代表点的体素索引
    for (const auto& point : input_cloud) {
        // printf("1\n");
        VoxelKey voxel_key = {
            static_cast<int>(std::floor(point.x / leaf_size)),
            static_cast<int>(std::floor(point.y / leaf_size)),
            static_cast<int>(std::floor(point.z / leaf_size))
        };

        // 如果这个体素还没有被占据，则将当前点作为该体素的代表点加入下采样点云
        if (occupied_voxels.find(voxel_key) == occupied_voxels.end()) {
            downsampled_cloud.push_back(point);
            occupied_voxels.insert(voxel_key);
        }
    }
    return downsampled_cloud;
}


int main() {
  // Load the .ply file
  teaser::PLYReader reader;
  teaser::PointCloud src_cloud_original;
  auto status = reader.read("../../../waste.ply", src_cloud_original);
  int N = src_cloud_original.size();


  teaser::PLYReader reader2;
  teaser::PointCloud tgt_cloud_original;
  status = reader.read("../../../BIM.ply", tgt_cloud_original);


  
  // 1. 对加载的点云进行下采样
  std::cout << "正在对源点云进行下采样 (leaf_size: " << DOWNSAMPLE_LEAF_SIZE << ")..." << std::endl;
  teaser::PointCloud src_cloud = downsamplePointCloud(src_cloud_original, DOWNSAMPLE_LEAF_SIZE);
  // teaser::PointCloud src_cloud = downsamplePointCloudRandom(src_cloud_original, 50000);
  std::cout << "下采样后源点云剩余 " << src_cloud.size() << " 个点。" << std::endl;

  std::cout << "正在对目标点云进行下采样 (leaf_size: " << DOWNSAMPLE_LEAF_SIZE << ")..." << std::endl;
  teaser::PointCloud tgt_cloud = downsamplePointCloud(tgt_cloud_original, DOWNSAMPLE_LEAF_SIZE);
  // teaser::PointCloud tgt_cloud = downsamplePointCloudRandom(tgt_cloud_original, 50000);
  std::cout << "下采样后目标点云剩余 " << tgt_cloud.size() << " 个点。" << std::endl;
   
  if (src_cloud.empty() || tgt_cloud.empty()) {
      std::cerr << "错误：下采样后一个或两个点云为空。请检查 leaf_size 是否过大或原始点云是否有效。" << std::endl;
      return -1;
  }
    if (src_cloud.size() < 3 || tgt_cloud.size() < 3 ) { // FPFH 和 TEASER 都需要足够多的点
      std::cerr << "错误：下采样后点云点数过少，无法进行后续处理。" << std::endl;
      return -1;
  }


  // *** 新增：保存下采样后的点云 ***
  teaser::PLYWriter writer;
  std::string downsampled_src_filename = "downsampled_source.ply";
  std::string downsampled_tgt_filename = "downsampled_target.ply";

  std::cout << "正在保存下采样后的源点云到: " << downsampled_src_filename << std::endl;
  writer.write(downsampled_src_filename, src_cloud, false); // false表示不写二进制，写ASCII PLY

  std::cout << "正在保存下采样后的目标点云到: " << downsampled_tgt_filename << std::endl;
  writer.write(downsampled_tgt_filename, tgt_cloud, false);


  // 2. 计算FPFH特征 (在处理后的点云上进行)
  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  float normal_radius = 0.5f; // 假设0.5米作为法线估计半径
  float fpfh_radius = 1.5f;   // 假设1.5米作为FPFH计算半径
  std::cout << "正在计算FPFH特征 (法线半径: " << normal_radius
              << ", FPFH半径: " << fpfh_radius << ")..." << std::endl;
  auto source_descriptors = fpfh.computeFPFHFeatures(src_cloud, normal_radius, fpfh_radius);
  auto target_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, normal_radius, fpfh_radius);
  std::cout << "FPFH特征计算完毕。" << std::endl;
    std::cout << "源点云描述子数量: " << source_descriptors->size() << std::endl;
    std::cout << "目标点云描述子数量: " << target_descriptors->size() << std::endl;
  if (source_descriptors->empty() || target_descriptors->empty()) { /* ...错误处理... */ return -1; }

  // 3. 特征匹配
  teaser::Matcher matcher;
  bool use_mutual_correspondence = true;
    std::cout << "正在匹配特征 (使用相互对应检查: "
              << (use_mutual_correspondence ? "是" : "否") << ")..." << std::endl;
  auto correspondences = matcher.calculateCorrespondences(
      src_cloud, tgt_cloud, *source_descriptors, *target_descriptors, false, use_mutual_correspondence, false, 0.95);
  std::cout << "找到 " << correspondences.size() << " 个初始对应点对。" << std::endl;
  if (correspondences.size() < 3) { /* ...错误处理... */ return -1; }

  // 4. TEASER++ 配准参数
  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 1.5; //1.5米噪声水平 如果 TEASER++ 的结果包含很多明显的错误匹配（即旋转/平移结果很离谱），但它仍然认为找到了很多内点，那么 noise_bound 可能设置得过大了，尝试减小它。
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 0.005;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  std::cout << "开始使用 TEASER++ 进行配准..." << std::endl;
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  solver.solve(src_cloud, tgt_cloud, correspondences);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  auto solution = solver.getSolution();

  // Compare results
  std::cout << "=====================================" << std::endl;
  std::cout << "          TEASER++ Results           " << std::endl;
  std::cout << "=====================================" << std::endl;

  if (solution.valid) {
        std::cout << "Estimated rotation: " << std::endl;
        std::cout << solution.rotation << std::endl;
        std::cout << std::endl;
        std::cout << "Estimated translation: " << std::endl;
        std::cout << solution.translation << std::endl;
        std::cout << std::endl;


        Eigen::Matrix3d R = solution.rotation;
        Eigen::Vector3d t = solution.translation;
        double s = solution.scale; // 如果 params.estimate_scaling = true, 否则为1.0

        // *** 对下采样后的源点云进行变换并保存 ***
        teaser::PointCloud transformed_src_cloud;
        transformed_src_cloud.reserve(src_cloud.size()); // 预分配空间

        

        std::cout << "正在变换下采样后的源点云..." << std::endl;
        for (const auto& src_point_teaser : src_cloud) {
            Eigen::Vector3d src_point_eigen(src_point_teaser.x, src_point_teaser.y, src_point_teaser.z);
            Eigen::Vector3d transformed_point_eigen = s * R * src_point_eigen + t;
            transformed_src_cloud.push_back({
                static_cast<float>(transformed_point_eigen.x()),
                static_cast<float>(transformed_point_eigen.y()),
                static_cast<float>(transformed_point_eigen.z())
            });
        }

        std::string transformed_src_filename = "transformed_source_registered.ply";
        std::cout << "正在保存变换后的源点云到: " << transformed_src_filename << std::endl;
        writer.write(transformed_src_filename, transformed_src_cloud, false); // false表示写ASCII PLY

        // --- 新增：对原始的源点云 (src_cloud_original) 进行变换并保存 ---
        teaser::PointCloud transformed_src_cloud_original;
        transformed_src_cloud_original.reserve(src_cloud_original.size()); // 预分配空间

        std::cout << "正在变换原始的源点云..." << std::endl;
        for (const auto& src_point_teaser : src_cloud_original) { // 遍历原始点云
            Eigen::Vector3d src_point_eigen(src_point_teaser.x, src_point_teaser.y, src_point_teaser.z);
            // 使用相同的 R, t, s 进行变换
            Eigen::Vector3d transformed_point_eigen = s * R * src_point_eigen + t;
            transformed_src_cloud_original.push_back({
                static_cast<float>(transformed_point_eigen.x()),
                static_cast<float>(transformed_point_eigen.y()),
                static_cast<float>(transformed_point_eigen.z())
            });
        }
        std::string transformed_src_original_filename = "transformed_source_original_registered.ply";
        std::cout << "正在保存变换后的(原始)源点云到: " << transformed_src_original_filename << std::endl;
        writer.write(transformed_src_original_filename, transformed_src_cloud_original, false); // false表示写ASCII PLY



        std::cout << "\n配准完成。你可以比较以下文件：" << std::endl;
        std::cout << "1. 变换后的(下采样)源点云: " << transformed_src_filename << std::endl;
        std::cout << "2. 变换后的(原始)源点云: " << transformed_src_original_filename << std::endl;
        std::cout << "(也可以查看原始下采样版本: " << downsampled_tgt_filename << " 和 " << downsampled_src_filename << ")" << std::endl;


    } else {
        std::cout << "TEASER++ 未能找到有效的解。" << std::endl;
    }
    
    std::cout << "\n初始对应点对数量: " << correspondences.size() << std::endl;
    // 如果需要内点数量，可以查看 solution.inliers (如果API提供)
    // 例如： std::cout << "Number of inliers: " << solution.getInliers().size() << std::endl;
    // 具体API请查阅TEASER++文档

    std::cout << "Time taken (s): "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() /
                     1000000.0
              << std::endl;
    std::cout << "=====================================" << std::endl;
}
