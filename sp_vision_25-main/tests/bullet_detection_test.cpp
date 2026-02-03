#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "tasks/bullet_detector/aim_corrector.hpp"
#include "tasks/bullet_detector/detect_bullet.hpp"
#include "tools/coord_converter.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{config-path c  | configs/standard5.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                 | 视频起始帧下标    }"
  "{end-index e    | 0                 | 视频结束帧下标    }"
  "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";

// 全局参数
const double BULLET_V0 = 20.0;         // 弹丸初速度 (m/s)
const double BULLET_RESISTANCE_K = 0.010;  // 空气阻力系数
const double GRAVITY_ACCELERATION = 9.79;  // 重力加速度 (m/s²)
const double BULLET_RADIUS = 0.021;     // 弹丸半径 (m)

// 计算最佳发射时间
double find_best_fire_time(
	aimer::CoordConverter* converter,
	const aimer::aim::ImageBullet& detected_bullet,
	const Eigen::Vector3d& fire_direction,
	double bullet_speed) {
	
	// 将检测到的弹丸转换为现实坐标系
	cv::Point2f undistorted_center = converter->pd_to_pu(detected_bullet.center);
	// 使用默认距离 10.0 米（假设弹丸在这个距离附近）
	double assumed_distance = 10.0;
	Eigen::Vector3d detected_xyz_c = converter->pu_to_pc(undistorted_center, assumed_distance);
	Eigen::Vector3d detected_xyz_i = converter->pc_to_pi(detected_xyz_c);
	
	// 搜索最佳发射时间（过去0.5秒内）
	double best_fire_time = converter->get_img_t() - 0.25;
	double min_error = std::numeric_limits<double>::max();
	
	for (double t = converter->get_img_t() - 0.5; t < converter->get_img_t(); t += 0.01) {
		// 假设此时发射，计算弹丸当前位置（简化模型：直线飞行）
		double flight_time = converter->get_img_t() - t;
		Eigen::Vector3d predicted_pos = fire_direction * bullet_speed * flight_time;
		
		// 计算误差
		double error = (predicted_pos - detected_xyz_i).norm();
		if (error < min_error) {
			min_error = error;
			best_fire_time = t;
		}
	}
	
	return best_fire_time;
}

// 创建模拟弹丸
void create_simulated_bullet(
	aimer::aim::AimCorrector& aim_corrector,
	const double& fire_time,
	const Eigen::Vector3d& fire_direction,
	const double& target_distance,
	int bullet_id,
	double bullet_v0) {
	
	Eigen::Vector3d target_pos = fire_direction * target_distance;
	
	aimer::AimInfo aim_info;
	aim_info.shoot_param.v0 = bullet_v0;  // 弹丸初速度
	aim_info.shoot_param.aim_angle = 0.0;  // 水平发射
	aim_info.shoot_param.target_xyz_i_camera = target_pos;
	
	aimer::aim::IdTLatencyAimCorrection aim_correction;
	aim_correction.id = bullet_id;  // 使用唯一ID
	aim_correction.img_t = fire_time;
	aim_correction.img_to_predict_latency = 0.0;
	aim_correction.aim = aim_info;
	aim_correction.correction = Eigen::Vector2d(0.0, 0.0);
	
	aim_corrector.add_aim(aim_correction);
	aim_corrector.update_bullet_id(bullet_id);
}

// 计算弹丸与预测轨迹的偏差
Eigen::Vector3d calculate_bullet_error(
	aimer::CoordConverter& converter,
	const aimer::aim::ImageBullet& detected_bullet,
	const aimer::aim::IdPos& predicted_bullet) {
	
	// 将检测到的弹丸转换为现实坐标系
	cv::Point2f undistorted_center = converter.pd_to_pu(detected_bullet.center);
	// 使用默认距离 10.0 米（假设弹丸在这个距离附近）
	double assumed_distance = 10.0;
	Eigen::Vector3d detected_xyz_c = converter.pu_to_pc(undistorted_center, assumed_distance);
	Eigen::Vector3d detected_xyz_i = converter.pc_to_pi(detected_xyz_c);
	
	// 计算偏差
	return detected_xyz_i - predicted_bullet.pos;
}

int main(int argc, char * argv[]) {
	// 读取命令行参数
	cv::CommandLineParser cli(argc, argv, keys);
	if (cli.has("help")) {
		cli.printMessage();
		return 0;
	}
	auto input_path = cli.get<std::string>(0);
	auto config_path = cli.get<std::string>("config-path");
	auto start_index = cli.get<int>("start-index");
	auto end_index = cli.get<int>("end-index");

	tools::Plotter plotter;
	tools::Exiter exiter;

	auto video_path = fmt::format("{}.avi", input_path);
	auto text_path = fmt::format("{}.txt", input_path);
	cv::VideoCapture video(video_path);
	std::ifstream text(text_path);

	// 初始化自瞄相关组件
	auto_aim::YOLO yolo(config_path);
	auto_aim::Solver solver(config_path);
	auto_aim::Tracker tracker(config_path, solver);
	auto_aim::Aimer aimer(config_path);

	// 初始化弹丸检测相关组件
	aimer::CoordConverter converter(config_path);
	aimer::aim::DoReproj do_reproj(
		converter.get_f_cv_mat_ref(),
		converter.get_rot_ic_sup_cv_mat_ref()
	);
	aimer::aim::DetectBullet bullet_detector(do_reproj);
	aimer::aim::AimCorrector aim_corrector(&converter);

	// 计算发射方向（由相机与枪管外参决定）
	Eigen::Vector3d fire_direction = Eigen::Vector3d(1.0, 0.0, 0.0);  // 默认相机正前方

	cv::Mat img, drawing;
	auto t0 = std::chrono::steady_clock::now();

	auto_aim::Target last_target;
	io::Command last_command;
	double last_t = -1;

	video.set(cv::CAP_PROP_POS_FRAMES, start_index);
	for (int i = 0; i < start_index; i++) {
		double t, w, x, y, z;
		text >> t >> w >> x >> y >> z;
	}

	for (int frame_count = start_index; !exiter.exit(); frame_count++) {
		if (end_index > 0 && frame_count > end_index) break;

		video.read(img);
		if (img.empty()) break;

		double t, w, x, y, z;
		text >> t >> w >> x >> y >> z;
		auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

		/// 自瞄核心逻辑

		solver.set_R_gimbal2world({w, x, y, z});

		auto yolo_start = std::chrono::steady_clock::now();
		auto armors = yolo.detect(img, frame_count);

		auto tracker_start = std::chrono::steady_clock::now();
		auto targets = tracker.track(armors, timestamp);

		auto aimer_start = std::chrono::steady_clock::now();
		auto command = aimer.aim(targets, timestamp, 27, false);

		if (
			!targets.empty() && aimer.debug_aim_point.valid &&
			std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
			command.shoot = true;

		if (command.control) last_command = command;
		/// 调试输出

		auto finish = std::chrono::steady_clock::now();
		tools::logger()->info(
			"[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
			tools::delta_time(tracker_start, yolo_start) * 1e3,
			tools::delta_time(aimer_start, tracker_start) * 1e3,
			tools::delta_time(finish, aimer_start) * 1e3);

		tools::draw_text(
			img,
			fmt::format(
					"command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
					command.pitch * 57.3, command.shoot),
			{10, 60}, {154, 50, 205});

		Eigen::Quaternion gimbal_q = {w, x, y, z};
		tools::draw_text(
			img,
			fmt::format(
					"gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
			{10, 90}, {255, 255, 255});

		nlohmann::json data;

		// 装甲板原始观测数据
		data["armor_num"] = armors.size();
		if (!armors.empty()) {
			const auto & armor = armors.front();
			data["armor_x"] = armor.xyz_in_world[0];
			data["armor_y"] = armor.xyz_in_world[1];
			data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
			data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
			data["armor_center_x"] = armor.center_norm.x;
			data["armor_center_y"] = armor.center_norm.y;
		}

		Eigen::Quaternion q{w, x, y, z};
		auto yaw = tools::eulers(q, 2, 1, 0)[0];
		data["gimbal_yaw"] = yaw * 57.3;
		data["cmd_yaw"] = command.yaw * 57.3;
		data["shoot"] = command.shoot;

		if (!targets.empty()) {
			auto target = targets.front();

			if (last_t == -1) {
				last_target = target;
				last_t = t;
				continue;
			}

			std::vector<Eigen::Vector4d> armor_xyza_list;

			// 当前帧target更新后
			armor_xyza_list = target.armor_xyza_list();
			for (const Eigen::Vector4d & xyza : armor_xyza_list) {
				auto image_points = 
					solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
				tools::draw_points(img, image_points, {0, 255, 0});
			}

			// aimer瞄准位置
			auto aim_point = aimer.debug_aim_point;
			Eigen::Vector4d aim_xyza = aim_point.xyza;
			auto image_points = 
				solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
			if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});

			// 观测器内部数据
			Eigen::VectorXd x = target.ekf_x();
			data["x"] = x[0];
			data["vx"] = x[1];
			data["y"] = x[2];
			data["vy"] = x[3];
			data["z"] = x[4];
			data["vz"] = x[5];
			data["a"] = x[6] * 57.3;
			data["w"] = x[7];
			data["r"] = x[8];
			data["l"] = x[9];
			data["h"] = x[10];
			data["last_id"] = target.last_id;

			// 卡方检验数据
			data["residual_yaw"] = target.ekf().data.at("residual_yaw");
			data["residual_pitch"] = target.ekf().data.at("residual_pitch");
			data["residual_distance"] = target.ekf().data.at("residual_distance");
			data["residual_angle"] = target.ekf().data.at("residual_angle");
			data["nis"] = target.ekf().data.at("nis");
			data["nees"] = target.ekf().data.at("nees");
			data["nis_fail"] = target.ekf().data.at("nis_fail");
			data["nees_fail"] = target.ekf().data.at("nees_fail");
			data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
		}

		/// 弹丸检测与匹配
		// 设置当前时间
		converter.set_img_t(t);
		
		// 检测弹丸
		Eigen::Quaterniond q_gimbal = {w, x, y, z};
		std::vector<aimer::aim::ImageBullet> detected_bullets = 
			bullet_detector.process_new_frame(img, q_gimbal);
		
		// 处理检测到的弹丸
		if (!detected_bullets.empty()) {
			tools::logger()->info("[{}] Detected {} bullets", frame_count, detected_bullets.size());
			
			// 存储每个检测弹丸的信息和偏差
			struct BulletWithError {
				aimer::aim::ImageBullet bullet;
				double error;
				Eigen::Vector3d error_vec;
				int id;
			};
			std::vector<BulletWithError> bullets_with_error;
			
			// 对每个检测弹丸计算最佳发射时间并创建模拟弹丸
			for (int i = 0; i < detected_bullets.size(); i++) {
				aimer::aim::ImageBullet detected = detected_bullets[i];
				
				// 动态计算最佳发射时间
				double best_fire_time = find_best_fire_time(
					&converter, detected, fire_direction, BULLET_V0);
				
				// 创建模拟弹丸（使用唯一ID）
				int bullet_id = 999 + i;  // 不同弹丸使用不同ID
				create_simulated_bullet(aim_corrector, best_fire_time, fire_direction, 10.0, bullet_id, BULLET_V0);
				
				// 弹丸匹配和误差计算
				aim_corrector.sample_aim_errors();
				
				// 获取预测的弹丸位置
				std::vector<aimer::aim::IdPos> predicted_bullets = aim_corrector.get_bullets();
				
				// 计算当前检测弹丸与对应预测弹丸的偏差
				if (!predicted_bullets.empty()) {
					// 找到对应ID的预测弹丸
					for (const auto& pred : predicted_bullets) {
						if (pred.id == bullet_id) {
							// 计算偏差
							Eigen::Vector3d error_vec = calculate_bullet_error(
									converter, detected, pred);
							double error = error_vec.norm();
							
							// 存储结果
							bullets_with_error.push_back({detected, error, error_vec, bullet_id});
							break;
						}
					}
				}
			}
			
			// 筛选偏差最小的弹丸
			if (!bullets_with_error.empty()) {
				// 按偏差排序
				std::sort(bullets_with_error.begin(), bullets_with_error.end(),
									[](const BulletWithError& a, const BulletWithError& b) {
											return a.error < b.error;
									});
				
				// 获取最优弹丸
				BulletWithError best_bullet = bullets_with_error[0];
				
				// 输出最优弹丸的偏差
				tools::logger()->info(
					"[{}] Best bullet error: {:.3f} meters (ID: {}), Error vector: [{:.3f}, {:.3f}, {:.3f}]", 
					frame_count, best_bullet.error, best_bullet.id,
					best_bullet.error_vec.x(), best_bullet.error_vec.y(), best_bullet.error_vec.z());
				
				// 在图像上标记最优弹丸
				cv::circle(img, best_bullet.bullet.center, best_bullet.bullet.radius, cv::Scalar(0, 255, 0), 2);
				tools::draw_text(
					img,
					fmt::format("Error: {:.3f}", best_bullet.error),
					{best_bullet.bullet.center.x + 10, best_bullet.bullet.center.y - 10},
					{0, 255, 0});
				
				// 存储弹丸数据
				data["bullet_detected"] = true;
				data["bullet_count"] = detected_bullets.size();
				data["best_bullet_error"] = best_bullet.error;
				data["best_bullet_error_x"] = best_bullet.error_vec.x();
				data["best_bullet_error_y"] = best_bullet.error_vec.y();
				data["best_bullet_error_z"] = best_bullet.error_vec.z();
			}
		} else {
			tools::logger()->info("[{}] No bullets detected", frame_count);
			data["bullet_detected"] = false;
			data["bullet_count"] = 0;
		}

		plotter.plot(data);

		cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
		cv::imshow("reprojection", img);
		auto key = cv::waitKey(30);
		if (key == 'q') break;
	}

	return 0;
}
