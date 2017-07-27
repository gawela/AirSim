// Copyright (c) Autonomous Systems Lab, ETH Zurich, Abel Gawel.
// Licensed under the MIT License.

#include <iostream>
#include <chrono>
#include "rpc/RpcLibClient.hpp"
#include "controllers/DroneControllerBase.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/Common.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

class RandomPointPoseGenerator {
public:
private:
	typedef common_utils::RandomGeneratorGaussianF RandomGeneratorGaussianF;
	typedef msr::airlib::Vector3r Vector3r;
	typedef msr::airlib::Quaternionr Quaternionr;
	typedef common_utils::Utils Utils;
	typedef msr::airlib::VectorMath VectorMath;

public:
	Vector3r position;
	Quaternionr orientation;

public:
	RandomPointPoseGenerator(int random_seed)
		:
		//settings are for neighbourhood environement
		//sigma = desired_max / 2 so 95% of the times we in desired
		rand_xy_(0.0f, 75.0f), rand_z_(2.0f, 1.0f),
		rand_pitch_(0.0f, M_PIf / 8), rand_roll_(0.0f, M_PIf / 16),
		rand_yaw_(0.0f, M_PIf / 2)
	{
		rand_xy_.seed(random_seed);
		rand_z_.seed(random_seed);
		rand_pitch_.seed(random_seed);
		rand_yaw_.seed(random_seed);
	}

	void next()
	{
		position.x() = rand_xy_.next();
		position.y() = rand_xy_.next();
		position.z() = Utils::clip(rand_z_.next(), -10.0f, -1.0f);

		float pitch = Utils::clip(rand_pitch_.next(), -M_PIf / 2, M_PIf / 2);
		float roll = Utils::clip(rand_pitch_.next(), -M_PIf / 4, M_PIf / 4);
		float yaw = Utils::clip(rand_yaw_.next(), -M_PIf, M_PIf);

		orientation = VectorMath::toQuaternion(pitch, roll, yaw);
	}
private:
	RandomGeneratorGaussianF rand_xy_, rand_z_, rand_pitch_, rand_yaw_, rand_roll_;
};

void readMatrixFromFile(const std::string& filename, Eigen::MatrixXf* out_matrix) {
	std::ifstream indata;
	indata.open(filename);
	std::string line;
	std::vector<float> values;
	int rows = 0;
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ',')) {
			values.push_back(std::stof(cell));
		}
		++rows;
	}
	//out_matrix->resize(rows, values.size() / rows);
	(*out_matrix) = Eigen::Map<const Eigen::Matrix<typename Eigen::MatrixXf::Scalar, Eigen::MatrixXf::RowsAtCompileTime, Eigen::MatrixXf::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size() / rows);
}

int main() 
{
    using namespace msr::airlib;


    // This assumes you are running DroneServer already on the same machine.
    // DroneServer must be running first.
    msr::airlib::RpcLibClient client;
    typedef DroneControllerBase::ImageRequest ImageRequest;
    typedef VehicleCameraBase::ImageResponse ImageResponse;
    typedef VehicleCameraBase::ImageType_ ImageType_;
    typedef common_utils::FileSystem FileSystem;
    
	// Read in waypoints from file.
	Eigen::MatrixXf waypoints;
	std::string filename = "C:/Users/gawela/Documents/neighbourhood/waypoints_neighbourhood_downward.csv";
	readMatrixFromFile(filename, &waypoints);
	std::cout << "Num waypoints: " << waypoints.rows() << " " << waypoints.cols() << std::endl;

	// Send drone to waypoint after qaypoint and record data at each location.
    try {
        client.confirmConnection();
		std::cout << "Connection confirmed." << std::endl;

		for (int i = 0; i < waypoints.rows(); ++i) {
			std::cout << "Choosing destrination." << std::endl;
			Eigen::Matrix<float, 1, 7> next_destination = waypoints.row(i);
			std::cout << "Flying to next location: " << next_destination << std::endl;
			// Send drone to position.
			msr::airlib::Vector3r destination_translation(next_destination(0), next_destination(1), next_destination(2));
			client.simSetPose(destination_translation, msr::airlib::Quaternionr(next_destination(3), next_destination(4), next_destination(5), next_destination(6)));
			// Wait for the drone to arrive.
			// todo(gawela) Check that Drone is in place.
			//std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
			// Take images of scene.
			std::cout << "Taking images." << std::endl;
			std::vector<ImageRequest> request = { ImageRequest(0, ImageType_::Scene), ImageRequest(0, ImageType_::Depth),  ImageRequest(0, ImageType_::Segmentation) };
			std::vector<ImageResponse> response = client.simGetImages(request);
			std::cout << "Done." << std::endl;
			// Save all data to disk (pose from waypoints and images).
			std::cout << "Saving image to file." << std::endl;
			std::string path = "D:/airsim_datasets/neighbourhood_downward/";
			int j = 0;
			for (const ImageResponse& image_info : response) {
				std::cout << "Image size: " << image_info.image_data.size() << std::endl;
				if (path != "") {
					if (j == 0) {
						std::ofstream file(FileSystem::combine(path, "rgb_" + std::to_string(i) + ".png"), std::ios::binary);
						file.write(reinterpret_cast<const char*>(image_info.image_data.data()), image_info.image_data.size());
						file.close();
						++j;
					} else if (j == 1) {
						std::ofstream file(FileSystem::combine(path, "depth_" + std::to_string(i) + ".png"), std::ios::binary);
						file.write(reinterpret_cast<const char*>(image_info.image_data.data()), image_info.image_data.size());
						file.close();
						++j;
					} else if (j == 2) {
						std::ofstream file(FileSystem::combine(path, "segmentation_" + std::to_string(i) + ".png"), std::ios::binary);
						file.write(reinterpret_cast<const char*>(image_info.image_data.data()), image_info.image_data.size());
						file.close();
						++j;
					}
				}
			}
			std::cout << "Done." << std::endl;
			//std::this_thread::sleep_for(std::chrono::duration<double>(0.5));




		}
		

    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}
