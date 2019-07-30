#include <avatar_locomanipulation/cubic_interpolation_module/six_dim.hpp>
#include <avatar_locomanipulation/helpers/param_handler.hpp>

class SixDimVec{
private:
	int N; // Number of waypoints extracted from the yaml file
	std::string filename;
	std::vector<std::shared_ptr<SixDim> > six_dim_vec;

	Eigen::Vector3d aa;
	Eigen::Vector3d axis;

	double s_ = 0.0;
	double s_local = 0.0;
	double smin = 0.0;
	double smax = 1.0;
	double angle;

public:
	SixDimVec();
	SixDimVec(const std::string & filename_input);

	~SixDimVec();

	void evaluate(const double & s_global);
	void getPose(const double & s_global, Eigen::Vector3d & position_out, Eigen::Quaterniond & orientation_out);


	double clamp(const double & s_in);

	std::shared_ptr<SixDim> temp;

	Eigen::Quaterniond quat_out;

	Eigen::Vector3d pos_out;

	void convertToQuat();
};