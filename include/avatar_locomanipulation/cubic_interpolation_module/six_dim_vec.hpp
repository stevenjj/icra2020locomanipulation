#include <avatar_locomanipulation/cubic_interpolation_module/six_dim.hpp>


class SixDimVec{
private:
	int N;
	
	std::string filename;

	std::vector<std::shared_ptr<SixDim> > seven_dim_vec;
public:
	SixDimVec();
	SixDimVec(const int & waypoint_length, const std::string & yaml_filename);

	~SixDimVec();

	void evaluate(const double & s_global);

	double clamp(const double & s_in);

	std::shared_ptr<SixDim> temp;

	Eigen::Quaterniond quat_out;

	Eigen::Vector3d pos_out;

	void convertToQuat();
};