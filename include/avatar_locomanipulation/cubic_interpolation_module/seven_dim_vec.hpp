#include <avatar_locomanipulation/cubic_interpolation_module/seven_dim.hpp>


class SevenDimVec{
private:
	int N;
	
	std::string filename;

	std::shared_ptr<SevenDim> temp;

	std::vector<std::shared_ptr<SevenDim> > seven_dim_vec;
public:
	SevenDimVec();
	SevenDimVec(const int & waypoint_length, const std::string & yaml_filename);

	~SevenDimVec();

	void evaluate(const double & s_global);

	double clamp(const double & s_in);
};