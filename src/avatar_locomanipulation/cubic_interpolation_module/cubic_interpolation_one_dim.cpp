#include <avatar_locomanipulation/cubic_interpolation_module/cubic_interpolation_one_dim.hpp>
#include <avatar_locomanipulation/helpers/pseudo_inverse.hpp>


CubicInterpolationOneDim::CubicInterpolationOneDim(){
	std::cout << "One Dimensional Cubic Interpolation Created" << std::endl;
}
	

CubicInterpolationOneDim::CubicInterpolationOneDim(const std::vector<double> & waypoints){
	wp1 = waypoints[0];
	wp2 = waypoints[1];
	wp3 = waypoints[2];
	wp4 = waypoints[3];
	interpolate();
}

CubicInterpolationOneDim::~CubicInterpolationOneDim(){

}

void CubicInterpolationOneDim::interpolate(){
	Eigen::Vector4d w, a;
	Eigen::MatrixXd B, Binv;

	B = Eigen::MatrixXd::Zero(4,4);
	Binv = Eigen::MatrixXd::Zero(4,4);

	unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
	std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> > svd_B;
	svd_B = std::unique_ptr< Eigen::JacobiSVD<Eigen::MatrixXd> >( new Eigen::JacobiSVD<Eigen::MatrixXd>(4, 4, svdOptions) );
	double singular_values_threshold = 1e-4;

	a << a0, a1, a2, a3;
	w << wp1, wp2, wp3, wp4;

	B << 1, 0, 0, 0,
		 1, 0.33, (std::pow(0.33,2)), (std::pow(0.33,3)),
		 1, 0.67, (std::pow(0.67,2)), (std::pow(0.67,3)),
		 1, 1, 1, 1;

	math_utils::pseudoInverse(B, *svd_B, Binv, singular_values_threshold, Eigen::ComputeThinU | Eigen::ComputeThinV);	

	a = Binv * w;
	a0 = a[0];
	a1 = a[1];
	a2 = a[2];
	a3 = a[3];
	// std::cout << "(a0, a1, a2, a3) = (" << a0 << ", " << a1 << ", " << a2 << ", " << a3 << ")" << std::endl;
}


double CubicInterpolationOneDim::evaluate(const double & s_in){
	double s_ = s_in;

	return (a0 + a1*s_ + a2*(std::pow(s_,2)) + a3*(std::pow(s_,3)));
}