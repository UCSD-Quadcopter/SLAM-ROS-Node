#include <std_msgs/Float32MultiArray.h"

template<typename Tp_, std::size_t r_, std::size_t c_>
struct mat 
{ 
	constexpr mat(Tp_* d) : data_(d) {}
	constexpr Tp_* data() { return data_; } 
	constexpr std::size_t rows() { return r_; }
	constexpr std::size_t cols() { return c_; }
	std_msgs::Float32MultiArray toF32MA() {
		Float32MultiArray fma;
		fma.data.clear();
		fma.data = data();
		return fma;
	}
	Tp_* data_;
};

template<std::size_t r_, std::size_t c_>
using matf = mat<float, r_, c_>;