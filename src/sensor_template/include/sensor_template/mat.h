#pragma once

template<typename Tp_, std::size_t r_, std::size_t c_>
struct mat 
{ 
	constexpr mat(Tp_* d) : data_(d)/*, r_(r), c_(c)*/ {}
	constexpr Tp_* data() const { return data_; } 
	constexpr std::size_t rows() const { return r_; }
	constexpr std::size_t cols() const { return c_; }
	operator std_msgs::Float32MultiArray() const {
		std_msgs::Float32MultiArray fma;
		fma.data.clear();
		fma.data = std::vector<float>(data(), data()+rows()*cols());
        fma.layout.dim.resize(2);
        fma.layout.dim[0].label = "rows";
        fma.layout.dim[0].size = rows();
        fma.layout.dim[0].stride = rows() * cols();
        fma.layout.dim[1].label = "cols";
        fma.layout.dim[1].size = cols();
        fma.layout.dim[1].stride = cols();
		return fma;
	}
	Tp_* data_;
    //std::size_t r_, c_;
};

template <std::size_t r_, std::size_t c_>
using matf = mat<float, r_, c_>;