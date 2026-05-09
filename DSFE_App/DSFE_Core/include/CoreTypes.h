// DSFE_Core CoreTypes.h
struct Vec3d {
	double x, y, z;
};

struct Vec4d {
	double x, y, z, w;
};

struct VecXd {
	std::vector<double> data;
};

struct Transform {
	Vec3d position;
	Vec3d rotation;
};