/*
*/

#include "circleParametersOptimization.hpp"

CircleParametersOptimization::CircleParametersOptimization()
{ 
}

CircleParametersOptimization::~CircleParametersOptimization(){
}

int CircleParametersOptimization::getCircleParametersFromPoints(const  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &circlePoints, 
        double &rXCenterCircle, double &rYCenterCircle, double &rRadiusCircle)
{			  	
	/*
	*/

	// Check input
	if (circlePoints.size() < 4) {
        ROS_WARN_STREAM("HOUGH CIRCLE PARAMS OPTMIZER TOOL: Input list of points to find the fitting circle has size: "
			<< circlePoints.size() << ". At least 4 points needed.");
        return 1;
    }

	// double x1;
	// double x2;
	// double x3;
	// Eigen::MatrixXd AFill(3, length);
	Eigen::MatrixXd matrix(circlePoints.size(), 3);
	// Eigen::VectorXd AFirst(length);
	// Eigen::VectorXd ASec(length);
	Eigen::VectorXd matrixFirst(circlePoints.size());
	Eigen::VectorXd matrixSecond(circlePoints.size());
	// Eigen::VectorXd ASquaredRes(length);
	Eigen::VectorXd circleFunction(circlePoints.size());
	Eigen::VectorXd output(3);
	// int ok = 0;

	// Define the matrix of points
	for (int i = 0; i < circlePoints.size(); i++)
	{
		matrix(i, 0) = circlePoints[i](0);
		matrix(i, 1) = circlePoints[i](1);
		matrix(i, 2) = 1;
	}

	// 
	for (int i = 0; i < circlePoints.size(); i++)
	{
		matrixFirst(i) = matrix(i, 0) * matrix(i, 0);
		matrixSecond(i) = matrix(i, 1) * matrix(i, 1);
	}

	// for (int i = 0; i < length; i++)
	// {
	// 	AFirstSquared(i) = AFirst(i) * AFirst(i);
	// 	ASecSquared(i) = ASec(i) * ASec(i);
	// }

	circleFunction = matrixFirst + matrixSecond;

	output = matrix.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(circleFunction);

	rXCenterCircle = output(0) * 0.5;
	rYCenterCircle = output(1) * 0.5;
	rRadiusCircle = sqrt((output(0) * output(0) + output(1) * output(1)) / 4 + output(2));

	return 0;
}