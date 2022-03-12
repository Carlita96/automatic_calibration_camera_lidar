/*
This class can be used to remove outliers from lists of observations in 3D and 2D.
*/

#include "removeOutliersListTool.hpp"

RemoveOutliersListTool::RemoveOutliersListTool()
{ 
}

RemoveOutliersListTool::~RemoveOutliersListTool(){
}

int RemoveOutliersListTool::removeOutliers3d3d(const std::vector<Eigen::Vector3d> list1,
            const std::vector<Eigen::Vector3d> list2, const double kTimesStdDev, std::vector<Eigen::Vector3d> &rOutputList1,
            std::vector<Eigen::Vector3d> &rOutputList2) 
{
    /*
    Method that checks 2 lists and removes outliers from both.
    * Input parameters: 
        - list1: list 1 to remove outliers from.
        - list2: list 2 to remove outliers from.
		- kTimesStdDev: the observations not in: [mean - kTimesStdDev * stdDev, mean + kTimesStdDev * stdDev] are deleted
        - rOutputList1: list 1 with outliers removed.
        - rOutputList2: list 2 with outliers removed.
    */
    auto start = std::chrono::system_clock::now();
    // Check input 
    if (list1.size() != list2.size()) {
        ROS_ERROR_STREAM("REMOVE OUTLIERS FROM LIST TOOL: Number of items in list 1 (" << list1.size() 
        << ") and number of items in list 2 (" << list2.size() << ") is not the same. Same number is needed.");
        return 1;
    }
	int outliersFound = 0;
	double k = kTimesStdDev;

	// LIST 1
	// Calculate the mean and standard deviation of each column in list 1
	std::vector<Eigen::Vector3d> list1Filter1;
    std::vector<Eigen::Vector3d> list2Filter1;

	double meanParam1List1;
	double meanParam2List1;
	double meanParam3List1;
	double stdDevParam1List1;
	double stdDevParam2List1;
	double stdDevParam3List1;
	getMeanAndStdDev3d(list1, meanParam1List1, meanParam2List1, meanParam3List1, stdDevParam1List1, stdDevParam2List1, stdDevParam3List1);

	// Remove outliers
	int i;
	for (auto it = std::begin(list1); it != std::end(list1); ++it) {
        i = it - list1.begin();
		if (((list1[i][0] > meanParam1List1 - k *stdDevParam1List1) && (list1[i][0] < meanParam1List1 + k *stdDevParam1List1)) 
					&& ((list1[i][1] > meanParam2List1 - k *stdDevParam2List1) && (list1[i][1] < meanParam2List1 + k *stdDevParam2List1)) 
					&& ((list1[i][2] > meanParam3List1 - k *stdDevParam3List1) && (list1[i][2] < meanParam3List1 + k *stdDevParam3List1))) {
			list1Filter1.push_back(list1[i]);
			list2Filter1.push_back(list2[i]);
		} else {
			outliersFound++;
		}
	}

	// LIST 2
	// Calculate the mean and standard deviation of each column in list 1
	std::vector<Eigen::Vector3d> list1Filter2;
    std::vector<Eigen::Vector3d> list2Filter2;

	double meanParam1List2;
	double meanParam2List2;
	double meanParam3List2;
	double stdDevParam1List2;
	double stdDevParam2List2;
	double stdDevParam3List2;
	getMeanAndStdDev3d(list2Filter1, meanParam1List2, meanParam2List2, meanParam3List2, stdDevParam1List2, stdDevParam2List2, stdDevParam3List2);

	// Remove outliers
	for (auto it = std::begin(list2Filter1); it != std::end(list2Filter1); ++it) {
        i = it - list2Filter1.begin();
		if (((list2Filter1[i][0] > meanParam1List2 - k * stdDevParam1List2) && (list2Filter1[i][0] < meanParam1List2 + k *stdDevParam1List2)) 
					&& ((list2Filter1[i][1] > meanParam2List2 - k *stdDevParam2List2) && (list2Filter1[i][1] < meanParam2List2 + k *stdDevParam2List2)) 
					&& ((list2Filter1[i][2] > meanParam3List2 - k *stdDevParam3List2) && (list2Filter1[i][2] < meanParam3List2 + k *stdDevParam3List2))) {
			list1Filter2.push_back(list1Filter1[i]);
			list2Filter2.push_back(list2Filter1[i]);
		} else {
			outliersFound++;
		}
	}

	// Save output
	rOutputList1 = list1Filter2;
	rOutputList2 = list2Filter2;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;


    ROS_DEBUG_STREAM("REMOVE OUTLIERS FROM LIST TOOL: Removing outliers from list done in " << duration.count() << " seconds. "
		<< outliersFound << " number of outliers removed.");
    return 0;
}

int RemoveOutliersListTool::removeOutliers3d2d(const std::vector<Eigen::Vector3d> list1,
            const std::vector<Eigen::Vector2d> list2, const double kTimesStdDev, std::vector<Eigen::Vector3d> &rOutputList1,
            std::vector<Eigen::Vector2d> &rOutputList2) 
{
    /*
    Method that checks 2 lists and removes outliers from both.
    * Input parameters: 
        - list1: list 1 to remove outliers from.
        - list2: list 2 to remove outliers from.
        - rOutputList1: list 1 with outliers removed.
        - rOutputList2: list 2 with outliers removed.
    */
    auto start = std::chrono::system_clock::now();
    // Check input 
    if (list1.size() != list2.size()) {
        ROS_ERROR_STREAM("REMOVE OUTLIERS FROM LIST TOOL: Number of items in list 1 (" << list1.size() 
        << ") and number of items in list 2 (" << list2.size() << ") is not the same. Same number is needed.");
        return 1;
    }
	int outliersFound = 0;
	double k = kTimesStdDev;

	// LIST 1
	// Calculate the mean and standard deviation of each column in list 1
	std::vector<Eigen::Vector3d> list1Filter1;
    std::vector<Eigen::Vector2d> list2Filter1;

	double meanParam1List1;
	double meanParam2List1;
	double meanParam3List1;
	double stdDevParam1List1;
	double stdDevParam2List1;
	double stdDevParam3List1;
	getMeanAndStdDev3d(list1, meanParam1List1, meanParam2List1, meanParam3List1, stdDevParam1List1, stdDevParam2List1, stdDevParam3List1);

	// Remove outliers
	int i;
	for (auto it = std::begin(list1); it != std::end(list1); ++it) {
        i = it - list1.begin();
		if (((list1[i][0] > meanParam1List1 - k* stdDevParam1List1) && (list1[i][0] < meanParam1List1 + k* stdDevParam1List1)) 
					&& ((list1[i][1] > meanParam2List1 - k* stdDevParam2List1) && (list1[i][1] < meanParam2List1 + k* stdDevParam2List1)) 
					&& ((list1[i][2] > meanParam3List1 - k* stdDevParam3List1) && (list1[i][2] < meanParam3List1 + k* stdDevParam3List1))) {
			list1Filter1.push_back(list1[i]);
			list2Filter1.push_back(list2[i]);
		} else {
			outliersFound++;
		}
	}

	// LIST 2
	// Calculate the mean and standard deviation of each column in list 1
	std::vector<Eigen::Vector3d> list1Filter2;
    std::vector<Eigen::Vector2d> list2Filter2;

	double meanParam1List2;
	double meanParam2List2;
	double stdDevParam1List2;
	double stdDevParam2List2;
	getMeanAndStdDev2d(list2Filter1, meanParam1List2, meanParam2List2, stdDevParam1List2, stdDevParam2List2);

	// Remove outliers
	for (auto it = std::begin(list2Filter1); it != std::end(list2Filter1); ++it) {
        i = it - list2Filter1.begin();
		if (((list2Filter1[i][0] > meanParam1List2 - k* stdDevParam1List2) && (list2Filter1[i][0] < meanParam1List2 + k* stdDevParam1List2)) 
					&& ((list2Filter1[i][1] > meanParam2List2 - k* stdDevParam2List2) && (list2Filter1[i][1] < meanParam2List2 + k* stdDevParam2List2))) {
			list1Filter2.push_back(list1Filter1[i]);
			list2Filter2.push_back(list2Filter1[i]);
		} else {
			outliersFound++;
		}
	}

	// Save output
	rOutputList1 = list1Filter2;
	rOutputList2 = list2Filter2;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("REMOVE OUTLIERS FROM LIST TOOL: Removing outliers from list done in " << duration.count() << " seconds. "
		<< 0 << " number of outliers removed.");
    return 0;
}

void RemoveOutliersListTool::getMeanAndStdDev3d(const std::vector<Eigen::Vector3d> list,
		double &rMeanParam1, double &rMeanParam2, double &rMeanParam3, double &rStdDevParam1, double &rStdDevParam2, double &rStdDevParam3)
{
    /*
    Method that calculates the mean and standard deviation of each column in a list.
    * Input parameters: 
        - list: list where the mean and stadard deviation will be calculated.
    */

	// Mean
	double meanParam1List = 0;
	double meanParam2List = 0;
	double meanParam3List = 0;
	int i;
	for (auto it = std::begin(list); it != std::end(list); ++it) {
        i = it - list.begin();
		meanParam1List += list[i][0];
		meanParam2List += list[i][1];
		meanParam3List += list[i][2];
	}
	meanParam1List = meanParam1List / list.size();
	meanParam2List = meanParam2List / list.size();
	meanParam3List = meanParam3List / list.size();
	
	// Std dev
	double stdDevParam1List = 0;
	double stdDevParam2List = 0;
	double stdDevParam3List = 0;
	for (auto it = std::begin(list); it != std::end(list); ++it) {
        i = it - list.begin();
		stdDevParam1List += pow(list[i][0] - meanParam1List, 2);
		stdDevParam2List += pow(list[i][1] - meanParam2List, 2);
		stdDevParam3List += pow(list[i][2] - meanParam3List, 2);
	}
	stdDevParam1List = sqrt(stdDevParam1List / list.size());
	stdDevParam2List = sqrt(stdDevParam2List / list.size());
	stdDevParam3List = sqrt(stdDevParam3List / list.size());

	// Save params
	rMeanParam1 = meanParam1List;
	rMeanParam2 = meanParam2List;
	rMeanParam3 = meanParam3List;
	rStdDevParam1 = stdDevParam1List;
	rStdDevParam2 = stdDevParam2List;
	rStdDevParam3 = stdDevParam3List;
} 

void RemoveOutliersListTool::getMeanAndStdDev2d(const std::vector<Eigen::Vector2d> list,
		double &rMeanParam1, double &rMeanParam2, double &rStdDevParam1, double &rStdDevParam2)
{
    /*
    Method that calculates the mean and standard deviation of each column in a list.
    * Input parameters: 
        - list: list where the mean and stadard deviation will be calculated.
    */

	// Mean
	double meanParam1List = 0;
	double meanParam2List = 0;
	int i;
	for (auto it = std::begin(list); it != std::end(list); ++it) {
        i = it - list.begin();
		meanParam1List += list[i][0];
		meanParam2List += list[i][1];
	}
	meanParam1List = meanParam1List / list.size();
	meanParam2List = meanParam2List / list.size();
	
	// Std dev
	double stdDevParam1List = 0;
	double stdDevParam2List = 0;
	for (auto it = std::begin(list); it != std::end(list); ++it) {
        i = it - list.begin();
		stdDevParam1List += pow(list[i][0] - meanParam1List, 2);
		stdDevParam2List += pow(list[i][1] - meanParam2List, 2);
	}
	stdDevParam1List = sqrt(stdDevParam1List / list.size());
	stdDevParam2List = sqrt(stdDevParam2List / list.size());

	// Save params
	rMeanParam1 = meanParam1List;
	rMeanParam2 = meanParam2List;
	rStdDevParam1 = stdDevParam1List;
	rStdDevParam2 = stdDevParam2List;
} 