/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
/**
 * @brief 单目SLAM初始化相关，双目和RGBD不会使用这个类
 */
class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    // 用reference frame来初始化，这个reference frame就是SLAM正式开始的第一帧
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    // 用current frame,也就是用SLAM逻辑上的第二帧来初始化整个SLAM，得到最开始两帧之间的R t,以及点云
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


private:

    // 假设场景为平面情况下通过前两帧求取Homography矩阵(current frame 2 到 reference frame 1),并得到该模型的评分
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    // 假设场景为非平面情况下通过前两帧求取Fundamental矩阵(current frame 2 到 reference frame 1),并得到该模型的评分
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    // 被FindHomography函数调用具体来算Homography矩阵
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    // 被FindFundamental函数调用具体来算Fundamental矩阵
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    // 被FindHomography函数调用，具体来算假设使用Homography模型的得分
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    // 被FindFundamental函数调用，具体来算假设使用Fundamental模型的得分
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    // 分解F矩阵，并从分解后的多个解中找出合适的R，t
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    // 分解H矩阵，并从分解后的多个解中找出合适的R，t
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    // 通过三角化方法，利用反投影矩阵将特征点恢复为3D点
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);
    // 归一化三维空间点和帧间位移t
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
    // ReconstructF调用该函数进行cheirality check，从而进一步找出F分解后最合适的解
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);
    // F矩阵通过结合内参可以得到Essential矩阵，该函数用于分解E矩阵，将得到4组解
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1; ///< 存储Reference Frame中的特征点

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2; ///< 存储Current Frame中的特征点

    // Current Matches from Reference to Current
    vector<Match> mvMatches12;///< Match的数据结构是pair,mvMatches12只记录Reference到Current匹配上的特征点对
    vector<bool> mvbMatched1;///< 记录Reference Frame的每个特征点在Current Frame是否有匹配的特征点

    // Calibration
    cv::Mat mK;///< 相机内参

    // Standard Deviation and Variance
    float mSigma, mSigma2;///< 测量误差

    // Ransac max iterations
    int mMaxIterations;///< 算Fundamental和Homography矩阵时RANSAC迭代次数

    // Ransac sets
    vector<vector<size_t> > mvSets;   ///< 二维容器，外层容器的大小为迭代次数，内层容器大小为每次迭代算H或F矩阵需要的点

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
