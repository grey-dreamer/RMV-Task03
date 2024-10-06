#include "windmill.hpp"
#include <ceres/ceres.h>

using namespace std;
using namespace cv;

struct TrigFunctionCost
{
    TrigFunctionCost(double time, double angle)
        : time_(time), angle_(angle) {}

    template <typename T>
    bool operator()(const T *const A_over_omega,
                    const T *const omega,
                    const T *const phi,
                    const T *const b,
                    const T *const c,
                    T *residuals) const
    {
        T model_angle = (*A_over_omega) * sin((*omega) * time_ + (*phi)) + (*b) * time_ + (*c);
        residuals[0] = model_angle - T(angle_);
        return true;
    }

    double time_;
    double angle_;
};

int main()
{
    const int numRuns = 10;           // 运行次数
    vector<double> runTimes(numRuns); // 存储每次运行的时间
    for (int run = 0; run < numRuns; ++run)
    {
        auto start = std::chrono::high_resolution_clock::now(); // 开始计时
        std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        WINDMILL::WindMill wm(t.count());
        cv::Mat src;

        const double pi = M_PI;
        vector<double> a;
        vector<double> times;
        double t0 = (double)t.count();

        // 初始化参数
        double A_over_omega = 1;
        double omega = 2.884;
        double phi = 0.65;
        double b = 2.305;
        double c = 1;

        int cnt = 0;

        while (1)
        {
            cnt++;
            t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            src = wm.getMat((double)t.count() / 1000);

            //==========================代码区========================//
            Point R, H;
            // 识别、绘制中心点
            // 将图像另存为 8 位单通道灰度图像
            Mat src2;
            if (src.type() != CV_8UC1)
            {
                cvtColor(src, src2, COLOR_BGR2GRAY);
            }

            // 查找轮廓
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy; // 保存层级信息
            cv::findContours(src2, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

            // 变量存储面积最小和第二小的外轮廓索引及其面积
            int minIndex = -1, secondMinIndex = -1;
            double minArea = std::numeric_limits<double>::max(), secondMinArea = std::numeric_limits<double>::max();

            // 遍历所有轮廓，找到面积最小和第二小的外轮廓
            for (size_t i = 0; i < contours.size(); ++i)
            {
                // 检查当前轮廓是否为外轮廓（父轮廓索引为-1）
                if (hierarchy[i][3] == -1)
                {
                    double area = cv::contourArea(contours[i]);
                    if (area < minArea)
                    {
                        secondMinArea = minArea;
                        secondMinIndex = minIndex;
                        minArea = area;
                        minIndex = i;
                    }
                    else if (area < secondMinArea && area > minArea)
                    {
                        secondMinArea = area;
                        secondMinIndex = i;
                    }
                }
            }
            // 如果找到了面积最小的外轮廓
            if (minIndex != -1)
            {
                // 获取面积最小外轮廓的边界框
                cv::Rect boundingBox = cv::boundingRect(contours[minIndex]);

                // 计算面积最小外轮廓中心点
                cv::Point minCenter(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
                R = minCenter;
                // 在面积最小外轮廓中心点绘制圆点
                cv::circle(src, minCenter, 4, cv::Scalar(0, 255, 0), -1);
            }
            // 如果找到了面积第二小的外轮廓
            if (secondMinIndex != -1)
            {
                // 检查该外轮廓是否有内轮廓
                for (size_t i = 0; i < contours.size(); ++i)
                {
                    // 检查当前轮廓是否为第二小外轮廓的子轮廓
                    if (hierarchy[i][3] == static_cast<int>(secondMinIndex))
                    {
                        cv::Rect boundingBox = cv::boundingRect(contours[i]);

                        cv::Point center(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
                        H = center;

                        cv::circle(src, center, 4, cv::Scalar(0, 255, 0), -1);
                    }
                }
            }

            // 迭代拟合
            double relative_time = ((double)t.count() - t0) / 1000; // 相对于开始时间的时间差
            times.push_back(relative_time);                         // 存储相对时间
            double angle = -atan2(H.y - R.y, H.x - R.x);            // 角度
            a.push_back(angle);

            if (times.size() % 100 == 0 && times.size() != 0)
            {
                ceres::Problem problem;
                for (size_t i = 0; i < times.size(); ++i)
                {
                    ceres::CostFunction *cost_function =
                        new ceres::AutoDiffCostFunction<TrigFunctionCost, 1, 1, 1, 1, 1, 1>(new TrigFunctionCost(times[i], a[i]));
                    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), &A_over_omega, &omega, &phi, &b, &c);
                }

                // 配置求解器选项
                ceres::Solver::Options options;
                options.logging_type = ceres::SILENT;
                options.linear_solver_type = ceres::DENSE_SCHUR; // 经测验最准确的方法
                options.minimizer_progress_to_stdout = false; // 不打印最小化过程到标准输出

                // 运行求解器
                ceres::Solver::Summary summary;
                Solve(options, &problem, &summary);

                // 可选: 输出拟合结果
                // cout << summary.BriefReport() << "\n";
                double A = A_over_omega * omega;
                // cout << "A: " << A << ", ω: " << omega
                //      << ", φ: " << phi << ", b: " << b << ", c: " << c << "\n";

                // 收敛与异常判断
                double real_A = 0.785, real_omega = 1.884, real_phi = 1.81, real_b = 1.305;
                if (abs(abs(A) - real_A) <= real_A * 0.05 && abs(abs(omega) - real_omega) <= real_omega * 0.05 && abs(abs(b) - real_b) <= real_b * 0.05)
                {
                    cout << "Run " << run + 1 << "收敛结果为：" << "A: " << A << ", ω: " << omega
                         << ", φ: " << phi << ", b: " << b << ", c: " << c << "\n";
                    break;
                }
                if (cnt >= 2000)
                {
                    cout << "Run " << run + 1 << "收敛错误！" << endl;
                    break;
                }
                auto end = std::chrono::high_resolution_clock::now(); // 结束计时
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                runTimes[run] = duration;
                // cout << "Run " << run + 1 << " took " << duration << " milliseconds." << endl;
            }

            imshow("windmill", src);

            //=======================================================//

            waitKey(1);
        }
    }
    // 计算平均时间
    double totalDuration = accumulate(runTimes.begin(), runTimes.end(), 0.0);
    double averageDuration = totalDuration / numRuns;
    cout << "Average time over " << numRuns << " runs: " << averageDuration << " milliseconds." << endl;
    return 0;
}