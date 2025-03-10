#include <iostream>
#include <vector>
#include <limits>
#include <string>
#include <cmath>
#include "common.h"
#include "solver.h"

using std::vector;
using std::pair;
using std::cout;
using std::endl;
using std::string;

int main(int argc, char* argv[])
{
    // 初始化随机数生成器
    srand(time(0));
    
    // 检查命令行参数
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " <input_file>" << endl;
        cout << "Example: " << argv[0] << " ../test/output_data_weighted.txt" << endl;
        return 1;
    }
    
    // 获取输入文件路径
    string filename = argv[1];
    
    // 初始化并加载配送问题数据
    DeliveryProblem problem;
    if (!loadProblemData(filename, problem)) {
        cout << "加载数据失败，程序退出。" << endl;
        return 1;
    }
    
    // 使用三阶段法求解静态配送问题
    auto allPaths = solveStaticProblem(problem);
    
    // 输出配送结果详情
    printDeliveryResults(problem, allPaths);

    // 输出每个配送中心到任务点的距离信息
    printCenterToTaskDistances(problem);

    return 0;
}
