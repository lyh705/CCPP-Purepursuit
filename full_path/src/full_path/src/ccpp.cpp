#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "coordinateself.h"

int main() {
    double working_wide = 0.5;
    [[maybe_unused]] double interpolation_step = 0.01;
    std::string yaml_file = "./a.yaml";

    std::vector<Coordinateself::Point> or_points{
        {1.0, 0.5},
        {9.0, 5.0},
        {5.0, 9.0},
        {1.0, 9.0}
    };

    Coordinateself transformer;
    auto srote_result = transformer.s_rote(or_points, working_wide);
    auto ass = srote_result.first;
    double angel_for_back = srote_result.second;

    Coordinateself::Point last_p = ass.back();
    std::vector<Coordinateself::Point> ok_l = ass;
    if (ass.size() % 2 != 0) {
        ok_l.pop_back();
    }

    auto path_li = transformer.point_extraction(ok_l, working_wide);
    path_li.push_back(last_p);
    path_li.insert(path_li.begin(), ass.front());

    double dx = or_points[0].first;
    double dy = or_points[0].second;

    auto path_list = transformer.back_transform(path_li, -angel_for_back);
    for (auto& point : path_list) {
        point.first += dx;
        point.second += dy;
    }

    std::ofstream file(yaml_file);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << yaml_file << std::endl;
        return 1;
    }

    file << std::fixed << std::setprecision(10);
    file << "[\n";
    for (std::size_t i = 0; i < path_list.size(); ++i) {
        file << "  [" << path_list[i].first << ", " << path_list[i].second << "]";
        if (i + 1 != path_list.size()) {
            file << ",\n";
        } else {
            file << "\n";
        }
    }
    file << "]";
    return 0;
}