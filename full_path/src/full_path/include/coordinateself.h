#ifndef COORDINATESELF_HPP
#define COORDINATESELF_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace std;

class Coordinateself {
public:
    using Point = std::pair<double, double>;
    using LineEquation = std::pair<double, double>;

    struct CircleData {
        Point center;
        double radius;
        Point refPoint;
    };

    struct MoveResult {
        std::vector<Point> x2_list;
        std::vector<Point> x3_list;
        std::vector<Point> x4_list;
    };

    /*
        将给定的4个坐标点进行坐标系转换
        参数:
        points (list): 包含4个坐标点的列表, 例如 [(3,6),(18,5),(19,28),(4,25)]
        返回:
        list: 转换后的4个坐标点
        新坐标系的前2个点
    */
    std::pair<std::vector<Point>, double> transform(const std::vector<Point>& points) const {
        double distance = std::hypot(points[1].first - points[0].first, points[1].second - points[0].second);//std::hypot()用于计算平方和开根号的函数

        std::vector<Point> translated;
        translated.reserve(points.size());//与地图角点的容器大小一致（保存容器容量与扩充）
        for (const auto& point : points) {
            translated.emplace_back(point.first - points[0].first, point.second - points[0].second);
        }

        double theta = -std::atan2(points[1].second - points[0].second, points[1].first - points[0].first);
        std::vector<Point> rotated;
        rotated.reserve(translated.size());
        for (const auto& point : translated) {
            double x = point.first * std::cos(theta) - point.second * std::sin(theta);
            double y = point.first * std::sin(theta) + point.second * std::cos(theta);
            rotated.emplace_back(x, y);
        }
        return {rotated, theta};
    }

    std::vector<Point> back_transform(const std::vector<Point>& points, double angle) const {
        std::vector<Point> rotated_points;
        rotated_points.reserve(points.size());
        for (const auto& point : points) {
            double x = point.first * std::cos(angle) - point.second * std::sin(angle);
            double y = point.first * std::sin(angle) + point.second * std::cos(angle);
            rotated_points.emplace_back(x, y);
        }
        return rotated_points;
    }

    static LineEquation calculate_slope_intercept(const Point& point1, const Point& point2) //计算斜率
    {
        double x1 = point1.first;
        double y1 = point1.second;
        double x2 = point2.first;
        double y2 = point2.second;
        if (x1 == x2) {
            double slope = std::numeric_limits<double>::infinity();
            double intercept = x1;
            return {slope, intercept};
        }
        double slope = (y2 - y1) / (x2 - x1);
        double intercept = y1 - slope * x1;
        return {slope, intercept};
    }

    std::vector<LineEquation> calculate_line_equations(const std::vector<Point>& points) const 
    {
        std::vector<LineEquation> line_equations;
        line_equations.reserve(points.size());
        for (std::size_t i = 0; i + 1 < points.size(); ++i) {
            line_equations.push_back(calculate_slope_intercept(points[i], points[i + 1]));
        }
        line_equations.push_back(calculate_slope_intercept(points.back(), points.front()));
        return line_equations;
    }

    double calculate_x_coordinate(double slope, double intercept, double known_y) const {
        return (known_y - intercept) / slope;
    }

    double calculate_y_coordinate(double slope, double intercept, double known_x) const {
        return slope * known_x + intercept;
    }

    MoveResult move_horizontally(const std::vector<LineEquation>& line_equations, double with_co,
                                 double y_plass, double limit3, double limit4) const {
        auto line_equation = small_quad_xiao(line_equations, with_co);
        const auto& l2 = line_equation[1];
        const auto& l3 = line_equation[2];
        const auto& l4 = line_equation[3];

        std::vector<Point> x2_list;
        std::vector<Point> x3_list;
        std::vector<Point> x4_list;

        if (limit3 < limit4) {
            while (y_plass < limit4) {
                double x4 = calculate_x_coordinate(l4.first, l4.second, y_plass);
                if (y_plass < limit3 && y_plass < limit4) {
                    double x2 = calculate_x_coordinate(l2.first, l2.second, y_plass);
                    x2_list.emplace_back(x2, y_plass);
                } else {
                    double x3 = calculate_x_coordinate(l3.first, l3.second, y_plass);
                    if (x3 >= x4) {
                        x3_list.emplace_back(x3, y_plass);
                    } else {
                        x3_list.emplace_back(x4, y_plass);
                        x4_list.emplace_back(x4, y_plass);
                        break;
                    }
                }
                x4_list.emplace_back(x4, y_plass);
                y_plass += with_co;
            }
        } else {
            while (y_plass < limit3) {
                double x2 = calculate_x_coordinate(l2.first, l2.second, y_plass);
                if (y_plass < limit4 && y_plass < limit3) {
                    double x4 = calculate_x_coordinate(l4.first, l4.second, y_plass);
                    x4_list.emplace_back(x4, y_plass);
                } else {
                    double x3 = calculate_x_coordinate(l3.first, l3.second, y_plass);
                    if (x3 <= x2) {
                        x3_list.emplace_back(x3, y_plass);
                    } else {
                        x3_list.emplace_back(x2, y_plass);
                        x2_list.emplace_back(x2, y_plass);
                        break;
                    }
                }
                x2_list.emplace_back(x2, y_plass);
                y_plass += with_co;
            }
        }
        return {x2_list, x3_list, x4_list};
    }

    std::vector<LineEquation> small_quad(const std::vector<LineEquation>& line_equations, double with_co) const {
        const auto& l2 = line_equations[1];
        const auto& l3 = line_equations[2];
        const auto& l4 = line_equations[3];

        double angle_2 = std::atan(l2.first);
        double angle_3 = std::atan(l3.first);
        double angle_4 = std::atan(l4.first);

        double y_1 = with_co;
        double y_2 = (l2.first < 0.0) ? -std::abs(with_co / std::cos(angle_2)) : std::abs(with_co / std::cos(angle_2));
        double y_3 = -std::abs(with_co / std::cos(angle_3));
        double y_4 = (l4.first < 0.0) ? std::abs(with_co / std::cos(angle_4)) : -std::abs(with_co / std::cos(angle_4));

        const double y_list[4] = {y_1, y_2, y_3, y_4};

        std::vector<LineEquation> line_equ_new;
        line_equ_new.reserve(line_equations.size());
        for (std::size_t i = 0; i < line_equations.size(); ++i) {
            line_equ_new.emplace_back(line_equations[i].first, line_equations[i].second + y_list[i]);
        }
        return line_equ_new;
    }

    std::vector<LineEquation> small_quad_xiao(const std::vector<LineEquation>& line_equations, double with_c) const {
        double with_co = with_c / 2.0;
        const auto& l2 = line_equations[1];
        const auto& l3 = line_equations[2];
        const auto& l4 = line_equations[3];

        double angle_2 = std::atan(l2.first);
        double angle_3 = std::atan(l3.first);
        double angle_4 = std::atan(l4.first);

        double y_1 = with_co;
        double y_2 = (l2.first < 0.0) ? -std::abs(with_co / std::cos(angle_2)) : std::abs(with_co / std::cos(angle_2));
        double y_3 = -std::abs(with_co / std::cos(angle_3));
        double y_4 = (l4.first < 0.0) ? std::abs(with_co / std::cos(angle_4)) : -std::abs(with_co / std::cos(angle_4));

        const double y_list[4] = {y_1, y_2, y_3, y_4};

        std::vector<LineEquation> line_equ_new;
        line_equ_new.reserve(line_equations.size());
        for (std::size_t i = 0; i < line_equations.size(); ++i) {
            line_equ_new.emplace_back(line_equations[i].first, line_equations[i].second + y_list[i]);
        }
        return line_equ_new;
    }

    Point find_Xpoint(const LineEquation& line1, const LineEquation& line2) const {
        double m1 = line1.first;
        double b1 = line1.second;
        double m2 = line2.first;
        double b2 = line2.second;
        double x = (b2 - b1) / (m1 - m2);
        double y = m1 * x + b1;
        return {x, y};
    }

    std::vector<Point> intersection_points(const std::vector<LineEquation>& line_equations, double with_co) const {
        auto intersection_point = [](double m1, double b1, double m2, double b2) -> Point {
            double x = (b2 - b1) / (m1 - m2);
            double y = m1 * x + b1;
            return {x, y};
        };

        std::vector<Point> intersections;
        intersections.reserve(line_equations.size() + 1);
        for (std::size_t i = 0; i + 1 < line_equations.size(); ++i) {
            intersections.push_back(intersection_point(line_equations[i].first, line_equations[i].second,
                                                       line_equations[i + 1].first, line_equations[i + 1].second));
        }

        const auto& last = line_equations.back();
        const auto& first = line_equations.front();
        intersections.push_back(intersection_point(last.first, last.second, first.first, first.second));

        double angle = std::atan(line_equations[3].first);
        double extra = std::abs(with_co / std::sin(angle));
        Point c_point = {intersections[3].first + extra, intersections[3].second};

        std::reverse(intersections.begin(), intersections.end());
        intersections.push_back(c_point);
        return intersections;
    }

    std::vector<Point> loop(std::vector<LineEquation> line_equations, double with_co) const {
        std::vector<Point> result;
        int a = 0;
        while (a < 20) {
            auto equ_i = small_quad(line_equations, with_co);
            auto b = intersection_points(equ_i, with_co);
            if ((b[3].first - b[0].first) <= with_co && (b[2].first - b[1].first) < with_co) {
                a = 100;
                result.push_back(b[0]);
                result.push_back(b[1]);
            } else {
                if ((b[1].second - b[0].second) < with_co && (b[2].second - b[3].second) < with_co) {
                    a = 100;
                    result.push_back(b[0]);
                    result.push_back(b[1]);
                } else {
                    a = 0;
                    line_equations = equ_i;
                }
                result.insert(result.end(), b.begin(), b.end());
            }
        }
        return result;
    }

    std::vector<Point> complete_path(const std::vector<Point>& list_a, const std::vector<Point>& list_b) const {
        std::vector<Point> new_list;
        if (list_a.empty()) {
            return new_list;
        }
        new_list.push_back(list_a.front());
        for (std::size_t i = 1; i < list_a.size(); i += 2) {
            if (i - 1 < list_b.size()) {
                new_list.push_back(list_b[i - 1]);
            }
            if (i < list_b.size()) {
                new_list.push_back(list_b[i]);
            }
            if (i < list_a.size()) {
                new_list.push_back(list_a[i]);
            }
            if (i + 1 < list_a.size()) {
                new_list.push_back(list_a[i + 1]);
            }
        }
        if (list_a.size() % 2 == 1 && !list_b.empty()) {
            new_list.push_back(list_b.back());
        }
        return new_list;
    }

    //弓字形路径
    std::pair<std::vector<Point>, double> s_rote(const std::vector<Point>& or_points, double working_wide) const {
        double with_co = working_wide;
        double y_plass = working_wide / 2.0;
        auto transformed = transform(or_points);
        auto points = transformed.first;
        //cout<<points[0].first<<points[0].second<<"   "<<points[1].first<<points[1].second<<"   "<<points[2].first<<points[2].second<<"   "<<points[3].first<<points[3].second<<endl;
        double angel_for_back = transformed.second;

        auto line_equations = calculate_line_equations(points);
        double limit3y = points[2].second;
        double limit4y = points[3].second;

        auto move_result = move_horizontally(line_equations, with_co, y_plass, limit3y, limit4y);
        int len_list2 = static_cast<int>(move_result.x2_list.size());
        int len_list4 = static_cast<int>(move_result.x4_list.size());

        std::vector<Point> list_he;
        std::vector<Point> list_max;
        if (len_list4 > len_list2) {
            list_he = move_result.x2_list;
            list_he.insert(list_he.end(), move_result.x3_list.begin(), move_result.x3_list.end());
            list_max = move_result.x4_list;
        } else {
            list_he = move_result.x4_list;
            list_he.insert(list_he.end(), move_result.x3_list.begin(), move_result.x3_list.end());
            list_max = move_result.x2_list;
        }

        auto list5 = complete_path(list_max, list_he);
        return {list5, angel_for_back};
    }

    std::vector<Point> point_extraction(const std::vector<Point>& security_route, double d) const {
        std::vector<Point> list_all;
        if (security_route.size() < 3) {
            return list_all;
        }

        for (std::size_t i = 0; i + 2 < security_route.size(); i += 2) {
            Point A = security_route[i + 1];
            Point B = security_route[i + 2];
            Point C = security_route[i];
            double angle = process_points(A, B, C);
            double dx = C.first - A.first;

            auto arc_data = interpolate_points_on_arc(A, B, angle, d);
            const auto& list_c = arc_data.first;
            const auto& midle = arc_data.second;

            int po_num = static_cast<int>(d / 2.0);
            int po_ns = (angle >= 1.57) ? 4 * po_num : po_num;
            int po_n = (angle >= 1.57) ? po_num : 4 * po_num;

            auto points1 = interpolate_points_on_circle(list_c[0].center, list_c[0].radius, list_c[0].refPoint, midle, po_ns, dx);
            auto points2 = interpolate_points_on_circle(list_c[1].center, list_c[1].radius, midle, list_c[1].refPoint, po_n, dx);

            std::vector<Point> points_all;
            points_all.reserve(points1.size() + points2.size() + 2);
            points_all.push_back(A);
            points_all.insert(points_all.end(), points1.begin(), points1.end());
            points_all.insert(points_all.end(), points2.begin(), points2.end());
            points_all.push_back(B);

            list_all.insert(list_all.end(), points_all.begin(), points_all.end());
        }
        return list_all;
    }

    std::vector<Point> interpolate_points_on_circle(const Point& center, double radius, const Point& point1,
                                                    const Point& point2, int num_points, double dx) const {
        if (num_points < 0) {
            num_points = 0;
        }
        double angle1 = std::atan2(point1.second - center.second, point1.first - center.first);
        double angle2 = std::atan2(point2.second - center.second, point2.first - center.first);

        if (dx > 0.0) {
            if (angle2 > angle1) {
                angle2 -= 2.0 * kPi;
            }
        } else {
            if (angle2 < angle1) {
                angle2 += 2.0 * kPi;
            }
        }

        int total = num_points + 1;
        if (total <= 1) {
            return {};
        }

        std::vector<Point> points;
        points.reserve(num_points);
        double step = (angle2 - angle1) / static_cast<double>(total - 1);
        for (int i = 1; i < total; ++i) {
            double angle = angle1 + step * static_cast<double>(i);
            double x = center.first + radius * std::cos(angle);
            double y = center.second + radius * std::sin(angle);
            points.emplace_back(x, y);
        }
        return points;
    }

    std::pair<std::vector<CircleData>, Point> interpolate_points_on_arc(const Point& A, const Point& B,
                                                                        double ang_le, double d) const {
        double x1 = A.first;
        double y1 = A.second;
        double x2 = B.first;
        double y2 = B.second;

        double l_distance = std::hypot(x2 - x1, y2 - y1);

        auto angle_with_y_axis = [this](const Point& po1, const Point& po2) -> double {
            double dx = po2.first - po1.first;
            double dy = po2.second - po1.second;
            if (dx == 0.0) {
                return (dy == 0.0) ? 0.0 : kPi / 2.0;
            }
            double slope = dy / dx;
            double angle_with_x_axis = std::atan(std::abs(slope));
            return kPi / 2.0 - angle_with_x_axis;
        };

        std::vector<CircleData> list2;
        Point midle{};

        if (ang_le >= 1.57) {
            double rb = 0.25 * d * std::sin(ang_le);
            double ra = 0.5 * std::abs(l_distance / std::sin(ang_le)) - rb;
            Point orb{x2, y2 - rb};
            Point ora{x1, y1 + ra};
            double l_c = std::hypot(x2 - x1, (y2 - y1) - rb);
            double an = angle_with_y_axis(orb, A);
            double numerator = (l_c * l_c) - (rb * rb);
            double denominator = 2.0 * (l_c * std::cos(an) - rb);
            double r3 = std::abs(numerator / denominator);
            Point or3{x1, y1 + r3};
            list2.push_back({or3, r3, A});
            list2.push_back({orb, rb, B});
            midle = tangent_points(list2[0].center, list2[0].radius, list2[1].center, list2[1].radius);
        } else {
            double ra = 0.25 * d * std::sin(ang_le);
            double rb = 0.5 * std::abs(l_distance / std::sin(ang_le)) - ra;
            Point ora{x1, y1 + ra};
            Point orb{x2, y2 - rb};
            double l_c = std::hypot(x2 - x1, (y2 - y1) - ra);
            double an = angle_with_y_axis(ora, B);
            double numerator = (l_c * l_c) - (ra * ra);
            double denominator = 2.0 * (l_c * std::cos(an) - ra);
            double r3 = std::abs(numerator / denominator);
            Point or3{x2, y2 - r3};
            list2.push_back({ora, ra, A});
            list2.push_back({or3, r3, B});
            midle = tangent_points(list2[1].center, list2[1].radius, list2[0].center, list2[0].radius);
        }
        return {list2, midle};
    }

    Point tangent_points(const Point& c1, double r1, const Point& c2, double r2) const {
        double dx = c2.first - c1.first;
        double dy = c2.second - c1.second;
        double distance = std::hypot(dx, dy);
        if (distance == 0.0) {
            return c1;
        }
        double unit_dx = dx / distance;
        double unit_dy = dy / distance;
        return {c1.first + r1 * unit_dx, c1.second + r1 * unit_dy};
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    double process_points(const Point& A, const Point& B, const Point& C) const {
        double angle_AC = std::atan2(C.second - A.second, C.first - A.first);
        double angle_AB = std::atan2(B.second - A.second, B.first - A.first);
        double angle = std::abs(angle_AB - angle_AC);
        if (angle > kPi) {
            angle = 2.0 * kPi - angle;
        }
        return angle;
    }
};

#endif
