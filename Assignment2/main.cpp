// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    float radians = rotation_angle * MY_PI / 180.0f;

    model(0, 0) = cos(radians), model(0, 1) = -sin(radians);
    model(1, 0) = sin(radians), model(1, 1) = cos(radians);

    // [cos, -sin, 0, 0]
    // [sin,  cos, 0, 0]
    // [  0,    0, 1, 0]
    // [  0,    0, 0, 1]

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    zNear = -zNear, zFar = -zFar;   // 改为右手坐标系
    // [l, r], [b, t], [n, f], 左手系 n 更大，f 更小
    float top = abs(zNear) * tan(eye_fov / 2.0f * MY_PI / 180.0f), bottom = -top;
    float right = top * aspect_ratio, left = -right;

    // 先平移再缩放, 正交投影 ortho = 伸缩变换 scale * 平移变换 transform
    // [2 / (r - l),           0,           0, 0]       [1, 0, 0, -(r + l) / 2]         [2 / (r - l),           0,           0,     -(r + l) / (r - l)]
    // [          0, 2 / (t - b),           0, 0]   *   [0, 1, 0, -(t + b) / 2]     =   [          0, 2 / (t - b),           0,     -(t + b) / (t - b)]
    // [          0,           0, 2 / (n - f), 0]       [0, 0, 1, -(n + f) / 2]         [          0,           0, 2 / (n - f),     -(n + f) / (n - f)]
    // [          0,           0,           0, 1]       [0, 0, 0,            1]         [          0,           0,           0,                      1]

    Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity(), persp_to_ortho = Eigen::Matrix4f::Zero();

    ortho(0, 0) = 2.0f / (right - left), ortho(0, 3) = (right + left) / (left - right);
    ortho(1, 1) = 2.0f / (top - bottom), ortho(1, 3) = (top + bottom) / (bottom - top);
    ortho(2, 2) = 2.0f / (zNear - zFar), ortho(2, 3) = (zNear + zFar) / (zFar - zNear);

    persp_to_ortho(0, 0) = zNear, persp_to_ortho(1, 1) = zNear, persp_to_ortho(2, 2) = zNear + zFar, persp_to_ortho(2, 3) = -zNear * zFar, persp_to_ortho(3, 2) = 1.0f;

    // 透视投影 persp_to_ortho , 将图形压缩为一个近平面的立方体
    // [n, 0,     0,      0]
    // [0, n,     0,      0]
    // [0, 0, n + f, -n * f]
    // [0, 0,     1,      0]

    return projection = ortho * persp_to_ortho;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    float radians = angle * MY_PI / 180.0f;
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();

    // 向量的叉乘矩阵
    // [0, -z, y]
    // [z, 0, -x]
    // [-y, x, 0]

    Eigen::Matrix3f N, R;
    N << 0, -axis.z(), axis.y(), axis.z(), 0, -axis.x(), -axis.y(), axis.x(), 0;
    R = cos(radians) * Eigen::Matrix3f::Identity() + (1 - cos(radians)) * axis * axis.transpose() + sin(radians) * N;

    rotation.block<3, 3>(0, 0) = R;

    return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on