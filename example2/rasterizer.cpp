// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f ab = _v[1] - _v[0];
    Vector3f bc = _v[2] - _v[1];
    Vector3f ca = _v[0] - _v[2];

    Vector3f ap = Vector3f(x, y, _v[0].z()) - _v[0];
    Vector3f bp = Vector3f(x, y, _v[1].z()) - _v[1];
    Vector3f cp = Vector3f(x, y, _v[2].z()) - _v[2];

    Vector3f a = ab.cross(ap);
    Vector3f b = bc.cross(bp);
    Vector3f c = ca.cross(cp);

    if ((a.dot(b) > 0) && (a.dot(c) > 0)) {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    // Eigen::Matrix4f mv = view * model;
    // std::cout << "mvp: \n" << mvp << std::endl;
    int tri_cout = 0;
    for (auto& i : ind)
    {

        tri_cout++;
        std::cout << "tri_cout: " << tri_cout << std::endl;

        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        // std::cout << "0" << ": " << std::endl << v[0] << std::endl;
        // std::cout << "1" << ": " << std::endl << v[1] << std::endl;
        // std::cout << "2" << ": " << std::endl << v[2] << std::endl;

        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }

        std::cout << "0" << ": " << std::endl << v[0] << std::endl;
        std::cout << "1" << ": " << std::endl << v[1] << std::endl;
        std::cout << "2" << ": " << std::endl << v[2] << std::endl;

        // std::cout<< "after mvp\n" << v[0] << "\n" << v[1] << "\n" << v[2] << std::endl;
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        // std::cout << " `````````````` " << i << std::endl;
        rasterize_triangle(t);
    }

    // for(int i = 0; i < width; i++){
    //     for(int j = 0; j < height; j++){
    //         // int list[][] = {[0, 0], [1, 0], [0, 1], [1, 1]};

    //         // float zValue = 0;
    //         Eigen::Vector3f color = {0, 0, 0};

    //         for (int li = 0; li < 4; li++){
    //             // zValue += depth_buf[get_index(i, j)];
    //             color += frame_ssaa_buf[get_index_ssaa(i, j) + li];
    //         }

    //         frame_buf[get_index(i, j)] = color / 4;
    //     }
    // }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int x_min = std::min({v[0].x(), v[1].x(), v[2].x()});
    int x_max = std::max({v[0].x(), v[1].x(), v[2].x()});
    int y_min = std::min({v[0].y(), v[1].y(), v[2].y()});
    int y_max = std::max({v[0].y(), v[1].y(), v[2].y()});
    if (x_min < 0) x_min =0;
    if (x_max > width) x_max = width - 1;
    if (y_min < 0) y_min =0;
    if (y_max > height) y_max = height - 1;

    // std::cout << "x_min: " << x_min << ", x_max: " << x_max << ", y_min: " << y_min << ", y_max: " << y_max << std::endl;
    for (int i = x_min; i <= x_max; i++) {
        for (int j = y_min; j <= y_max; j++) {

            // std::cout << i << ", " <<j << std::endl;

            if (insideTriangle(i + 0.5, j + 0.5, t.v)) {
                // If so, use the following code to get the interpolated z value.
                // auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                // z_interpolated *= w_reciprocal;

                auto[alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
                // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() + beta * v[1].z() + gamma * v[2].z();
                // std::cout << "z_interpolated: " << z_interpolated << std::endl;
                // z_interpolated *= w_reciprocal;

                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

                if (z_interpolated < depth_buf[get_index(i, j)]) {
                    depth_buf[get_index(i, j)] = z_interpolated;
                    set_pixel(Eigen::Vector3f(i, j, 1), t.getColor());
                }
            }
        }
    }



    // for (int i = x_min; i <= x_max; i++) {
    //     for (int j = y_min; j <= y_max; j++) {
    //         float list[4][2] = {{0.25, 0.25}, {0.75, 0.25}, {0.25, 0.75},{ 0.75, 0.75}};

    //         for (int li = 0; li < 4; li++){
    //             if (insideTriangle(i + list[li][0], j + list[li][1], t.v)) {
    //                 // If so, use the following code to get the interpolated z value.
    //                 auto[alpha, beta, gamma] = computeBarycentric2D(i + list[li][0], j + list[li][1], t.v);
    //                 float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                 float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //                 z_interpolated *= w_reciprocal;

    //                 // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    //                 if (z_interpolated < depth_ssaa_buf[get_index_ssaa(i, j) + li]) {
    //                     depth_ssaa_buf[get_index_ssaa(i, j) + li] = z_interpolated;
    //                     frame_ssaa_buf[get_index_ssaa(i, j) + li] = t.getColor();
    //                 }
    //             }
    //         } 

    //     }
    // }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_ssaa_buf.begin(), frame_ssaa_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_ssaa_buf.begin(), depth_ssaa_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_ssaa_buf.resize(w * h * 4);
    depth_ssaa_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_index_ssaa(int x, int y)
{
    return (y * width + x) * 4;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width; //如果左下角是坐标原点，这种方式是从下往上，从右往左扫描。
    auto ind = (height-1-point.y())*width + point.x();  //这样方式是从上往下，从右往左扫描。符合视觉上的情况？如果把图片输入内存。
    frame_buf[ind] = color;
}

// clang-format on