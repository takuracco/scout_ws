/*-------------------------------------------------------------------------------------------
Astarの経路計画を行うクラス
3種類のAstarの経路計画ができるように
cost_modeで切り替える
dist_costのみ，dist_costとroughness_costのみ，dist_costとslope_costとroughness_cost
入力
 roughness_cost
 slope_cost
 dist_cost
出力
 path
----------------------------------------------------------------------------*/
#include <vector>
#include <queue>
#include "Library.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

struct CostFG {
  float f;
  float g;
};

struct Cell_id{
    int cell;
    int dir_in;

    bool operator==(const Cell_id& other) const {
        return cell == other.cell && dir_in == other.dir_in;
    }
};

struct OpenNode {
    Cell_id state_id;
    float f_cost;
};

struct CompareF {
    bool operator()(const OpenNode& a, const OpenNode& b) const {
        return a.f_cost > b.f_cost;
    }
};

class AstarPlanner{
    private:
    //メンバー変数
        CostMode cost_mode;
        Odometry odom;
        
        //worldサイズ
        int W,H;
        int center_cell_x;
        int center_cell_y;

        //コストの保存
        std::vector<float> roughness_cost;
        std::vector<std::vector<float>> slope_cost;

        //コストの重み係数
        const float roughness_k = 1.0;
        const float slope_k = 1.0;


        //Astar用
        std::vector<Cell> path;
        int now_idx;
        int goal_idx;
        bool no_path = false;


    public:
        AstarPlanner(int W_, int H_, Cell center);
        
        //セッター
        void set_cost_mode(CostMode m);
        void set_goal_cell(Cell goal);
        //現在のセルを確認する関数
        void set_now_cell();
        //コスト
        void set_dist_cost();
        void set_roughness_cost(std_msgs::msg::Float32MultiArray msg);
        void set_slope_cost(std_msgs::msg::Float32MultiArray msg);
        //odom
        void set_odom(nav_msgs::msg::Odometry msg);
        
        //Astarを支持する
        void Astar_Plan();
        
    private:
        //Astar
        CostFG culculate_cost(size_t now, size_t next, float now_cost);
        float heuristic(int x1, int y1, int x2, int y2);
        void Astar();
    };