#include "AstarPlanner/AstarPlanner.hpp"

#include "math.h"

AstarPlanner::AstarPlanner(int W_, int H_, Cell center){
    W = W_;
    H = H_;
    center_cell_x = center.x;
    center_cell_y = center.y;
}

void AstarPlanner::set_cost_mode(CostMode m){
    cost_mode = m;
}

void AstarPlanner::set_goal_cell(Cell goal){
    goal_idx = (goal.x + center_cell_x) + (goal.y + center_cell_y) * W;
}


void AstarPlanner::set_dist_cost(){
    return;
}

void AstarPlanner::set_roughness_cost(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    roughness_cost.assign(static_cast<size_t>(W) * static_cast<size_t>(H), std::numeric_limits<float>::quiet_NaN());

    const auto& d = msg->data;
    const size_t n = d.size();

    for(int k = 0; k + 2 < int(n); k += 3){
        int i = static_cast<int>(std::lround(d[k])) + center_cell_x;
        int j = static_cast<int>(std::lround(d[k + 1])) + center_cell_y;
        float v = d[k + 2];

        if(i < 0 || i >= W || j < 0 || j >= H) continue;
        if(!std::isfinite(v)) continue;

        roughness_cost[static_cast<size_t>(j) * static_cast<size_t>(W) + static_cast<size_t>(i)] = v;
    }

}

void AstarPlanner::set_slope_cost(std_msgs::msg::Float32MultiArray::SharedPtr msg){
    slope_cost.assign(static_cast<size_t>(W) * static_cast<size_t>(H), std::vector<float>(8, std::numeric_limits<float>::quiet_NaN()));

    const auto& d = msg->data;
    const size_t n = d.size();

    for(int k = 0; k + 9 < int(n); k += 10){
        int i = static_cast<int>(std::lround(d[k])) + center_cell_x;
        int j = static_cast<int>(std::lround(d[k + 1])) + center_cell_y;

        if(i < 0 || i >= W || j < 0 || j >= H) continue;

        const size_t idx = static_cast<size_t>(j) * W + static_cast<size_t>(i);
        
        for(int dir = 0; dir < 8; dir++){
            const float v = d[k + 2 + dir];
            if(std::isfinite(v)){
                slope_cost[idx][dir] = v;
            }
        }
    }
}

void AstarPlanner::set_odom(Odometry msg){
    odom = msg;
}

void AstarPlanner::Astar_Plan(){
    //Astarを使ってpathを生成する
    set_now_cell();
    Astar();
}

std::vector<Cell> AstarPlanner::get_path(){
    return path;
}

bool AstarPlanner::get_no_path(){
    return no_path;
}

void AstarPlanner::set_now_cell(){
    int i = static_cast<int>(floor(odom.x)) + center_cell_x;
    int j = static_cast<int>(floor(odom.y)) + center_cell_y;

    now_idx = static_cast<size_t>(j) * static_cast<size_t>(W) + static_cast<size_t>(i);
}

CostFG AstarPlanner::culculate_cost(size_t now, size_t next, float now_cost){
    int now_x = now % W;
    int now_y = now / W;
    int next_x = next % W;
    int next_y = next / W;
    int goal_x = goal_idx % W;
    int goal_y = goal_idx / W;

    int dx = next_x - now_x;
    int dy = next_y - now_y;

    float dis = (dx == 0 || dy == 0) ? 1.0f : sqrt(2);

    float h_cost = heuristic(next_x, next_y, goal_x, goal_y);
    
    
    int now_dir  = dir_from_dxdy(next_x - now_x, next_y - now_y);
    int next_dir = dir_from_dxdy(now_x - next_x, now_y - next_y);

    float roughness_next_cost = std::isfinite(roughness_cost[next]) ? roughness_cost[next] : 0.0f;
    float slope_now_cost = std::isfinite(slope_cost[now][now_dir]) ? slope_cost[now][now_dir] : 0.0f;
    float slope_next_cost = std::isfinite(slope_cost[next][next_dir]) ? slope_cost[next][next_dir] : 0.0f;
    
    float roughness = roughness_k * (1.0f + roughness_next_cost);
    float slope = slope_k * (1.0f + 0.5f * (slope_now_cost + slope_next_cost));

    float g_cost;
    float f_cost;

    switch(cost_mode){
        case CostMode::DIST_ONLY:
            //一旦後で実装----------------------------------
            g_cost = 0;
            f_cost = 0;
            break;
        case CostMode::ROUGHNESS_ONLY:
            g_cost = now_cost + dis * roughness;
            f_cost = h_cost + g_cost;
            break;
        case CostMode::ROUGHNESS_SLOPE:
            g_cost = now_cost + dis * roughness * slope;
            f_cost = h_cost + g_cost;
            break;
    }

    CostFG cost = {f_cost, g_cost};

    return cost;
}

float AstarPlanner::heuristic(int x1, int y1, int x2, int y2){
    return std::max(abs(x2 - x1), abs(y2 - y1));
}


void AstarPlanner::Astar(){
    /* 
     * 推定の流れ
     * 
     * スタートセルをOPENとparentに追加する
     * 
     * 以下ループ
     * OPENのノードの中から最小のものを見つける
     * 最小のものの周囲8セルのそのノードからの向きを計算しOPENに追加する
     * ループしてゴールにたどり着くまで行う
     * 
     * parentをたどって経路を確立する
     * path_statesにgoalから逆にたどって配列に保存->startまで
     * 逆向きにすることでpathができる
     */

    //idxをx,yに変換
    int now_x = now_idx % W;
    int now_y = now_idx / W;
    int goal_x = goal_idx % W;
    int goal_y = goal_idx / W;

    //変数宣言
    Cell_id cell_state;
    OpenNode open_node, open_top;
    const Cell_id INVALID_STATE = {-1, -1};
    Cell_id goal_state = INVALID_STATE;

    std::priority_queue<OpenNode, std::vector<OpenNode>, CompareF> OPEN;
    std::vector<std::vector<bool>> CLOSED(W * H, std::vector<bool>(9, false));
    std::vector<std::vector<Cell_id>> parent(W * H, std::vector<Cell_id>(9, INVALID_STATE));
    std::vector<std::vector<float>> g_cost(W * H, std::vector<float>(9, INFINITY));//無限で宣言　後から少ないgに更新していく形
    std::vector<Cell_id> path_states;

    float culed_cost;//計算した後のコスト(f値)を一時保存する変数
    float g_now, g_candidate;//経路を探索中の探索している地点のg値と計算結果からでたg値
    int now_cell, next_cell;//経路探索中の探索セルとその次に探索するセル
    int now_dir, dir_in;//経路探索中の探索セルから次の探索への方向とその逆
    int nx, ny;//経路探索中の now_cell のx,yの値
    //この辺の変数名は割とごみではある

    path.clear();

    //経路生成の初期位置の処理
    open_node.state_id.cell = now_idx;
    open_node.state_id.dir_in = 8;
    open_node.f_cost = heuristic(now_x, now_y, goal_x, goal_y);
    OPEN.push(open_node);
    g_cost[now_idx][8] = 0.0f;
    cell_state.cell = now_idx;
    cell_state.dir_in = 8;
    parent[now_idx][8] = cell_state;
    //オープンから選んでゴールまでコスト計算をするwhile
    while(!OPEN.empty()){
        open_top = OPEN.top();
        OPEN.pop();
        now_cell = open_top.state_id.cell;
        now_dir = open_top.state_id.dir_in;
        g_now = g_cost[now_cell][now_dir];

        //close判定
        if(CLOSED[now_cell][now_dir]) continue;
        CLOSED[now_cell][now_dir] = true;
        //ゴールなら終了
        if(now_cell == goal_idx){
            goal_state = {now_cell, now_dir};
            break;
        }
        
        nx = now_cell % W;
        ny = now_cell / W;
        for(int dir_out = 0; dir_out < 8; dir_out++){
            if(nx + dx_from_dir(dir_out) < 0 || ny +  dy_from_dir(dir_out) < 0) continue;
            if (nx + dx_from_dir(dir_out) >= W || ny + dy_from_dir(dir_out) >= H) continue;

            next_cell = now_cell + dx_from_dir(dir_out) + W * dy_from_dir(dir_out);
            CostFG c = culculate_cost(size_t(now_cell), size_t(next_cell), g_now);
            culed_cost = c.f;
            g_candidate = c.g;
            dir_in = (dir_out + 4) % 8;

            if(g_candidate < g_cost[next_cell][dir_in]){
                g_cost[next_cell][dir_in] = g_candidate;
                cell_state.cell = now_cell;
                cell_state.dir_in = now_dir;
                parent[next_cell][dir_in] = cell_state;

                open_node.state_id.cell = next_cell;
                open_node.state_id.dir_in = dir_in;
                open_node.f_cost = culed_cost;
                OPEN.push(open_node);
            }
        }
    }

    //parentをたどって経路を確立する
    Cell_id cur;
    cur = goal_state;
    if(goal_state == INVALID_STATE){
        no_path = true;
        return;
    }

    while(!(cur == Cell_id{now_idx, 8})){
        path_states.push_back(cur);
        cur = parent[cur.cell][cur.dir_in];

        if(cur == INVALID_STATE){
            no_path = true;
            return;
        }
    }
    path_states.push_back(cur);

    //path_statesを逆向きにする
    std::reverse(path_states.begin(), path_states.end());

    //pathの生成
    int path_cell, path_x, path_y;
    Cell cell;
    for(size_t i = 0; i < path_states.size(); i++){
        path_cell = path_states[i].cell;
        path_x = path_cell % W - center_cell_x;
        path_y = path_cell / W - center_cell_y;
        cell = {path_x, path_y};
        path.push_back(cell);
    }

    no_path = false;
}
