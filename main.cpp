#define CPPHTTPLIB_ZLIB_SUPPORT 0
#include "httplib.h"
#include "json.hpp"
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <string>
#include <algorithm>

using json = nlohmann::json;
using namespace std;

// Helper struct for pathfinding nodes
struct Node {
    int row, col;
    int cost;
    int parent_row, parent_col;
    
    Node(int r, int c, int cst = 0, int pr = -1, int pc = -1) 
        : row(r), col(c), cost(cst), parent_row(pr), parent_col(pc) {}
};

// Comparator for priority queue (min-heap)
struct CompareNode {
    bool operator()(const Node& a, const Node& b) const {
        return a.cost > b.cost;
    }
};

// Hash function for pair<int, int> to use in unordered_set/map
struct PairHash {
    size_t operator()(const pair<int, int>& p) const {
        return hash<int>()(p.first) ^ (hash<int>()(p.second) << 1);
    }
};

class MazeSolver {
private:
    vector<vector<int>> grid;
    int rows, cols;
    pair<int, int> start, end;
    vector<pair<int, int>> visited_order;
    
    // Direction vectors: up, down, left, right
    const vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    bool isValid(int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols && grid[r][c] != 1;
    }
    
    int getCellCost(int r, int c) {
        if (grid[r][c] == 5) return 5; // Mud
        if (grid[r][c] == 2) return 0; // Start
        return 1; // Empty or End
    }
    
    vector<pair<int, int>> reconstructPath(
        const unordered_map<pair<int, int>, pair<int, int>, PairHash>& parent_map) {
        vector<pair<int, int>> path;
        pair<int, int> current = end;
        
        while (current != start) {
            path.push_back(current);
            if (parent_map.find(current) == parent_map.end()) {
                return {}; // Path broken
            }
            current = parent_map.at(current);
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }
    
public:
    MazeSolver(const vector<vector<int>>& g) : grid(g) {
        rows = grid.size();
        cols = rows > 0 ? grid[0].size() : 0;
        
        // Find start and end positions
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (grid[i][j] == 2) start = {i, j};
                if (grid[i][j] == 3) end = {i, j};
            }
        }
    }
    
    pair<bool, json> solveBFS() {
        visited_order.clear();
        queue<pair<int, int>> q;
        unordered_set<pair<int, int>, PairHash> visited;
        unordered_map<pair<int, int>, pair<int, int>, PairHash> parent;
        
        q.push(start);
        visited.insert(start);
        visited_order.push_back(start);
        
        while (!q.empty()) {
            auto [r, c] = q.front();
            q.pop();
            
            if (r == end.first && c == end.second) {
                auto path = reconstructPath(parent);
                return {true, {
                    {"solved", true},
                    {"path", path},
                    {"visited_order", visited_order}
                }};
            }
            
            for (auto [dr, dc] : directions) {
                int nr = r + dr, nc = c + dc;
                if (isValid(nr, nc) && visited.find({nr, nc}) == visited.end()) {
                    q.push({nr, nc});
                    visited.insert({nr, nc});
                    parent[{nr, nc}] = {r, c};
                    visited_order.push_back({nr, nc});
                }
            }
        }
        
        return {false, {
            {"solved", false},
            {"path", json::array()},
            {"visited_order", visited_order}
        }};
    }
    
    pair<bool, json> solveDFS() {
        visited_order.clear();
        stack<pair<int, int>> st;
        unordered_set<pair<int, int>, PairHash> visited;
        unordered_map<pair<int, int>, pair<int, int>, PairHash> parent;
        
        st.push(start);
        visited.insert(start);
        visited_order.push_back(start);
        
        while (!st.empty()) {
            auto [r, c] = st.top();
            st.pop();
            
            if (r == end.first && c == end.second) {
                auto path = reconstructPath(parent);
                return {true, {
                    {"solved", true},
                    {"path", path},
                    {"visited_order", visited_order}
                }};
            }
            
            for (auto [dr, dc] : directions) {
                int nr = r + dr, nc = c + dc;
                if (isValid(nr, nc) && visited.find({nr, nc}) == visited.end()) {
                    st.push({nr, nc});
                    visited.insert({nr, nc});
                    parent[{nr, nc}] = {r, c};
                    visited_order.push_back({nr, nc});
                }
            }
        }
        
        return {false, {
            {"solved", false},
            {"path", json::array()},
            {"visited_order", visited_order}
        }};
    }
    
    pair<bool, json> solveDijkstra() {
        visited_order.clear();
        priority_queue<Node, vector<Node>, CompareNode> pq;
        unordered_map<pair<int, int>, int, PairHash> dist;
        unordered_map<pair<int, int>, pair<int, int>, PairHash> parent;
        
        pq.push(Node(start.first, start.second, 0));
        dist[start] = 0;
        visited_order.push_back(start);
        
        while (!pq.empty()) {
            Node curr = pq.top();
            pq.pop();
            
            pair<int, int> curr_pos = {curr.row, curr.col};
            
            // Skip if we've found a better path already
            if (dist.count(curr_pos) && curr.cost > dist[curr_pos]) continue;
            
            if (curr.row == end.first && curr.col == end.second) {
                auto path = reconstructPath(parent);
                return {true, {
                    {"solved", true},
                    {"path", path},
                    {"visited_order", visited_order}
                }};
            }
            
            for (auto [dr, dc] : directions) {
                int nr = curr.row + dr, nc = curr.col + dc;
                if (isValid(nr, nc)) {
                    int new_cost = curr.cost + getCellCost(nr, nc);
                    pair<int, int> next_pos = {nr, nc};
                    
                    if (!dist.count(next_pos) || new_cost < dist[next_pos]) {
                        dist[next_pos] = new_cost;
                        parent[next_pos] = curr_pos;
                        pq.push(Node(nr, nc, new_cost));
                        
                        // Only add to visited order if first time visiting
                        if (!dist.count(next_pos) || dist[next_pos] == new_cost) {
                            visited_order.push_back(next_pos);
                        }
                    }
                }
            }
        }
        
        return {false, {
            {"solved", false},
            {"path", json::array()},
            {"visited_order", visited_order}
        }};
    }
    
    pair<bool, json> solveAStar() {
        visited_order.clear();
        
        auto heuristic = [this](int r, int c) {
            return abs(r - end.first) + abs(c - end.second);
        };
        
        priority_queue<Node, vector<Node>, CompareNode> pq;
        unordered_map<pair<int, int>, int, PairHash> g_score;
        unordered_map<pair<int, int>, pair<int, int>, PairHash> parent;
        
        int h_start = heuristic(start.first, start.second);
        pq.push(Node(start.first, start.second, h_start));
        g_score[start] = 0;
        visited_order.push_back(start);
        
        while (!pq.empty()) {
            Node curr = pq.top();
            pq.pop();
            
            pair<int, int> curr_pos = {curr.row, curr.col};
            
            if (curr.row == end.first && curr.col == end.second) {
                auto path = reconstructPath(parent);
                return {true, {
                    {"solved", true},
                    {"path", path},
                    {"visited_order", visited_order}
                }};
            }
            
            for (auto [dr, dc] : directions) {
                int nr = curr.row + dr, nc = curr.col + dc;
                if (isValid(nr, nc)) {
                    int tentative_g = g_score[curr_pos] + getCellCost(nr, nc);
                    pair<int, int> next_pos = {nr, nc};
                    
                    if (!g_score.count(next_pos) || tentative_g < g_score[next_pos]) {
                        g_score[next_pos] = tentative_g;
                        parent[next_pos] = curr_pos;
                        int f_score = tentative_g + heuristic(nr, nc);
                        pq.push(Node(nr, nc, f_score));
                        
                        visited_order.push_back(next_pos);
                    }
                }
            }
        }
        
        return {false, {
            {"solved", false},
            {"path", json::array()},
            {"visited_order", visited_order}
        }};
    }
};

int main() {
    httplib::Server svr;
    
    // CORS Headers Handler
    svr.set_default_headers({
        {"Access-Control-Allow-Origin", "*"},
        {"Access-Control-Allow-Methods", "GET, POST, OPTIONS"},
        {"Access-Control-Allow-Headers", "Content-Type"}
    });
    
    // GET / endpoint - Server status
    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        json response = {
            {"status", "Server is running"},
            {"message", "Maze Solver API"},
            {"endpoints", {
                {"/", "GET - Server status"},
                {"/solve", "POST - Solve maze with algorithm"}
            }},
            {"usage", {
                {"algorithm", "BFS, DFS, Dijkstra, or AStar"},
                {"grid", "2D array where 0=empty, 1=wall, 2=start, 3=end, 5=mud"}
            }}
        };
        res.set_content(response.dump(2), "application/json");
    });
    
    // Handle OPTIONS preflight requests
    svr.Options("/solve", [](const httplib::Request&, httplib::Response& res) {
        res.status = 200;
    });
    
    // POST /solve endpoint
    svr.Post("/solve", [](const httplib::Request& req, httplib::Response& res) {
        try {
            json input = json::parse(req.body);
            
            string algorithm = input["algorithm"];
            vector<vector<int>> grid = input["grid"];
            
            MazeSolver solver(grid);
            pair<bool, json> result;
            
            if (algorithm == "BFS") {
                result = solver.solveBFS();
            } else if (algorithm == "DFS") {
                result = solver.solveDFS();
            } else if (algorithm == "Dijkstra") {
                result = solver.solveDijkstra();
            } else if (algorithm == "AStar") {
                result = solver.solveAStar();
            } else {
                res.status = 400;
                res.set_content(json({{"error", "Invalid algorithm"}}).dump(), "application/json");
                return;
            }
            
            res.set_content(result.second.dump(), "application/json");
            
        } catch (const exception& e) {
            res.status = 500;
            res.set_content(json({{"error", e.what()}}).dump(), "application/json");
        }
    });
    
    cout << "Server starting on http://localhost:8080" << endl;
    svr.listen("0.0.0.0", 8080);
    
    return 0;
}