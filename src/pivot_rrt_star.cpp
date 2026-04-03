#include "pivot_planner/pivot_rrt_star.hpp"
#include "pivot_planner/speculation.hpp"
#include <algorithm>
#include <limits>
#include <cmath>
#include <unordered_map>
#include <list>      
#include <stack>    

namespace pivot_planner {

PivotRRTStar::PivotRRTStar(PivotSampler& sampler,
                           FeatureExtractor& feat_extractor,
                           const Theta& theta,
                           const State& start,
                           const State& goal,
                           double xmin, double xmax, double ymin, double ymax,
                           double step_size,
                           double neighbor_radius,
                           double goal_tolerance,
                 std::shared_ptr<MapCollisionChecker> collision_checker)
    : sampler_(sampler),
      feat_extractor_(feat_extractor),
      theta_(theta),
      start_(start),
      goal_(goal),
      xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax),
      step_size_(step_size),
      neighbor_radius_(neighbor_radius),
      goal_tolerance_(goal_tolerance),
      collision_checker_(collision_checker),
      best_goal_node_(nullptr),
      iteration_count_(0),
      grid_cell_size_(neighbor_radius_)
{
    // Initialize tree with start node
    Node* root = new Node(start_, nullptr, 0.0);
    tree_.push_back(root);
    insertIntoHash(root);
    addToQueue(root);
    pushToStack(root);
    
    // Mark start as visited
    feat_extractor_.markVisited(start_);
    
    // Setup speculation context
    spec_ctx_.sampler = &sampler_;
    spec_ctx_.feat = &feat_extractor_;
    spec_ctx_.theta = &theta_;
    spec_ctx_.K = 0;  // Disabled by default
    spec_ctx_.min_future_utility = 0.3;
    spec_ctx_.step_size = step_size_ * 0.5;
}

PivotRRTStar::~PivotRRTStar() {
    // Clean up tree nodes
    for (Node* node : tree_) {
        delete node;
    }
}

void PivotRRTStar::run(int max_iterations) {
    for (int i = 0; i < max_iterations; ++i) {
        step();
    }
}

void PivotRRTStar::step() {
    iteration_count_++;
    
    //  Sample
    State x_rand = sampler_.sample();
    
    // Find nearest node
    Node* x_near = nearestNode(x_rand);
    
    //  Steer
    State x_new_state = steerTowards(x_near->state, x_rand, step_size_);
    
    //  Collision check using map
    if (collision_checker_ && !collision_checker_->isCollisionFree(x_near->state, x_new_state)) {
        return;
    }
    
    //  Speculative test
    if (!speculativeAccept(x_new_state, spec_ctx_)) {
        return;  // Reject this branch
    }
    
    // Find neighbors for rewiring
    std::vector<Node*> neighbors = findNeighbors(x_new_state, neighbor_radius_);
    
    //  Choose best parent (RRT* logic)
    Node* best_parent = x_near;
    double best_cost = x_near->cost_from_start + x_near->state.distance(x_new_state);
    
    for (Node* n : neighbors) {
        double new_cost = n->cost_from_start + n->state.distance(x_new_state);
        if (new_cost < best_cost) {
            // Check collision with map
            if (collision_checker_ && !collision_checker_->isCollisionFree(n->state, x_new_state)) {
                continue;
            }
            best_parent = n;
            best_cost = new_cost;
        }
    }
    
    // Create new node
    Node* x_new = new Node(x_new_state, best_parent, best_cost);
    tree_.push_back(x_new);
    insertIntoHash(x_new);
    addToQueue(x_new);
    pushToStack(x_new);
    
    // Mark as visited for exploration feature
    feat_extractor_.markVisited(x_new_state);
    
    //  Rewire neighbors
    for (Node* n : neighbors) {
        double through_new = x_new->cost_from_start + x_new->state.distance(n->state);
        if (through_new < n->cost_from_start) {
            // Check collision with map
            if (collision_checker_ && !collision_checker_->isCollisionFree(x_new->state, n->state)) {
                continue;
            }
            n->parent = x_new;
            n->cost_from_start = through_new;
        }
    }
    
    //  Check if we reached goal
    if (x_new_state.distance(goal_) <= goal_tolerance_) {
        if (!best_goal_node_ || x_new->cost_from_start < best_goal_node_->cost_from_start) {
            best_goal_node_ = x_new;
        }
    }
}


std::vector<State> PivotRRTStar::getBestPath() const {
    std::vector<State> path;

    if (!best_goal_node_) {
        return path;  // No path found yet
    }

    // Use a stack to backtrack from goal 
    std::stack<Node*> backtrack_stack;
    Node* current = best_goal_node_;
    while (current != nullptr) {
        backtrack_stack.push(current);
        current = current->parent;
    }

    // Use std::list 
    std::list<State> path_list;
    while (!backtrack_stack.empty()) {
        path_list.push_back(backtrack_stack.top()->state);
        backtrack_stack.pop();
    }

    path.assign(path_list.begin(), path_list.end());
    return path;
}


std::vector<Node*> PivotRRTStar::getSortedNodesByCost() const {
    // Create a copy of tree for sorting
    std::vector<Node*> sorted_nodes = tree_;
    
    // Sort nodes by cost using quicksort (via std::sort)
    std::sort(sorted_nodes.begin(), sorted_nodes.end(), 
              [](const Node* a, const Node* b) {
                  return a->cost_from_start < b->cost_from_start;
              });
    
    return sorted_nodes;
}

bool PivotRRTStar::hasReachedGoal() const {
    return best_goal_node_ != nullptr;
}

Node* PivotRRTStar::nearestNode(const State& s) const {
    Node* nearest = nullptr;
    double min_dist = std::numeric_limits<double>::infinity();
    
    for (Node* node : tree_) {
        double dist = node->state.distance(s);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    
    return nearest;
}

std::vector<Node*> PivotRRTStar::findNeighbors(const State& s, double radius) const {
    std::vector<Node*> neighbors;
    
    for (Node* node : tree_) {
        if (node->state.distance(s) <= radius) {
            neighbors.push_back(node);
        }
    }
    
    return neighbors;
}

bool PivotRRTStar::isValid(const State& s) const {
    return s.x >= xmin_ && s.x <= xmax_ && s.y >= ymin_ && s.y <= ymax_;
}

GridCell PivotRRTStar::getGridCell(const State& s) const {
    int cell_x = static_cast<int>(std::floor(s.x / grid_cell_size_));
    int cell_y = static_cast<int>(std::floor(s.y / grid_cell_size_));
    return GridCell(cell_x, cell_y);
}

void PivotRRTStar::insertIntoHash(Node* node) {
    GridCell cell = getGridCell(node->state);
    spatial_hash_[cell].push_back(node);
}

std::vector<Node*> PivotRRTStar::getNodesInCell(const GridCell& cell) const {
    auto it = spatial_hash_.find(cell);
    if (it != spatial_hash_.end()) {
        return it->second;
    }
    return std::vector<Node*>();
}

void PivotRRTStar::addToQueue(Node* node) {
    expansion_queue_.push(node);
}

Node* PivotRRTStar::getNextBestNode() {
    if (expansion_queue_.empty()) {
        return nullptr;
    }
    Node* best = expansion_queue_.top();
    expansion_queue_.pop();
    return best;
}

bool PivotRRTStar::queueEmpty() const {
    return expansion_queue_.empty();
}

void PivotRRTStar::pushToStack(Node* node) {
    exploration_stack_.push(node);
}

Node* PivotRRTStar::popFromStack() {
    if (exploration_stack_.empty()) {
        return nullptr;
    }
    Node* top = exploration_stack_.top();
    exploration_stack_.pop();
    return top;
}

bool PivotRRTStar::stackEmpty() const {
    return exploration_stack_.empty();
}

} // namespace pivot_planner