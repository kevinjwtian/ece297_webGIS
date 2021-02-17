#pragma once
#include <map>
#include <vector>
#include <string>
#include <cmath>
#include <queue>

using namespace std;

#define  min(a,b) ((a<b)?a:b)
int levenshtein_distance(const string source, const string target);
int levenshtein_distance(const string source, const string target)//calculate two strings distance
{
    //step 1

    int n = source.length();
    int m = target.length();
    if (m == 0) return n;
    if (n == 0) return m;
    //Construct a matrix
    typedef vector<vector<int> >  Tmatrix;
    Tmatrix matrix(n + 1);
    for (int i = 0; i <= n; i++)  matrix[i].resize(m + 1);

    //step 2 Initialize

    for (int i = 1; i <= n; i++) matrix[i][0] = i;
    for (int i = 1; i <= m; i++) matrix[0][i] = i;

    //step 3
    for (int i = 1; i <= n; i++)
    {
        const char si = source[i - 1];
        //step 4
        for (int j = 1; j <= m; j++)
        {

            const char dj = target[j - 1];
            //step 5
            int cost;
            if (si == dj) {
                cost = 0;
            }
            else {
                cost = 1;
            }
            //step 6
            const int above = matrix[i - 1][j] + 1;
            const int left = matrix[i][j - 1] + 1;
            const int diag = matrix[i - 1][j - 1] + cost;
            matrix[i][j] = min(above, min(left, diag));

        }
    }//step7
    return matrix[n][m];
}

class TreeNode {
private:
    string key_;
    
    map<int, TreeNode*>* children_;  // {distance: child}
public:
    // constructor
    explicit TreeNode(const string& key) : key_(key), children_(NULL) {}

    // destructor
    ~TreeNode() {
        if (children_) {
            for (auto it = children_->begin(); it != children_->end(); it++) {
                delete it->second;
            }
            delete children_;
        }
    }

    // add a new node into the tree
    bool Add(TreeNode* node) {
        if (!node) {
            return false;
        }

        int distance = levenshtein_distance(key_, node->key_);

        if (!children_) {
            children_ = new std::map<int, TreeNode*>();
        }

        // if there exists a node having the same distance
        // if theere does, add the current node as that node's children node
        // else create a new node linkling with the corresponding distance
        auto it = children_->find(distance);
        if (it == children_->end()) {
            children_->insert(make_pair(distance, node));
            return true;
        }
        else {
            return it->second->Add(node);
        }
    }

    void Find(vector<pair<int, string>>* found, const string& key, int max_distance) {
        std::cout<<"search at node : "<<key_<<std::endl;
        queue<TreeNode*> candidates;
        candidates.push(this);

        while (!candidates.empty()) {
            TreeNode* candidate = candidates.front();
            candidates.pop();
            int distance = levenshtein_distance(candidate->key_, key);

            if (distance <= max_distance) {
                found->push_back(make_pair(distance, candidate->key_));
            }

            if (candidate->HasChildren()) {
                for (auto it = candidate->children_->begin(); it != candidate->children_->end(); it++) {
                    if ((distance - max_distance) <= it->first && it->first <= (distance + max_distance)) {
                        candidates.push(it->second);
                    }
                }
            }
        }
    }
    bool HasChildren() {
        return children_ && children_->size();
    }
};


class BKTree {
private:
    TreeNode* root_;
public:
    BKTree() : root_(NULL) {}
    // add a node
    void Add(const string& key) {
        TreeNode* node = new TreeNode(key);
        if (!root_) {
            root_ = node;
        }
        else {
            root_->Add(node);
        }
    }

    // find given string
    vector<pair<int, string>> Find(const string& key, int max_distance) { //string:corrected string int: distance between corrected string and original string(key)
        vector<pair<int, string>> found;
        if (root_) {
            root_->Find(&found, key, max_distance);
        }
        return found;
    }   
    
    void Clear()
    {
        if(!root_)
            delete root_;
        root_ = NULL;
    }
};