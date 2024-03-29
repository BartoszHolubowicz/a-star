#include <iostream>
#include <vector>
#include <algorithm>
#include <stack>
#include <fstream>
#include <string>
#include <cmath>
#include <cfloat>

using namespace std;

const int W = 20, H = 20, STEP = 1;

enum Dir {Up, Down, Left, Right, Undefined = -1};
enum NodeType {Empty = 0, Obstacle = 5, Checked = 1, Path = 3};

class Node {
  public:
    NodeType type;
    Node *parent;

    Node(int _x, int _y, NodeType _type) {
      x = _x;
      y = _y;
      type = _type;
      parent = NULL;
      gCost = STEP / 1.;
      hCost = FLT_MAX;
    }

    bool operator <(Node& node) {
      return getFCost() < node.getFCost();
    }

    bool operator !=(Node& node) {
      return getId() != node.getId();
    }

    int getId() {
      if (this == NULL)
        return -1;
      return x + y * W;
    }

    int getX() {
      return x;
    }

    int getY() {
      return y;
    }

    float getFCost() {
      return gCost + hCost;
    }

    float getGCost() {
      return gCost;
    }

    float getHCost() {
      return hCost;
    }

    Dir getDirToParent() {
      if (x - parent->x == 1)
        return Left;
      if (x - parent->x == -1)
        return Right;
      if (y - parent->y == 1)
        return Up;
      if (y - parent->y == -1)
        return Down;
      return Undefined;
    }

    float calculateFCost(int targetX, int targetY) {
      return gCost + calculateHCost(targetX, targetY);
    }

    float calculateFCost(Node *target) {
      return calculateFCost(target->getX(), target->getY());
    }

    float calculateHCost(int targetX, int targetY) {
      return sqrt(pow(x - targetX, 2) + pow(y - targetY, 2));
    }

    float calculateHCost(Node *target) {
      return calculateHCost(target->getX(), target->getY());
    }

    void setHCost(float newCost) {
      hCost = newCost;
    }

    string toString() {
      if (this == NULL)
        return "NULL Node";
      return "Node(" + to_string(x) + ", " + to_string(y) + ") of type " + to_string(type);
    }

  private:
    int x, y;
    float gCost, hCost;
};

class Grid {
  public:
    static void displayGrid(vector<vector<Node*> > grid) {
      for (auto row : grid) {
        for (Node* node : row) {
          cout << node->type << " ";
        }
        cout << endl;
      }
    }

    static vector<vector<Node*> > loadGrid(string fileName) {
      string line;
      vector<vector<Node*> > result;
      vector<Node*> row;
      ifstream file(fileName);

      if (file.is_open()) {
        int y = 0;
        while (getline(file, line)) {
          for (int i = 0, x = 0; line[i] != '\0'; i++) {
            if (line[i] != ' ') {
              row.push_back(new Node(x++, y, (NodeType)((int)line[i] - 48)));
            }
          }
          result.push_back(row);
          row.clear();
          y++;
        }
        file.close();
      }
      return result;
    }
};

class Field {
  public:
    vector<vector<Node*> > grid;
    Node *start, *target;

    Field(vector<vector<Node*> > _grid) {
      grid = _grid;
      start = (*this)(0, 0);
      target = (*this)(19, 19);
    }

    Node*& operator ()(int i, int j) {
      return grid.at(j).at(i);
    }

    Node* getUpNode(Node *node) {
      if (node->getY() - 1 < 0)
        return NULL;
      return (*this)(node->getX(), node->getY() - 1);
    }

    Node* getDownNode(Node *node) {
      if (node->getY() + 1 >= H)
        return NULL;
      return (*this)(node->getX(), node->getY() + 1);
    }

    Node* getLeftNode(Node *node) {
      if (node->getX() - 1 < 0) 
        return NULL;
      return (*this)(node->getX() - 1, node->getY());
    }

    Node* getRightNode(Node *node) {
      if (node->getX() + 1 >= W)
        return NULL;
      return (*this)(node->getX() + 1, node->getY());
    }

    vector<Node*> getNeighbours(Node *node) {
      vector<Node*> result;
      Node* up = getUpNode(node);
      Node* down = getDownNode(node);
      Node* left = getLeftNode(node);
      Node* right = getRightNode(node);
      if (up != NULL)
        result.push_back(up);
      if (down != NULL)
        result.push_back(down);
      if (left != NULL)
        result.push_back(left);
      if (right != NULL)
        result.push_back(right);

      return result;
    }

    vector<Node*> getEmptyNeighbours(Node *node) {
      vector<Node*> result;
      vector<Node*> neighbours = getNeighbours(node);

      for (vector<Node*>::iterator it = neighbours.begin(); it != neighbours.end(); ++it) {
        Node *itNode = *it;
        if (itNode->type == Empty)
          result.push_back(itNode);
      }

      return result;
    }

    vector<Node*> getPath(Node* begin, Node* end) {
      vector<Node*> result;
      Node* current = begin;

      while (current != end) {
        result.push_back(current);
        current = current->parent;
      }
      // TODO: Getting last node in the loop instead of adding it outside of the loop
      result.push_back(end);

      return result;
    }

    void tracePath(vector<Node*> path) {
      for (vector<Node*>::iterator it = path.begin(); it != path.end(); ++it) {
        Node *itNode = *it;
        itNode->type = Path;
      }
    }

    Node* findLowestFCostNode(vector<Node*> list) {
      vector<Node*>::iterator it = list.begin();
      Node *result = *it;
      ++it;

      for (; it != list.end(); ++it) {
        Node *itNode = *it;
        if (itNode->getFCost() <= result->getFCost())
          result = itNode;
      }

      return result;
    }

    void addToOpenList(Node *node) {
      // If this node doesn't even have a parent, set one
      if (node->parent == NULL)
        node->parent = closedList.top();
      // Set new f cost and parent if node's f cost is higher than previous one
      if (node->getFCost() > node->calculateFCost(target)) {
        node->parent = closedList.top();
        node->setHCost(node->calculateHCost(target));
      }
      openList.push_back(node);
    }

    void addToOpenList(vector<Node*> nodes) {
      for (vector<Node*>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
        Node* itNode = *it;
        addToOpenList(itNode);
      }
    }

    void addToClosedList(Node *node) {
      node->type = Checked;
      vector<Node*>::iterator it = find(openList.begin(), openList.end(), node);
      if (it != openList.end())
        openList.erase(it);
      closedList.push(node);
    }

    void addToClosedList(vector<Node*> nodes) {
      for (vector<Node*>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
        Node* itNode = *it;
        addToClosedList(itNode);
      }
    }

    bool isAtTarget(int x, int y) {
      return x == target->getX() && y == target->getY();
    }

    bool isAtTarget(Node* node) {
      return isAtTarget(node->getX(), node->getY());
    }

    void aStar() {
      // Closed list initialized with start node
      closedList.push(start);
      
      // do-while, since at the beginning openList.size() will be equal to 0
      do {
        // Adding empty nodes around last visited node
        addToOpenList(getEmptyNeighbours(closedList.top()));
        // Adding the lowest f cost node from the open list to the closed list
        // while also removing it from the open list
        addToClosedList(findLowestFCostNode(openList));
      } while (isAtTarget(closedList.top()) == false && openList.size() > 0);

      if (isAtTarget(closedList.top()) == false && openList.size() == 0) {
        cout << "There's no optimal way to the target :(\n";
      } else {
        vector<Node*> path = getPath(closedList.top(), start);
        tracePath(path);
        Grid::displayGrid(grid);
      }
    }

  private:
    vector<Node*> openList;
    stack<Node*> closedList;
};

int main() {
  Field F(Grid::loadGrid("grid.txt"));
  F.aStar();

  return 0;
}