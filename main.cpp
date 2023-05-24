//
//  main.cpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "DStar.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, const char * argv[]) {
    Vector2i start = Vector2i(3, 2);
    Vector2i destination = Vector2i(23, 16);

    Dstar Algorithm(25, 25);
    Algorithm.run(start, destination, 5);

    return 0;
}
