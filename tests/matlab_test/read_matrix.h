#include "definitions.h"

#include <fstream>
#include <iostream>

inline cnearest::Matrix read_csv(std::string file, int rows, int cols) {
    std::ifstream in(file);

    std::string line;

    int row = 0;
    int col = 0;

    cnearest::Matrix res = cnearest::Matrix(rows, cols);
    res.setConstant(NAN);

    if (in.is_open()) {
        while (std::getline(in, line)) {
            char *ptr = (char *)line.c_str();
            size_t len = line.length();

            col = 0;

            char *start = ptr;
            for (size_t i = 0; i < len; i++) {
                if (ptr[i] == ',') {
                    res(row, col++) = atof(start);
                    start = ptr + i + 1;
                }
            }
            res(row, col) = atof(start);

            row++;
        }

        in.close();
    } else {
        throw std::runtime_error("File could not be opened");
    }
    return res;
}


inline cnearest::Vector read_csv(std::string file, int rows) {
    std::ifstream in(file);

    std::string line;

    int row = 0;

    cnearest::Vector res = cnearest::Vector(rows);
    res.setConstant(NAN);

    if (in.is_open()) {
        while (std::getline(in, line)) {
            res(row) = atof(line.c_str());
            row++;
        }

        in.close();
    }
    return res;
}