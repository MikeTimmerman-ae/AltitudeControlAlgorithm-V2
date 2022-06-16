/**
 *	\file src/helpers.cpp
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#include "../header.h"    // #include header



MatrixXf loadFromFile(string FileName, int row, int col)
{
    MatrixXf Matrix(row, col);
	ifstream File(FileName);
	for (int k = 0; k < row; ++k) {
		string line;
		getline(File, line, '\n');
		for (int j = 0; j < col; ++j) {
			string entry = line.substr(0, line.find(','));
			line.erase(0, line.find(',') + 1);
			Matrix(k, j) = stod(entry);
		}
	}
    return Matrix;
}


void saveToFile(MatrixXf &data, int rows, int cols, string FileName)
{
    ofstream File; File.open(FileName);
    for (int k = 0; k < rows; ++k) {
        string line = "";
        for (int j = 0; j < cols; ++j) {
            line.append(to_string(data(k, j)));
            line.append(",");
        }
        line.pop_back();
        File << line << "\n";
    }
    File.close();
}