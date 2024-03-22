#pragma once
#include <iostream>
#include <vector>
#include <fstream>

#define NAMA_LEN_MAX 100

class SaveDataClass
{
public:
	int iter;
	char path[NAMA_LEN_MAX];
	std::ofstream fout;
	std::vector<std::vector<double>> saveData;


	SaveDataClass(int precision=5);
	~SaveDataClass();

	void save(double* dataHead, int length);
	void load(std::vector<double> data);
	void updateSaveFileName(char path[NAMA_LEN_MAX]);

};


void writeCSV(char* fileName, double* data, int colNum);

