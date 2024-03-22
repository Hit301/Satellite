#include "Utils/utils.h"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <iomanip>

// writeCSV函数需要忽略警告
#pragma warning(disable:4996)
#define _CRT_SECURE_NO_WARNINGS

void writeCSV(char* fileName, double* data, int colNum) {
	/*
		数据流写入CSV文件，单次时间步
		fileName: 数据文件路径，不存在自动创建
		data: double类型指针（仅限一维数据）
		colNum: 单次时间步的数据维数
	*/
	

	FILE* fpt;
	const char writeMode[] = "a+";
	fpt = fopen(fileName, writeMode);

	int i;
	bool isFirst = true;
	for (i = 0; i < colNum; i++)
	{
		if (!isFirst) fprintf(fpt, ",");
		fprintf(fpt, "%f", data[i]);
		isFirst = false;
	}
	fprintf(fpt, "\n");
	fclose(fpt);
}


SaveDataClass::SaveDataClass(int precision)
{
	this->iter = 1;
	fout << std::setprecision(5);
	saveData.clear();
}


SaveDataClass::~SaveDataClass()
{
	this->fout.close();
}


void SaveDataClass::save(double *dataHead, int length)
{
	for (int i = 0; i < length; i++)
	{
		if (i != length - 1)
			this->fout << dataHead[i] <<',';
		else
			this->fout << dataHead[i] << std::endl;
	}
	
	for (auto const& x : this->saveData)
		this->fout << x[0] << ',' << x[1] << ',' << x[2] << '\n';

	fout.close(); // 保存完即关闭
}

void SaveDataClass::load(std::vector<double> data)
{
	this->saveData.push_back(data);
}


void SaveDataClass::updateSaveFileName(char path[NAMA_LEN_MAX])
{
	fout.open(path);
}

