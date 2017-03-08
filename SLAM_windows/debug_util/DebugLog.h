#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>


#define flushlog DebugLog::getLogFile()->flushLog();
#define debugLog(v) DebugLog::getLogFile()->wirteLog(v);flushlog;//DebugLog::getLogFile()->wirteLog(" ")
#define debugLogLine(v) debugLog(v);debugLog("\n");flushlog;
#define debugWriteLog(v) debugLog(#v);debugLog(":\n")debugLog(v);debugLog("\n");flushlog;
#define debugDisp(x) std::cout<<#x<<std::endl<<x<<std::endl;//log(#x);log("\n");log(x);log("\n");
#define debugLogMatrix(v,row,col) for(int i = 0;i<row;i++){ for(int j = 0;j<col-1;j++){debugLog(v[i*col+j]);debugLog("\t");debugLog("\t");}debugLog(v[i*col+col-1]);}debugLog("\n");flushlog;


	class DebugLog
	{
	private:
		DebugLog()//µ¥ÀýÄ£Ê½
		{
			File.open("log.txt");
		};


	public:
		std::ofstream File;
		template<class T>
		void wirteLog(std::vector<T> v)
		{


			for (int i = 0; i < v.size(); i++)
			{
				File << v[i] << std::endl;
			}
		}


		template<class T>
		void wirteLog(T v)
		{
			File << v;// << std::endl;
		}


		void flushLog()
		{
			File.flush();
		}


		static DebugLog* getLogFile()//getInstance()
		{
			static DebugLog* instance;
			if (instance == NULL)
			{
				instance = new DebugLog();
			}
			return instance;
		}




	};






std::string type2str(int type);