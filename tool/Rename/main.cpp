#include <dirent.h>
#include<string>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include<fstream>
#include<iostream>
using namespace std;
const char path[100] = "/home/wangchao/paper/data/";
int main()
{
    int num=3;
    char oname[100];
    char nname[100];
    while(num<26)
    {
//    if(num<7)
//    {
         sprintf(oname,"%s%s%d%s",path,"2017-6-4-6-34/pcd/",num,".pcd");
        sprintf(nname,"%s%s%d%s",path,"2017-6-4-6-34/pcd/",num-2,".pcd");
        if( rename(oname,nname)==0)
        {
            cout<<"change success"<<endl;
        }
//    }
//    else if(num>7)
//    {
//        sprintf(oname,"%s%s%d%s",path,"2017-4-16-12-55/pcd/",num,".pcd");
//        sprintf(nname,"%s%s%d%s",path,"2017-4-16-12-55/pcd/",num-3,".pcd");
//        if( rename(oname,nname)==0)
//        {
//            cout<<"change success"<<endl;
//        }
//    }
    num++;
    }
}
