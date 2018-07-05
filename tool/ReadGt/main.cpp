#include <math.h>
#include <dirent.h>
#include<string>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include<fstream>
#include<iostream>
#include<vector>
using namespace std;
const char  img_path[100] = "/home/wangchao/paper/data/2017-4-16-12-55/GT";
struct  OBJID
{
    int ImageID ;
    int ObjectTD;
};
int main()
{

    DIR *dir=NULL;
    struct dirent* pDir=NULL;
    dir=opendir(img_path);
    if(dir == NULL)
    {
        printf("Error! can't open this dir\n");

    }
    else{
            while(1)
             {
                pDir = readdir(dir);
                if (pDir == NULL) break;
                if (pDir->d_type == DT_REG)
                {
                    int ImageID = 0;
                    int ObjectTD = 0;
                    vector<char> imageS;
                    vector<char> objectS;
                    string d_name = pDir ->d_name;
                    bool findx = false;
                    bool findd = false;
                    for(int i=0;i<d_name.size();i++)
                    {
                        if(d_name[i]=='-')
                        {
                            findx=true;
                        }
                        if(d_name[i]=='.')
                        {
                            findd=true;
                        }
                        if(!findx&&!findd)
                        {
                            imageS.push_back(d_name[i]);
                        }
                        if(findx&&!findd)
                        {
                            if(d_name[i]!='-')
                            {
                            objectS.push_back(d_name[i]);
                            }
                        }
                    }
                for(int i=0;i<imageS.size();i++)
                {
                    ImageID =ImageID+ (imageS[i]-'0')*pow(10,imageS.size()-i-1);
                }
                for(int i=0;i<objectS.size();i++)
                {
                    ObjectTD = ObjectTD +(objectS[i] - '0')*pow(10,objectS.size()-i-1);
                }

                if(ImageID>=9)
                {
                    char oname[100];
                    char nname[100];
                    sprintf(oname,"%s%s%d%s%d%s",img_path,"/",ImageID,"-",ObjectTD,".png");
                    sprintf(nname,"%s%s%d%s%d%s",img_path,"/",ImageID-3,"-",ObjectTD,".png");
                    if( rename(oname,nname)==0)
                    {
                        cout<<"change success"<<endl;
                    }
                }
                }
            }
    }
    return 0;
}
