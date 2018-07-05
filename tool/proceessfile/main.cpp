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
const char  img_path[100] = "/media/wangchao/84F804EB3A0D306C/rgbd_data/rutgers_apc_dataset/all_data/imges";
const char  depth_path[100] = "/media/wangchao/84F804EB3A0D306C/rgbd_data/rutgers_apc_dataset/all_data/depth_im";
const  char  mask_path[100] = "/media/wangchao/84F804EB3A0D306C/rgbd_data/rutgers_apc_dataset/all_data/mask";
const char  pose_path[100] = "/media/wangchao/84F804EB3A0D306C/rgbd_data/rutgers_apc_dataset/all_data/pose";
void ClassificationAcut(char *path)
{
    DIR *dir=NULL;
    struct dirent* pDir=NULL;
    dir=opendir(path);
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
                    char filepath[150];
                     char newfilepath[150];
                    string img_name = ".png";
                    string image = "image";
                    string depth = "depth";
                    string mask = "mask";
                    string pose = ".yml";
                    string d_name = pDir ->d_name;
                    sprintf(filepath,"%s%s%s",path,"/",pDir ->d_name);

                    if(d_name.find(img_name)<d_name.length())
                    {
                        if(d_name.find(image)<d_name.length())
                        {
                            sprintf(newfilepath,"%s%s%s",img_path,"/",pDir ->d_name);
                            if(rename(filepath,newfilepath)==0)
                            {
                                cout<<"move image success"<<endl;
                            }
                        }
                        if(d_name.find(depth)<d_name.length())
                        {
                            sprintf(newfilepath,"%s%s%s",depth_path,"/",pDir ->d_name);
                            if(rename(filepath,newfilepath)==0)
                            {
                                cout<<"move depth image success"<<endl;
                            }
                        }
                        if(d_name.find(mask)<d_name.length())
                        {
                            sprintf(newfilepath,"%s%s%s",mask_path,"/",pDir ->d_name);
                            if(rename(filepath,newfilepath)==0)
                            {
                                cout<<"move mask image success"<<endl;
                            }
                        }
                    }
                    if(d_name.find(pose)<d_name.length())
                    {
                        sprintf(newfilepath,"%s%s%s",pose_path,"/",pDir ->d_name);
                        if(rename(filepath,newfilepath)==0)
                        {
                            cout<<"move pose success"<<endl;
                        }
                    }
//                     printf("%s\n",pDir ->d_name);

                }
                if (pDir->d_type == DT_DIR)
                {
                    char newpath[200];
                    sprintf(newpath,"%s%s%s",path,"/",pDir ->d_name);
//                    cout<<pDir ->d_name<<endl;
                    char *a = ".";
                    char *b = "..";
//                     string c = pDir ->d_name;
                    if(strcmp(a,pDir ->d_name) != 0&& strcmp(b,pDir ->d_name) != 0)
                    {
                    ClassificationAcut(newpath);
                    }
                }
            }
        }
    closedir(dir);

}
int main(){
   char  path[100] = "/media/wangchao/84F804EB3A0D306C/rgbd_data/rutgers_apc_dataset/pose";

   if(access(img_path,F_OK)==-1)
   {
       mkdir(img_path,0777);
   }
   if(access(depth_path,F_OK)==-1)
   {
       mkdir(depth_path,0777);
   }
   if(access(mask_path,F_OK)==-1)
   {
       mkdir(mask_path,0777);
   }
   if(access(pose_path,F_OK)==-1)
   {
       mkdir(pose_path,0777);
   }
   ClassificationAcut(path);
    return 0;
}
