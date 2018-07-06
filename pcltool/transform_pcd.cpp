#include <vector>
#include <string>
#include <dirent.h>
#include <algorithm>
#include <fstream>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(pcldir,"","pcd file directory");
DEFINE_string(pcldir_new,"","asc file name");
DEFINE_bool(isLoam,false,"because in loam algorithm's scanRegistration.cpp  lines 244 - 246");

DEFINE_double(roll,0,"");
DEFINE_double(pitch,0,"");

DEFINE_double(corrX,0,"");
DEFINE_double(corrY,0,"");
DEFINE_double(corrZ,0,"");

std::vector<std::string> load_dir_files(const char *dir)
{
    DIR *dp;
    struct dirent *ep;
    char filename[PATH_MAX];
    dp = opendir(dir);
    if(dp == NULL) LOG(ERROR)<< dir << " not exists";
    std::vector<std::string> files;
    while((ep = readdir(dp)))
    {
        if(ep->d_name[0] == '.')
            continue;
        sprintf(filename,"%s/%s",dir,ep->d_name);
        //LOG(INFO)<<std::string(filename);
        files.push_back(std::string(filename));
    }
    closedir(dp);
    sort(files.begin(), files.end());
    return files;
}
inline void transform(pcl::PointXYZI &p)
{

	float x;
	float y;
	float z;
	if(!FLAGS_isLoam) 
	{
		x=p.x;
		y=p.y;
		z=p.z;
	}
	else 
	{
		x=p.z;
		y=p.x;
		z=p.y;
	}

    float x1=x;
	float y1=y*std::cos(M_PI*FLAGS_roll/180)-1*z*std::sin(M_PI*FLAGS_roll/180);
	float z1=y*std::sin(M_PI*FLAGS_roll/180)+z*std::cos(M_PI*FLAGS_roll/180);
	
	float x2=x1*std::cos(M_PI*FLAGS_pitch/180)+z1*std::sin(M_PI*FLAGS_pitch/180);
	float y2=y1;
	float z2=-1*x1*std::sin(M_PI*FLAGS_pitch/180)+z1*std::cos(M_PI*FLAGS_pitch/180);

	p.x=y2 + FLAGS_corrX;
	p.y=z2 + FLAGS_corrY;
	p.z=x2 + FLAGS_corrZ;
}
void myLoadPCDFile(const std::string filenames,  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string name)
{

	if(pcl::io::loadPCDFile(filenames, *cloud) == -1)
	{
		LOG(ERROR)<<"ERROR: cannot load pcd file\n";
		exit(1);
	}
	//LOG(INFO)<<"cloud size: "<<cloud->size();
	for(size_t i = 0; i < cloud->size(); ++i)
	{
		transform(cloud->points[i]);
	}
	pcl::io::savePCDFileBinary(name,*cloud);
}

/* ---[ */

int main (int argc, char** argv)
{

	if(argc<5)
	{
		std::cerr<<"usage:\n1)./transform_pcd -pcldir pclFilePath -pcldir_new newPclFilePath\n2)./transform_pcd -pcldir pclFilePath -pcldir_new newPclFilePath -isLoam\n";
		exit(-1);
	}
	FLAGS_logtostderr=1;
	FLAGS_colorlogtostderr=1;
	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc,&argv,true);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	if(FLAGS_pcldir[0]!='\0')
    {
		std::vector<std::string> pcl_files;
		LOG(INFO)<<"pcl dir:"<<FLAGS_pcldir;
		LOG(INFO)<<"pcl_dir_new name:"<<FLAGS_pcldir_new;
		LOG(INFO)<<"is LOAM:"<<FLAGS_isLoam;
		LOG(INFO)<<"roll:"<<FLAGS_roll;
		LOG(INFO)<<"pitch:"<<FLAGS_pitch;
		LOG(INFO)<<"corr_x:"<<FLAGS_corrX;
		LOG(INFO)<<"corr_y:"<<FLAGS_corrY;
		LOG(INFO)<<"corr_z:"<<FLAGS_corrZ;
		pcl_files=load_dir_files(FLAGS_pcldir.c_str());
		size_t cnt=0;
		while(cnt<pcl_files.size())
		{
			std::string pclName;
			pclName=pcl_files[cnt].substr(pcl_files[cnt].rfind('/')+1);
			//LOG(INFO)<<pclName;
			cloud->clear();
			myLoadPCDFile(pcl_files[cnt],cloud, std::string(FLAGS_pcldir_new+pclName));
			cnt++;
		}
    }
	return (0);
}
