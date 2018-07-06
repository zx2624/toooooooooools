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
DEFINE_string(ascfilename,"","asc file name");
DEFINE_int32(factor,20,"factor");
DEFINE_bool(isLoam,false,"because in loam algorithm's scanRegistration.cpp  lines 244 - 246");

DEFINE_double(roll,0,"");
DEFINE_double(pitch,0,"");
DEFINE_double(yaw,0,"");

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
inline void transform(float  &x, float  &y, float &z)
{
    float x1=x;
	float y1=y*std::cos(M_PI*FLAGS_roll/180)-1*z*std::sin(M_PI*FLAGS_roll/180);
	float z1=y*std::sin(M_PI*FLAGS_roll/180)+z*std::cos(M_PI*FLAGS_roll/180);
	
	float x2=x1*std::cos(M_PI*FLAGS_pitch/180)+z1*std::sin(M_PI*FLAGS_pitch/180);
	float y2=y1;
	float z2=-1*x1*std::sin(M_PI*FLAGS_pitch/180)+z1*std::cos(M_PI*FLAGS_pitch/180);

	x=x2*std::cos(M_PI*FLAGS_yaw/180)-1*y2*std::sin(M_PI*FLAGS_yaw/180);
	y=x2*std::sin(M_PI*FLAGS_yaw/180)+y2*std::cos(M_PI*FLAGS_yaw/180);
	z=z2;
}
void myLoadPCDFile(const std::string filenames,  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,ofstream &of)
{

	if(pcl::io::loadPCDFile(filenames, *cloud) == -1)
	{
		LOG(ERROR)<<"ERROR: cannot load pcd file\n";
		exit(1);
	}

	//LOG(INFO)<<"cloud size: "<<cloud->size();
	if(FLAGS_isLoam) 
	{
		for(size_t i = 0; i < cloud->size(); ++i)
		{
			pcl::PointXYZI &p = cloud->points[i];
			size_t r;
			size_t g;
			size_t b;
			r = 0;
			g = p.intensity * FLAGS_factor <= 255 ? p.intensity * FLAGS_factor : 255;
			b = 0;
			transform(p.z, p.x, p.y);
			of<<p.z<<" "<<p.x<<" "<<p.y<<" "<<r<<" "<<g<<" "<<b<<endl;
		}
	}
	else
	{
		for(size_t i = 0; i < cloud->size(); ++i)
		{
			pcl::PointXYZI &p = cloud->points[i];
			size_t r;
			size_t g;
			size_t b;
			r = 255;
			g = 255;
			b = 255;
			of<<p.x<<" "<<p.y<<" "<<p.z<<" "<<r<<" "<<g<<" "<<b<<endl;
		}
	}

}

/* ---[ */

int main (int argc, char** argv)
{

	if(argc<5)
	{
		std::cerr<<"usage:\n1)./pcd2asc -pcldir pclFilePath -ascfillname ascfilename\n2)./pcd2asc -pcldir pclFilePath -ascfillname filename.asc -isLoam -roll rollvalue -pitch pitchvalue\n";
		exit(-1);
	}
	FLAGS_logtostderr=1;
	FLAGS_colorlogtostderr=1;
	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc,&argv,true);
	ofstream of;
	of.open(FLAGS_ascfilename);
	// Parse the command line arguments for .pcd and .ply files
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	if(FLAGS_pcldir[0]!='\0')
    {
		std::vector<std::string> pcl_files;
		LOG(INFO)<<"pcl dir:"<<FLAGS_pcldir;
		LOG(INFO)<<"asc file name:"<<FLAGS_ascfilename;
		LOG(INFO)<<"is LOAM:"<<FLAGS_isLoam;
		LOG(INFO)<<"roll:"<<FLAGS_roll;
		LOG(INFO)<<"pitch:"<<FLAGS_pitch;
		LOG(INFO)<<"yaw:"<<FLAGS_yaw;
		pcl_files=load_dir_files(FLAGS_pcldir.c_str());
		size_t cnt=0;
		while(cnt<pcl_files.size())
		{
			cloud->clear();
			myLoadPCDFile(pcl_files[cnt],cloud,of);
			cnt++;
		}
    }
    of.close();
	return (0);
}
