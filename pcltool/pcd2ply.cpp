#include <dirent.h>

#include <vector>
#include <string>
#include <algorithm>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

DEFINE_string(pcldir,"","pcd file directory");
DEFINE_string(plyfilename,"","ply file name");
DEFINE_bool(format,false,"binary or ascii,true is binary");
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>());

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
        LOG(INFO)<<std::string(filename);
        files.push_back(std::string(filename));
    }
    closedir(dp);
    sort(files.begin(), files.end());
    return files;
}

void myLoadPCDFile(const std::string filenames,  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

	if(pcl::io::loadPCDFile(filenames, *cloud) == -1)
	{
		LOG(ERROR)<<"ERROR: cannot load pcd file\n";
		exit(1);
	}

	LOG(INFO)<<"cloud size: "<<cloud->size();
	*cloudPtr+=*cloud;
}


void toPly ()
{
	pcl::PLYWriter writer;
	writer.write(FLAGS_plyfilename, *cloudPtr, FLAGS_format);
}
/* ---[ */

int main (int argc, char** argv)
{
	FLAGS_logtostderr=1;
	FLAGS_colorlogtostderr=1;
	google::InitGoogleLogging(argv[0]);
	google::ParseCommandLineFlags(&argc,&argv,true);

	// Parse the command line arguments for .pcd and .ply files
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	if(FLAGS_pcldir[0]!='\0')
    {
			std::vector<std::string> pcl_files;
			LOG(INFO)<<"pcl dir:"<<FLAGS_pcldir;
			LOG(INFO)<<"ply file name:"<<FLAGS_plyfilename;
			pcl_files=load_dir_files(FLAGS_pcldir.c_str());
			cloudPtr->clear();
			size_t cnt=0;
			while(cnt<pcl_files.size())
			{
				cloud->clear();
				myLoadPCDFile(pcl_files[cnt],cloud);
				cnt++;
			}
    }
    else LOG(ERROR)<<"input is't valid";
	// Convert to PLY and save
	toPly();

	//pcl::visualization::PCLVisualizer viewer("viewer");
	//viewer.addPointCloud(cloudPtr, "cloud");
  //viewer.spin();
	return (0);
}
